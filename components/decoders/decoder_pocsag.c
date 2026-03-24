#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "decoder_framework.h"

static const char *TAG = "dec_pocsag";

/* ═══════════════════════════════════════════════════════════════
 *  POCSAG protocol constants
 * ═══════════════════════════════════════════════════════════════ */

#define POCSAG_SYNC_WORD    0x7CD215D8
#define POCSAG_IDLE_WORD    0x7A89C197
#define POCSAG_BATCH_SIZE   16          /* 16 codewords per batch */
#define POCSAG_MAX_CW       128         /* Max codewords in a message */
#define POCSAG_MAX_MSG_LEN  512
#define POCSAG_SYNC_THRESH  2           /* Max bit errors for sync detection */

/* ═══════════════════════════════════════════════════════════════
 *  PLL clock recovery state
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    float phase;            /* 0.0 to 1.0 within current bit */
    float phase_inc;        /* 1.0 / samples_per_bit */
    float gain;             /* PLL bandwidth */
    int prev_sample_sign;   /* Sign of previous sample for zero-cross */
    bool initialized;
} pll_clock_t;

/* ═══════════════════════════════════════════════════════════════
 *  POCSAG frame decoder state
 * ═══════════════════════════════════════════════════════════════ */

typedef enum {
    POCSAG_STATE_PREAMBLE,  /* Looking for alternating 1/0 preamble */
    POCSAG_STATE_SYNC,      /* Looking for sync word */
    POCSAG_STATE_BATCH,     /* Receiving 16 codewords in a batch */
} pocsag_rx_state_t;

typedef struct {
    /* Current message accumulation */
    uint32_t msg_codewords[POCSAG_MAX_CW];
    int msg_cw_count;
    uint32_t msg_ric;       /* RIC (address) for current message */
    int msg_function;       /* Function code 0-3 */
    bool msg_active;        /* Currently accumulating message codewords */
} pocsag_msg_state_t;

typedef struct {
    pocsag_rx_state_t state;
    uint32_t shift_reg;     /* 32-bit shift register for sync detection */
    int bits_in_cw;         /* Bits received for current codeword */
    uint32_t current_cw;    /* Codeword being assembled */
    int cw_index;           /* Index within current batch (0-15) */
    int preamble_bits;      /* Count of alternating preamble bits */
    int prev_bit;           /* Previous bit for preamble detection */
    pocsag_msg_state_t msg;
} pocsag_frame_state_t;

/* ═══════════════════════════════════════════════════════════════
 *  Decoder context (expanded from stub)
 * ═══════════════════════════════════════════════════════════════ */

typedef struct {
    SemaphoreHandle_t mutex;
    bool running;
    uint32_t baud_rate;
    int page_count;

    /* DSP state */
    pll_clock_t pll;
    pocsag_frame_state_t frame;
    uint32_t configured_rate;
} pocsag_ctx_t;

static pocsag_ctx_t s_pocsag_512_ctx  = { .baud_rate = 512 };
static pocsag_ctx_t s_pocsag_1200_ctx = { .baud_rate = 1200 };
static pocsag_ctx_t s_pocsag_2400_ctx = { .baud_rate = 2400 };

/* ═══════════════════════════════════════════════════════════════
 *  BCH(31,21) error correction
 * ═══════════════════════════════════════════════════════════════ */

/* Generator polynomial: x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1 */
#define BCH_POLY 0x769

static uint32_t bch_syndrome(uint32_t codeword) {
    /* Work on bits 31..1 (exclude parity bit 0) */
    uint32_t cw = codeword >> 1;
    for (int i = 30; i >= 10; i--) {
        if (cw & (1u << i)) {
            cw ^= (BCH_POLY << (i - 10));
        }
    }
    return cw & 0x3FF;
}

/* Attempt single-bit error correction. Returns:
 *  0  = no error
 *  1  = corrected single-bit error
 * -1  = uncorrectable error */
static int bch_correct(uint32_t *codeword) {
    uint32_t syn = bch_syndrome(*codeword);
    if (syn == 0) return 0;

    /* Try flipping each of the 31 data+parity bits (bits 31..1) */
    for (int bit = 1; bit <= 31; bit++) {
        uint32_t test = *codeword ^ (1u << bit);
        if (bch_syndrome(test) == 0) {
            *codeword = test;
            return 1;
        }
    }
    return -1;
}

/* Check even parity (bit 0) over entire 32-bit codeword */
static bool bch_parity_ok(uint32_t codeword) {
    uint32_t p = codeword;
    p ^= p >> 16;
    p ^= p >> 8;
    p ^= p >> 4;
    p ^= p >> 2;
    p ^= p >> 1;
    return (p & 1) == 0;
}

/* ═══════════════════════════════════════════════════════════════
 *  Hamming distance for sync word detection
 * ═══════════════════════════════════════════════════════════════ */

static int count_bit_errors(uint32_t a, uint32_t b) {
    uint32_t diff = a ^ b;
    int count = 0;
    while (diff) {
        count += diff & 1;
        diff >>= 1;
    }
    return count;
}

/* ═══════════════════════════════════════════════════════════════
 *  Message decoding
 * ═══════════════════════════════════════════════════════════════ */

static void decode_numeric(const uint32_t *codewords, int count, char *out, int max_len) {
    static const char bcd[] = "0123456789*U -.][";
    int pos = 0;
    for (int i = 0; i < count && pos < max_len - 1; i++) {
        uint32_t data = (codewords[i] >> 1) & 0xFFFFF;
        for (int j = 0; j < 5 && pos < max_len - 1; j++) {
            int digit = (data >> (16 - j * 4)) & 0xF;
            out[pos++] = bcd[digit];
        }
    }
    out[pos] = '\0';
    /* Trim trailing spaces */
    while (pos > 0 && (out[pos - 1] == ' ' || out[pos - 1] == ']')) {
        out[--pos] = '\0';
    }
}

static void decode_alpha(const uint32_t *codewords, int count, char *out, int max_len) {
    /* Collect all 20-bit data fields, extract 7-bit ASCII characters.
     * POCSAG alpha: bits are transmitted LSB first within each codeword's
     * 20-bit data field, and characters are packed continuously across
     * codeword boundaries. */
    int pos = 0;
    int bit_buf = 0;
    int bits_avail = 0;

    for (int i = 0; i < count; i++) {
        uint32_t data = (codewords[i] >> 1) & 0xFFFFF;
        /* Process 20 bits, MSB first as transmitted */
        for (int b = 19; b >= 0; b--) {
            /* Accumulate bits LSB-first for character extraction */
            bit_buf |= ((data >> b) & 1) << bits_avail;
            bits_avail++;
            if (bits_avail == 7) {
                char ch = (char)(bit_buf & 0x7F);
                if (ch >= 32 && ch < 127 && pos < max_len - 1) {
                    out[pos++] = ch;
                } else if (ch == '\n' || ch == '\r') {
                    if (pos < max_len - 1) out[pos++] = ' ';
                }
                bit_buf = 0;
                bits_avail = 0;
            }
        }
    }
    out[pos] = '\0';
    /* Trim trailing spaces */
    while (pos > 0 && out[pos - 1] == ' ') {
        out[--pos] = '\0';
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  Message completion: publish event
 * ═══════════════════════════════════════════════════════════════ */

static void pocsag_emit_message(pocsag_ctx_t *c) {
    pocsag_msg_state_t *m = &c->frame.msg;

    if (!m->msg_active || m->msg_cw_count == 0) return;

    /* Determine message type from function code:
     * Function 0-1: try alpha first, fall back to numeric
     * Function 2-3: numeric */
    char message[POCSAG_MAX_MSG_LEN];
    const char *msg_type;

    if (m->msg_function <= 1) {
        decode_alpha(m->msg_codewords, m->msg_cw_count, message, POCSAG_MAX_MSG_LEN);
        msg_type = "alpha";
        /* If alpha decode produced empty/garbage, try numeric */
        if (strlen(message) == 0) {
            decode_numeric(m->msg_codewords, m->msg_cw_count, message, POCSAG_MAX_MSG_LEN);
            msg_type = "numeric";
        }
    } else {
        decode_numeric(m->msg_codewords, m->msg_cw_count, message, POCSAG_MAX_MSG_LEN);
        msg_type = "numeric";
    }

    xSemaphoreTake(c->mutex, portMAX_DELAY);
    c->page_count++;
    xSemaphoreGive(c->mutex);

    char decoder_name[16];
    snprintf(decoder_name, sizeof(decoder_name), "pocsag_%lu", (unsigned long)c->baud_rate);

    ESP_LOGI(TAG, "[%s] RIC=%lu func=%d type=%s: %s",
             decoder_name, (unsigned long)m->msg_ric, m->msg_function, msg_type, message);

    /* Build event JSON */
    cJSON *data = cJSON_CreateObject();
    if (!data) return;

    cJSON_AddNumberToObject(data, "ric", m->msg_ric);
    cJSON_AddNumberToObject(data, "function", m->msg_function);
    cJSON_AddStringToObject(data, "type", msg_type);
    cJSON_AddStringToObject(data, "message", message);

    /* Key for tracking table and event log */
    char key[32];
    snprintf(key, sizeof(key), "%lu", (unsigned long)m->msg_ric);
    cJSON_AddStringToObject(data, "key", key);

    decode_event_t ev = {
        .decoder_name = decoder_name,
        .event_type = "page",
        .timestamp_ms = (int64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
        .rssi_db = 0,
        .freq_hz = 0,
        .data = data,
    };
    decode_bus_publish(&ev);

    /* Upsert to tracking table */
    tracking_table_t *tt = decoder_get_global_tracking();
    if (tt) {
        cJSON *track = cJSON_CreateObject();
        if (track) {
            cJSON_AddNumberToObject(track, "ric", m->msg_ric);
            cJSON_AddNumberToObject(track, "function", m->msg_function);
            cJSON_AddStringToObject(track, "type", msg_type);
            cJSON_AddStringToObject(track, "message", message);
            tracking_table_upsert(tt, decoder_name, key, track, 0);
            cJSON_Delete(track);
        }
    }

    /* Reset message state */
    m->msg_active = false;
    m->msg_cw_count = 0;
}

/* ═══════════════════════════════════════════════════════════════
 *  Codeword processing within a batch
 * ═══════════════════════════════════════════════════════════════ */

static void pocsag_process_codeword(pocsag_ctx_t *c, uint32_t codeword, int position) {
    pocsag_msg_state_t *m = &c->frame.msg;

    /* Skip idle codewords */
    if (codeword == POCSAG_IDLE_WORD) {
        if (m->msg_active) {
            pocsag_emit_message(c);
        }
        return;
    }

    /* BCH error correction */
    int bch_result = bch_correct(&codeword);
    if (bch_result < 0) {
        ESP_LOGD(TAG, "Uncorrectable BCH error at position %d", position);
        if (m->msg_active) {
            pocsag_emit_message(c);
        }
        return;
    }

    /* Check parity */
    if (!bch_parity_ok(codeword)) {
        ESP_LOGD(TAG, "Parity error at position %d", position);
        if (m->msg_active) {
            pocsag_emit_message(c);
        }
        return;
    }

    /* Bit 31 (MSB): 0 = address codeword, 1 = message codeword */
    bool is_message = (codeword >> 31) & 1;

    if (!is_message) {
        /* Address codeword */
        if (m->msg_active) {
            /* Previous message ended, emit it */
            pocsag_emit_message(c);
        }

        /* Extract address: bits 30..13 (18 bits) form the upper address bits.
         * The full RIC = (address_bits << 3) | (position_in_batch / 2).
         * Each pair of codewords in a batch corresponds to one address frame. */
        uint32_t addr_bits = (codeword >> 13) & 0x3FFFF;
        int frame_pos = position / 2;  /* 0-7: which frame position in batch */
        m->msg_ric = (addr_bits << 3) | frame_pos;
        m->msg_function = (codeword >> 11) & 0x03;
        m->msg_active = true;
        m->msg_cw_count = 0;
    } else {
        /* Message codeword */
        if (m->msg_active && m->msg_cw_count < POCSAG_MAX_CW) {
            m->msg_codewords[m->msg_cw_count++] = codeword;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  Bit-level processing (after clock recovery)
 * ═══════════════════════════════════════════════════════════════ */

static void pocsag_process_bit(pocsag_ctx_t *c, int bit) {
    pocsag_frame_state_t *f = &c->frame;

    /* Always shift into the 32-bit register */
    f->shift_reg = (f->shift_reg << 1) | (bit & 1);

    switch (f->state) {
    case POCSAG_STATE_PREAMBLE:
        /* Detect alternating 1/0 pattern */
        if (bit != f->prev_bit) {
            f->preamble_bits++;
        } else {
            f->preamble_bits = 0;
        }
        f->prev_bit = bit;

        /* After sufficient preamble, look for sync */
        if (f->preamble_bits >= 16) {
            /* Check if we already have a sync word in shift register */
            if (count_bit_errors(f->shift_reg, POCSAG_SYNC_WORD) <= POCSAG_SYNC_THRESH) {
                f->state = POCSAG_STATE_BATCH;
                f->cw_index = 0;
                f->bits_in_cw = 0;
                f->current_cw = 0;
                f->preamble_bits = 0;
                ESP_LOGD(TAG, "POCSAG sync acquired");
            }
        }

        /* Also try direct sync detection without preamble counting */
        if (count_bit_errors(f->shift_reg, POCSAG_SYNC_WORD) <= POCSAG_SYNC_THRESH) {
            f->state = POCSAG_STATE_BATCH;
            f->cw_index = 0;
            f->bits_in_cw = 0;
            f->current_cw = 0;
            f->preamble_bits = 0;
        }
        break;

    case POCSAG_STATE_BATCH:
        /* Assemble codewords, 32 bits each */
        f->current_cw = (f->current_cw << 1) | (bit & 1);
        f->bits_in_cw++;

        if (f->bits_in_cw == 32) {
            pocsag_process_codeword(c, f->current_cw, f->cw_index);
            f->current_cw = 0;
            f->bits_in_cw = 0;
            f->cw_index++;

            if (f->cw_index >= POCSAG_BATCH_SIZE) {
                /* Batch complete, expect next sync word or end */
                f->state = POCSAG_STATE_SYNC;
            }
        }
        break;

    case POCSAG_STATE_SYNC:
        /* After a batch, the next 32 bits should be a sync word */
        /* We already shifted the bit into shift_reg at the top */
        f->bits_in_cw++;

        if (f->bits_in_cw >= 32) {
            if (count_bit_errors(f->shift_reg, POCSAG_SYNC_WORD) <= POCSAG_SYNC_THRESH) {
                /* Another batch follows */
                f->state = POCSAG_STATE_BATCH;
                f->cw_index = 0;
                f->bits_in_cw = 0;
                f->current_cw = 0;
            } else {
                /* No sync found, message transmission ended */
                if (c->frame.msg.msg_active) {
                    pocsag_emit_message(c);
                }
                f->state = POCSAG_STATE_PREAMBLE;
                f->preamble_bits = 0;
                f->bits_in_cw = 0;
            }
        }
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  Lifecycle callbacks
 * ═══════════════════════════════════════════════════════════════ */

static esp_err_t pocsag_init(void *ctx) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    c->mutex = xSemaphoreCreateMutex();
    c->running = false;
    c->page_count = 0;
    c->configured_rate = 0;
    memset(&c->pll, 0, sizeof(c->pll));
    memset(&c->frame, 0, sizeof(c->frame));
    ESP_LOGI(TAG, "POCSAG %lu baud decoder initialized", (unsigned long)c->baud_rate);
    return ESP_OK;
}

static esp_err_t pocsag_start(void *ctx) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    c->running = true;
    return ESP_OK;
}

static esp_err_t pocsag_stop(void *ctx) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    c->running = false;
    c->configured_rate = 0;
    return ESP_OK;
}

static void pocsag_destroy(void *ctx) { (void)ctx; }

/* ═══════════════════════════════════════════════════════════════
 *  Audio processing: FSK comparator -> PLL clock recovery -> bits
 * ═══════════════════════════════════════════════════════════════ */

static void pocsag_process_audio(void *ctx, const int16_t *samples,
                                  uint32_t count, uint32_t sample_rate) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    if (!c->running) return;

    /* Lazy PLL initialization */
    if (c->configured_rate != sample_rate) {
        c->pll.phase = 0.0f;
        c->pll.phase_inc = (float)c->baud_rate / (float)sample_rate;
        c->pll.gain = 0.05f;
        c->pll.prev_sample_sign = 0;
        c->pll.initialized = true;

        /* Reset frame state */
        memset(&c->frame, 0, sizeof(c->frame));
        c->frame.state = POCSAG_STATE_PREAMBLE;

        c->configured_rate = sample_rate;
        ESP_LOGI(TAG, "POCSAG PLL initialized: rate=%lu, baud=%lu, phase_inc=%.6f",
                 (unsigned long)sample_rate, (unsigned long)c->baud_rate, c->pll.phase_inc);
    }

    for (uint32_t i = 0; i < count; i++) {
        /* FSK decision: after FM demod, audio > 0 = mark (1), < 0 = space (0)
         * The audio input represents FM discriminator output, so the instantaneous
         * value directly represents frequency deviation. */
        int sample_sign = (samples[i] > 0) ? 1 : 0;

        /* Zero crossing detection for PLL adjustment */
        if (sample_sign != c->pll.prev_sample_sign) {
            /* Crossing detected: adjust phase toward bit boundary (0.0 or 1.0) */
            float phase_error;
            if (c->pll.phase < 0.5f)
                phase_error = c->pll.phase;
            else
                phase_error = c->pll.phase - 1.0f;
            c->pll.phase -= phase_error * c->pll.gain;
        }
        c->pll.prev_sample_sign = sample_sign;

        /* Advance PLL phase */
        c->pll.phase += c->pll.phase_inc;

        /* Sample bit at center of bit period (phase crosses 1.0) */
        if (c->pll.phase >= 1.0f) {
            c->pll.phase -= 1.0f;
            pocsag_process_bit(c, sample_sign);
        }
    }
}

static void pocsag_process_iq(void *ctx, const uint8_t *iq, uint32_t len) {
    (void)ctx; (void)iq; (void)len;
}

static cJSON *pocsag_get_status(void *ctx) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    cJSON *j = cJSON_CreateObject();
    if (j) {
        cJSON_AddBoolToObject(j, "running", c->running);
        cJSON_AddNumberToObject(j, "baud_rate", c->baud_rate);
        cJSON_AddNumberToObject(j, "page_count", c->page_count);
    }
    return j;
}

static cJSON *pocsag_get_results(void *ctx) {
    pocsag_ctx_t *c = (pocsag_ctx_t *)ctx;
    char name[16];
    snprintf(name, sizeof(name), "pocsag_%lu", (unsigned long)c->baud_rate);
    return decoder_get_global_tracking() ?
        tracking_table_query(decoder_get_global_tracking(), name) :
        cJSON_CreateArray();
}

static decoder_plugin_t s_pocsag_512 = {
    .name = "pocsag_512",
    .description = "POCSAG 512 baud Pager Decoder",
    .category = "pager",
    .demod_type = DEMOD_FSK,
    .center_freq_hz = 152000000,
    .bandwidth_hz = 25000,
    .audio_rate_hz = 22050,
    .demod_params.fsk = { .shift_hz = 4500, .baud = 512 },
    .init = pocsag_init, .start = pocsag_start, .stop = pocsag_stop, .destroy = pocsag_destroy,
    .process_audio = pocsag_process_audio, .process_iq = pocsag_process_iq,
    .get_status = pocsag_get_status, .get_results = pocsag_get_results,
    .ctx = &s_pocsag_512_ctx,
};

static decoder_plugin_t s_pocsag_1200 = {
    .name = "pocsag_1200",
    .description = "POCSAG 1200 baud Pager Decoder",
    .category = "pager",
    .demod_type = DEMOD_FSK,
    .center_freq_hz = 152000000,
    .bandwidth_hz = 25000,
    .audio_rate_hz = 22050,
    .demod_params.fsk = { .shift_hz = 4500, .baud = 1200 },
    .init = pocsag_init, .start = pocsag_start, .stop = pocsag_stop, .destroy = pocsag_destroy,
    .process_audio = pocsag_process_audio, .process_iq = pocsag_process_iq,
    .get_status = pocsag_get_status, .get_results = pocsag_get_results,
    .ctx = &s_pocsag_1200_ctx,
};

static decoder_plugin_t s_pocsag_2400 = {
    .name = "pocsag_2400",
    .description = "POCSAG 2400 baud Pager Decoder",
    .category = "pager",
    .demod_type = DEMOD_FSK,
    .center_freq_hz = 152000000,
    .bandwidth_hz = 25000,
    .audio_rate_hz = 22050,
    .demod_params.fsk = { .shift_hz = 4500, .baud = 2400 },
    .init = pocsag_init, .start = pocsag_start, .stop = pocsag_stop, .destroy = pocsag_destroy,
    .process_audio = pocsag_process_audio, .process_iq = pocsag_process_iq,
    .get_status = pocsag_get_status, .get_results = pocsag_get_results,
    .ctx = &s_pocsag_2400_ctx,
};

void register_pocsag_decoders(void) {
    decoder_registry_add(&s_pocsag_512);
    decoder_registry_add(&s_pocsag_1200);
    decoder_registry_add(&s_pocsag_2400);
}
