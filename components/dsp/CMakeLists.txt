idf_component_register(
    SRCS "dsp.c" "pie_kernels.c" "pie_power_spectrum_arp4.S" "pie_nco_mix_arp4.S"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer espressif__esp-dsp
)
