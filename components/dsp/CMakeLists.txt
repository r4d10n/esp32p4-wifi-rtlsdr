idf_component_register(
    SRCS "dsp.c" "pie_kernels.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer espressif__esp-dsp
)
