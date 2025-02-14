idf_component_register(SRCS 
                        "main.c"
                        "rlcd_lib/ls012b7dd06.c"
                        "rlcd_lib/ls012b7dd06_hal.c"
                        "rlcd_lib/graphics.c"
                        "rlcd_lib/i2s_parallel_driver/i2s_parallel.c"
                        "rlcd_lib/rmt/rlcd_gck_encoder.c"
                        "rlcd_lib/rmt/rlcd_gsp_encoder.c"
                        "rlcd_lib/rmt/rlcd_intb_encoder.c"
                       INCLUDE_DIRS "." "rlcd_lib")
