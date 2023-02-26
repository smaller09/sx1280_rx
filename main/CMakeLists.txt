idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(
                    SRCS "sx1280_rx.c"
                    SRCS "sx1280.c"
                    SRCS "ota.c"
                    SRCS "serial.c"
                    INCLUDE_DIRS "."
)
