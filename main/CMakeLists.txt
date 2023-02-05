idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(
                    SRCS "sx1280_rx.c"
                    SRCS "sx1280.c"
                    SRCS "serial.c"
                    SRCS "ota.c"
                    INCLUDE_DIRS "."
)
