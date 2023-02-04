idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(
                    SRCS "sx1280_rx.c"

                    INCLUDE_DIRS "."
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
