set(COMPONENT_SRCS "espcam2640.c" "tcpserver.c")

set(COMPONENT_REQUIRES
    esp32-camera-master
    nvs_flash
    )

set(COMPONENT_EMBED_FILES "www/index_ov2640.html.gz")

register_component()
