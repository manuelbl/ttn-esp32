set(COMPONENT_SRCDIRS
    "src"
    "src/aes"
    "src/hal"
    "src/lmic"
)
set(COMPONENT_ADD_INCLUDEDIRS
    "include"
)
set(COMPONENT_REQUIRES
    nvs_flash
)

register_component()
