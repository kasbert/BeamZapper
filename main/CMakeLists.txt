idf_component_register(SRCS 
    "beamzapper.c" 
    "beamsync.c" 
    "esp_hid_wiimote.c" 
    "esp_hid_gap.c"
    "libcwiid/command.c"
    "libcwiid/process.c"
    "libcwiid/state.c"
    "libcwiid/util.c"
    INCLUDE_DIRS "." "libcwiid"
    REQUIRES driver esp_hid nvs_flash esp_driver_rmt)
