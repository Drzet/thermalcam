idf_component_register(SRCS "app_main.c" "sensor.c" "thermal.c" "uvc_mode_control.c"
    INCLUDE_DIRS "."
    REQUIRES mlx90640 esp_driver_i3c esp_driver_gpio esp_driver_jpeg esp_timer esp_psram espressif__tinyusb)

# TinyUSB 0.19 exposes no Processing Unit callback. Intercept only the
# descriptor and VC control-request entry points, leaving streaming untouched.
target_link_options(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=tud_descriptor_configuration_cb"
    "-Wl,--wrap=videod_control_xfer_cb")
