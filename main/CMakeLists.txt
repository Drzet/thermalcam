idf_component_register(SRCS "app_main.c" "sensor.c" "thermal.c"
    INCLUDE_DIRS "."
    REQUIRES mlx90640 esp_driver_i3c esp_driver_gpio esp_driver_jpeg esp_timer esp_psram)
