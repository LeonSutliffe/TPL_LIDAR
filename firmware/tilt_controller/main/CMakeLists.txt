idf_component_register(
    SRCS "main.c" "mks_servo42d.c" "tilt_axis.c"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component driver esp_timer esp_usbcdc_transport esp_usbcdc_logging
)
