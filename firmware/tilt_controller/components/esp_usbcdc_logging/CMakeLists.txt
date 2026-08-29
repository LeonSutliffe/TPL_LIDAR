idf_component_register(SRCS "esp_usbcdc_logging.c" INCLUDE_DIRS "."
    # esp_tinyusb: include <tinyusb.h>/<tusb_cdc_acm.h>/<tusb_console.h> -- see the
    # matching note in ../esp_usbcdc_transport/CMakeLists.txt.
    REQUIRES esp_tinyusb
)
