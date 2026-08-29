idf_component_register(SRCS "esp_usbcdc_transport.c"
    INCLUDE_DIRS "."
    # micro_ros_espidf_component: include <uxr/client/transport.h>
    # esp_tinyusb: include <tinyusb.h>/<tusb_cdc_acm.h> -- this IDF version (5.3.2)
    # dropped the legacy in-tree tinyusb component in favor of the component-manager
    # one; the upstream example this file was copied from didn't need this REQUIRES
    # line on whatever older IDF version it was written against.
    REQUIRES micro_ros_espidf_component esp_tinyusb
)
