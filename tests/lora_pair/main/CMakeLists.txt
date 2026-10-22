idf_component_register(SRCS "bridge.c" INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_uart esp_driver_usb_serial_jtag esp_driver_gpio)
