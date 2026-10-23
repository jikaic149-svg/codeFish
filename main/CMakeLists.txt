idf_component_register(
    SRCS "robotfish.cpp" "network.cpp" "bsp.cpp"
         "BSP/LoRaUart.cpp" "BSP/ImuUart.cpp"
         "Service/LoRa/LoRaService.cpp" "Service/LoRa/LoraProtocol.cpp"
       "Service/Wifi/ControlProtocol.cpp"
         "Mode/Motion/MotionMode.cpp"
         "Device/Actuator/TailMotion.cpp"
         "Device/Actuator/PropulsionMotion.cpp"
         "Device/Actuator/PectoralMotion.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES app_update esp_app_format esp_driver_gpio esp_driver_ledc esp_driver_uart
                  esp_http_server esp_netif esp_timer esp_wifi
                  json mbedtls nvs_flash
    EMBED_TXTFILES "control.html")
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
option(ROBOTFISH_USB_DIAGNOSTICS "Read-only USB telemetry for wired sensor tests" OFF)
if(ROBOTFISH_USB_DIAGNOSTICS)
    target_sources(${COMPONENT_LIB} PRIVATE "Service/Usb/UsbTelemetry.cpp")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ROBOTFISH_USB_DIAGNOSTICS=1)
endif()
