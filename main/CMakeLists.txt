idf_component_register(
    SRCS "robotfish.cpp" "network.cpp" "bsp.cpp"
         "Service/Wifi/ControlProtocol.cpp"
         "Mode/Motion/MotionMode.cpp"
         "Device/Actuator/TailMotion.cpp"
         "Device/Actuator/PropulsionMotion.cpp"
         "Device/Actuator/PectoralMotion.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES app_update esp_app_format esp_driver_gpio esp_driver_ledc
                  esp_http_server esp_netif esp_timer esp_wifi
                  json mbedtls nvs_flash
    EMBED_TXTFILES "control.html")
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
