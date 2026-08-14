set(ROBOFISH_HAS_ESP32_CAMERA 0)
if("esp32-camera" IN_LIST BUILD_COMPONENTS OR
   EXISTS "${CMAKE_CURRENT_LIST_DIR}/../components/esp32-camera/CMakeLists.txt")
    set(ROBOFISH_HAS_ESP32_CAMERA 1)
elseif("espressif__esp32-camera" IN_LIST BUILD_COMPONENTS OR
       EXISTS "${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp32-camera/CMakeLists.txt")
    set(ROBOFISH_HAS_ESP32_CAMERA 1)
endif()

idf_component_register(
    SRCS
        "app_main.cpp"
        "../src/app/RoboFishRuntime.cpp"
        "../src/bsp/ServoPwm.cpp"
        "../src/bsp/RgbStatusStrip.cpp"
        "../src/camera/CameraService.cpp"
        "../src/control/MotionController.cpp"
        "../src/device/Propulsion.cpp"
        "../src/device/PumpOutput.cpp"
        "../src/device/RoboFishDriver.cpp"
        "../src/device/SideRudder.cpp"
        "../src/device/TailServos.cpp"
        "../src/partner/PartnerScriptEngine.cpp"
        "../src/service/OtaService.cpp"
        "../src/web/WebAssets.cpp"
        "../src/web/ProductWebService.cpp"
    INCLUDE_DIRS
        "../src"
    PRIV_REQUIRES
        app_update
        esp_driver_gpio
        esp_driver_ledc
        esp_driver_rmt
        esp_http_server
        esp_netif
        esp_timer
        esp_wifi
        mbedtls
        nvs_flash
    EMBED_TXTFILES
        "../../reverse_analysis/web_assets/html_01_0x0000100b.html"
        "../../reverse_analysis/web_assets/html_02_0x0001f6ad.html"
        "../../reverse_analysis/web_assets/manifest_01_0x00000d57.json"
        "../../reverse_analysis/web_assets/service_worker_01_0x00000bc5.js"
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
target_compile_definitions(
    ${COMPONENT_LIB} PRIVATE
    RF_HAS_ESP32_CAMERA=${ROBOFISH_HAS_ESP32_CAMERA}
)

if(ROBOFISH_HAS_ESP32_CAMERA)
    if(TARGET idf::esp32-camera)
        target_link_libraries(${COMPONENT_LIB} PRIVATE idf::esp32-camera)
    elseif(TARGET idf::espressif__esp32-camera)
        target_link_libraries(${COMPONENT_LIB} PRIVATE idf::espressif__esp32-camera)
    else()
        message(FATAL_ERROR
            "esp32-camera was detected but no ESP-IDF component target exists")
    endif()
    target_link_libraries(${COMPONENT_LIB} PRIVATE idf::esp_driver_sdm)
endif()
