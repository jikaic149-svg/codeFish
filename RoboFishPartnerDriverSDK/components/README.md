# Local ESP-IDF components

The camera service uses Espressif's official `esp32-camera` component. It is
declared in `main/idf_component.yml`; ESP-IDF Component Manager resolves the
exact versions recorded in the project `dependencies.lock` and places its
generated working copies under `managed_components/`.

The resolved and build-verified versions are:

```text
espressif/esp32-camera 2.1.7
espressif/esp_jpeg     1.3.1
ESP-IDF                5.3.2
```

`managed_components/` is generated and intentionally not source-controlled.
Keep `main/idf_component.yml` and `dependencies.lock` in source control so a
connected build can reproduce the same dependency set.

For a completely offline build on a machine where Component Manager has no
cache, an already obtained official component can instead be placed here so
that these files exist:

```text
components/esp32-camera/CMakeLists.txt
components/esp32-camera/driver/include/esp_camera.h
components/esp32-camera/conversions/include/img_converters.h
```

The main component detects either the managed component or that local
directory, enables the real XIAO ESP32-S3 Sense implementation and links the
camera target plus `idf::esp_driver_sdm`. If neither is available,
`CameraService::begin()` returns `ESP_ERR_NOT_SUPPORTED` and camera HTTP
handlers return 503; the rest of the firmware remains buildable.

Do not copy headers alone. The driver sources, sensor implementations,
converters and the component's own CMake metadata are all required.
