# **!!! DEPRECATED !!!**
**As an alternative I recommend using [ESPHome](https://esphome.io/components/prometheus.html)**

# esp32 ds18b20 prometheus exporter

A simple program that exposes temperature readings from all connected ds18b20 temperature sensors in [prometheus format](https://prometheus.io/docs/instrumenting/exposition_formats/#text-based-format)

By default the server listens on pot 80 and exposes data at `/metrics`

This project expects the sensor(s) to be connected to pin `4`. If you want to use a diffrent pin make sure to edit the `GPIO_DS18B20_0` define in [main.c](./main/main.c#L16)

# Compiling

This project has dependencies in the form of submodules.  
Make sure to clone the repository with the `--recursive` flag to download them

To build this project you will need the [ESP-IDF toolchain](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started)  
I recommend using [Visual studio code](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/vscode-setup.html) as it make the entire process much easier.

When configuring the project make sure to put your WI-FI credentials in the `Wifi configuration` section

# Dependencies 

This project uses [esp32-ds18b20](https://github.com/DavidAntliff/esp32-ds18b20/) and [esp32-owb](https://github.com/DavidAntliff/esp32-owb)

# License

The code in this project is licensed under the MIT license - see [LICENSE](./LICENSE) for details.
