MicroRDK module for the [TCS3200 Color Sensor](https://www.amazon.com/TCS230-TCS3200-Detector-Recognition-Arduino/dp/B0190PZK46).

To use:

1. Follow the wiring tutorial [here](https://www.electronicwings.com/esp32/tcs3200-color-sensor-interfacing-with-esp32) to connect the sensor to your ESP32.
2. [Build custom MicroRDK firmware](https://docs.viam.com/operate/get-started/other-hardware/micro-module/#build-custom-firmware) that includes this module. Sample Cargo.toml for such a project:

```
[package]
name = "esha-test-project"
version = "0.0.1"
edition = "2021"
authors = ["esha.maharishi <esha.maharishi@viam.com>"]
resolver = "2"
rust-version = "1.83"

[profile.release]
debug = true
opt-level = "z"

[profile.dev]
debug = true
opt-level = "z"

[dependencies.micro-rdk]
git = "https://github.com/gvaradarajan/micro-rdk.git"
branch = "falling_edge"
#version = "0.4.1-rc10"
#rev = "v0.4.1-rc10"
features = [
  "esp32",
  "binstart",

]

[dependencies.micro-rdk-modular-driver-example]
git = "https://github.com/gvaradarajan/micro-rdk.git"
branch = "falling_edge"
#version = "0.4.1-rc10"
#rev = "v0.4.1-rc10"
features = ["esp32"]

[dependencies]
async-channel = "2"
embedded-hal = { version = "~0.2", features = ["unproven"]}
embedded-svc = "0.27"
futures-lite = "2"
log = "0.4"
color-detector = { git = "https://github.com/EshaMaharishi/color-detector.git" }

[build-dependencies]
cargo_metadata = "0.19"
embuild = "0.32"
regex = "1.11"
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"

[package.metadata.esp-idf-sys]
# TODO(RSDK-8998): Upgrade ESP-IDF to latest stable
esp_idf_version = "v4.4.8"
esp_idf_tools_install_dir = "custom:target/.embuild/espressif"
esp_idf_sdkconfig_defaults = [
  "sdkconfig.defaults",
]
```

3. Add the 'esp32' model of Board and this color detector to your Viam machine's config. Example Viam JSON configuration:

```
{
  "components": [
    {
      "name": "color_detector",
      "namespace": "rdk",
      "type": "sensor",
      "model": "my_sensor",
      "attributes": {
        "interrupt_pin": 25
      },
      "depends_on": [
        "board-1"
      ]
    },
    {
      "name": "board-1",
      "api": "rdk:component:board",
      "model": "rdk:builtin:esp32",
      "attributes": {
        "digital_interrupts": [
          {}
        ],
        "pins": [
          23,
          22,
          25
        ]
      }
    }
  ]
}
```
