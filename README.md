> [!WARNING]
> this software is work in progress and may be broken in its present state.

# proxy

proxy is a microcontroller firmware acting as a proxy between a real bluetooth human interface device (such as a keyboard, a mouse or a controller) and a computer. it can process their input events before being transmitted to the target pc. only bluetooth low-energy (BLE) is planned to be supported. check out the n1 project website at https://n1.rip/ or join the [discord server](https://discord.gg/PTYAeRdtHR).

## requirements

- [install esp-idf v5.1.2](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/) on your computer and don't forget to [setup the get_idf alias](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html#step-4-set-up-the-environment-variables).
- a supported esp32 chip and usb cable for power. currently supported chips are:
    - esp32s3
- a bluetooth low-energy (BLE) mouse.

## usage

1. clone the repository
```bash
git clone https://github.com/n1rip/proxy && cd proxy
```

2. setup the esp-idf virtual environment
```bash
get_idf
```

3. set the chip target (see supported chips)
```bash
idf.py set-target <chip_name>
```

4. build and flash
```bash
idf.py -p <PORT> flash monitor
```

## troubleshooting

for any technical queries, please open an [issue](https://github.com/n1rip/proxy/issues) on the github repository. we will get back to you soon.
