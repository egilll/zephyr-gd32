This is a temporary fork of [Zephyr](https://github.com/zephyrproject-rtos/zephyr) with a few work-in-progress commits for GigaDevice chips.

It includes: 

- SDHC – works but is missing non-DMA paths and doesn't work if the user supplies non-DMA accessible buffers
- Ethernet
  - Requires you to apply the changes shown on the `enet` branch in [this hal_gigadevice fork](https://github.com/egilll/hal_gigadevice_tmp_fork)
- USBFS/USBHS – Does currently mostly work and uses Zephyr's dwc2 driver, but requires a workaround since for some reason GigaDevice isn't exposing some dwc2 config registers.

