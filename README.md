This is a temporary fork of [Zephyr](https://github.com/zephyrproject-rtos/zephyr) with a few work-in-progress commits for GigaDevice chips.

It includes:

- GD32 support:
  - SDHC ([`pr/sdhc/gd32-kinetis-drivers`](../../tree/pr/sdhc/gd32-kinetis-drivers))
  - Ethernet ([`pr/ethernet/dwmac-gd32`](../../tree/pr/ethernet/dwmac-gd32))
  - USB and UAC1 audio ([`pr/usb/dwc2-gd32`](../../tree/pr/usb/dwc2-gd32), [`pr/usb/uac1-device-class`](../../tree/pr/usb/uac1-device-class))
  - SoC, devicetree, clock, DMA, and peripheral-driver support ([`pr/soc/gd32-peripheral-devicetree`](../../tree/pr/soc/gd32-peripheral-devicetree), [`pr/clock-control/gd32-driver`](../../tree/pr/clock-control/gd32-driver), [`pr/dma/gd32-driver`](../../tree/pr/dma/gd32-driver), [`pr/adc/gd32-driver`](../../tree/pr/adc/gd32-driver), [`pr/i2c/gd32-driver`](../../tree/pr/i2c/gd32-driver), [`pr/i2s/gd32-driver`](../../tree/pr/i2s/gd32-driver), [`pr/spi/gd32-driver`](../../tree/pr/spi/gd32-driver), [`pr/serial/gd32-driver`](../../tree/pr/serial/gd32-driver), [`pr/rtc/gd32-mcux-drivers`](../../tree/pr/rtc/gd32-mcux-drivers))
- NXP:
  - SDHC ([`pr/sdhc/gd32-kinetis-drivers`](../../tree/pr/sdhc/gd32-kinetis-drivers))
  - FRDM-K66F board, Kinetis SoC, clock, serial, I2C, and RTC support ([`pr/boards/frdm-k66f`](../../tree/pr/boards/frdm-k66f), [`pr/soc/kinetis-k66`](../../tree/pr/soc/kinetis-k66), [`pr/clock-control/kinetis-sim`](../../tree/pr/clock-control/kinetis-sim), [`pr/serial/mcux-driver`](../../tree/pr/serial/mcux-driver), [`pr/i2c/mcux-50khz`](../../tree/pr/i2c/mcux-50khz), [`pr/rtc/gd32-mcux-drivers`](../../tree/pr/rtc/gd32-mcux-drivers))
- Other:
  - DNS-SD, mDNS, and DHCPv4 client improvements ([`pr/net/dns-sd-hostname`](../../tree/pr/net/dns-sd-hostname), [`pr/net/mdns-address-lifecycle`](../../tree/pr/net/mdns-address-lifecycle), [`pr/net/dhcpv4-client-identifier`](../../tree/pr/net/dhcpv4-client-identifier))
  - Default filesystem mounts ([`pr/fs/default-mount`](../../tree/pr/fs/default-mount))
  - Link-time code relocation ([`pr/cmake/code-relocation-lto`](../../tree/pr/cmake/code-relocation-lto))
