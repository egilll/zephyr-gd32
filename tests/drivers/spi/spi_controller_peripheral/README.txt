In this test suite two instances of the SPI peripheral are connected together.
One SPI instance works as a controller, second one is configured as a peripheral.
In each test, both instances get identical configuration (CPOL, CPHA, bitrate, etc.).

Four GPIO loopbacks are required (see overlay for nrf54l15dk for reference):
1. spi22-SPIM_SCK connected with spi21-SPIS_SCK,
2. spi22-SPIM_MISO connected with spi21-SPIS_MISO,
3. spi22-SPIM_MOSI connected with spi21-SPIS_MOSI,
4. spi22-cs-gpios connected with spi21-SPIS_CSN.

On gd32f450z_eval, the test uses SPI3 as controller and SPI4 as peripheral. Wire:
1. JP6-3 SPI3_CS PE4 to JP6-16 SPI4_NSS PF6,
2. JP6-1 SPI3_SCK PE2 to JP6-17 SPI4_SCK PF7,
3. JP6-4 SPI3_MISO PE5 to JP6-18 SPI4_MISO PF8,
4. JP6-5 SPI3_MOSI PE6 to JP6-19 SPI4_MOSI PF9.

On gd32f450z_eval, the generic suite is narrowed at runtime to a GD32-specific subset.
Non-DMA full-duplex cases are skipped. The intended GD32 coverage is:
1. controller-TX to slave-RX,
2. DMA full-duplex when the GD32 DMA overlay/configuration is enabled.

The board overlay also disables SPI5 for this test so only the SPI3/SPI4 loopback path is active.
