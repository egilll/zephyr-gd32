.. zephyr:board:: frdm_k66f

Overview
********

The FRDM-K66F is an NXP Freedom development board built around the
``MK66FN2M0VMD18`` Kinetis K66 MCU.

The board provides:

- A Cortex-M4F MCU running at up to 180 MHz
- 2 MiB of flash and 256 KiB of SRAM
- OpenSDA debug and UART console connectivity
- On-board Ethernet PHY
- A microSD socket wired to ``SDHC0``
- An RGB LED and two user buttons

Hardware
********

- ``MK66FN2M0VMD18`` MCU
- 12 MHz main crystal
- OpenSDAv2.1 debug interface
- 10/100 Ethernet PHY
- microSD card slot
- RGB LED
- User buttons ``SW2`` and ``SW3``

Supported Features
==================

.. zephyr:board-supported-hw::

Connections And IOs
===================

+--------+----------------+-------------------+
| Name   | Function       | Usage             |
+========+================+===================+
| PTB16  | UART0_RX       | Console RX        |
+--------+----------------+-------------------+
| PTB17  | UART0_TX       | Console TX        |
+--------+----------------+-------------------+
| PTC9   | GPIO           | RGB LED red       |
+--------+----------------+-------------------+
| PTE6   | GPIO           | RGB LED green     |
+--------+----------------+-------------------+
| PTA11  | GPIO           | RGB LED blue      |
+--------+----------------+-------------------+
| PTD11  | GPIO           | User button SW2   |
+--------+----------------+-------------------+
| PTA10  | GPIO           | User button SW3   |
+--------+----------------+-------------------+
| PTD10  | GPIO           | microSD card-detect |
+--------+----------------+-------------------+
| PTE0   | SDHC0_D1       | microSD DAT1      |
+--------+----------------+-------------------+
| PTE1   | SDHC0_D0       | microSD DAT0      |
+--------+----------------+-------------------+
| PTE2   | SDHC0_DCLK     | microSD CLK       |
+--------+----------------+-------------------+
| PTE3   | SDHC0_CMD      | microSD CMD       |
+--------+----------------+-------------------+
| PTE4   | SDHC0_D3       | microSD DAT3      |
+--------+----------------+-------------------+
| PTE5   | SDHC0_D2       | microSD DAT2      |
+--------+----------------+-------------------+
| PTA5   | RMII0_RXER     | Ethernet          |
+--------+----------------+-------------------+
| PTA12  | RMII0_RXD1     | Ethernet          |
+--------+----------------+-------------------+
| PTA13  | RMII0_RXD0     | Ethernet          |
+--------+----------------+-------------------+
| PTA14  | RMII0_CRS_DV   | Ethernet          |
+--------+----------------+-------------------+
| PTA15  | RMII0_TXEN     | Ethernet          |
+--------+----------------+-------------------+
| PTA16  | RMII0_TXD0     | Ethernet          |
+--------+----------------+-------------------+
| PTA17  | RMII0_TXD1     | Ethernet          |
+--------+----------------+-------------------+
| PTB0   | RMII0_MDIO     | Ethernet MDIO     |
+--------+----------------+-------------------+
| PTB1   | RMII0_MDC      | Ethernet MDC      |
+--------+----------------+-------------------+
| PTE26  | ENET_1588_CLKIN| Ethernet RMII ref |
+--------+----------------+-------------------+

System Clock
============

This board uses the on-board 12 MHz crystal and boots the K66 into PEE mode
at 180 MHz.

The board-specific ``clock_init.c`` also routes the SDHC source clock to
``OSCERCLK``. With the current board support this limits the SDHC bus clock to
6 MHz, so the devicetree ``sdhc0`` node advertises ``max-bus-freq = 6 MHz``.

Programming And Debugging
*************************

.. zephyr:board-supported-runners::

Use the usual Zephyr workflow:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: frdm_k66f
   :goals: build flash

Serial Console
==============

The default console uses the OpenSDA virtual COM port at 115200 8N1.
