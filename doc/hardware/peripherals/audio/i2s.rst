.. _i2s_api:

Inter-IC Sound (I2S) Bus
########################

Overview
********

The I2S (Inter-IC Sound) API provides support for the standard I2S interface
as well as common non-standard extensions such as PCM Short/Long Frame Sync
and Left/Right Justified Data Formats.

Drivers may optionally report stream progress and the effective frame rate
with :c:func:`i2s_timing_get`. The frame counter and timestamp form one atomic
snapshot taken at a transfer-completion event. The timestamp is in the system
hardware cycle-counter domain and need not correspond exactly to an I2S clock
edge.

Configuration Options
*********************

Related configuration options:

* :kconfig:option:`CONFIG_I2S`

API Reference
*************

.. doxygengroup:: i2s_interface
