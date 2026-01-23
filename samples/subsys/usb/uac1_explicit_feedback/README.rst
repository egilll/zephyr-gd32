.. zephyr:code-sample:: uac1-explicit-feedback
   :name: USB Audio Class 1 explicit feedback sample
   :relevant-api: usbd_api uac1_device i2s_interface

Overview
********

This sample demonstrates USB Audio Class 1 playback (Host -> Device) using an
isochronous OUT endpoint and an explicit feedback endpoint.

Building and Running
********************

The code can be found in :zephyr_file:`samples/subsys/usb/uac1_explicit_feedback`.

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/usb/uac1_explicit_feedback
   :board: gd32f450z_eval
   :goals: build
   :compact:

