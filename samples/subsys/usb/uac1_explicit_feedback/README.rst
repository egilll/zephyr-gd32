.. zephyr:code-sample:: uac1-explicit-feedback
   :name: USB Audio Class 1 explicit feedback sample
   :relevant-api: usbd_api uac1_device i2s_interface

Overview
********

This sample demonstrates USB Audio Class 1 playback (Host -> Device) using an
isochronous OUT endpoint and an explicit feedback endpoint.

The asynchronous stream uses a 1 ms Full-Speed data interval
(``bInterval=1``) and feedback interval (``bRefresh=0``). The feedback value
uses a low-pass filtered buffer level and bounded proportional correction. On
GD32, its baseline comes from the frame rate actually produced by PLLI2S rather
than the requested rate.
The AudioControl interface exposes master and per-channel mute and volume
controls. Host volume changes are applied to the 16-bit PCM stream in software
before it is sent to I2S. Gain changes are ramped over at most 10 ms to avoid
clicks caused by discontinuities when changing volume or mute state. The
optional AudioControl status endpoint is also enabled, so device-side control
changes can be reported to the host.

USB packet boundaries are decoupled from I2S DMA boundaries by a PCM FIFO. On
GD32, fixed-size I2S blocks use the DMA controller's two-memory switch-buffer
mode, allowing the inactive buffer to be replaced while the other buffer is
still being transmitted instead of stopping and restarting DMA for every USB
packet.

Building and Running
********************

The code can be found in :zephyr_file:`samples/subsys/usb/uac1_explicit_feedback`.

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/usb/uac1_explicit_feedback
   :board: gd32f450z_eval
   :goals: build
   :compact:

On Windows, ``windows_playback_test.ps1`` directly selects the UAC1 waveOut
endpoint and plays a generated stereo test sequence. For example::

   powershell -ExecutionPolicy Bypass -File windows_playback_test.ps1 -Seconds 60

Add ``-ExerciseVolume`` to lower, mute, and restore the endpoint during the
test.
