# Rev A3 bench diagnostic

Date: 2026-11-17

Board: Aerial Kit ELRS USB TX Rev A3 prototype

MCU: ESP32-S3 QFN56 revision 0.2, MAC `90:70:69:28:56:e4`

Flash: 8 MiB XMC, manufacturer `0x20`, device `0x4017`

Security: secure boot and flash encryption disabled

RF state during test: both RFX2401C enables held low; diagnostic did not
command an SX1280 transmission

Observed results:

```text
DIAG usb=ok uart0_tx=43 uart0_rx=44
DIAG role_gpio4=low button_gpio0=released
DIAG straps gpio3=0 gpio45=0 gpio46=0
DIAG breakout gpio35=0 gpio36=0 gpio37=0 gpio38=0
DIAG fem txen_high readback=1
DIAG fem rxen_high readback=1
DIAG fem txen=0 rxen=0 final=shutdown
DIAG sx1280 spi=ok version=0xA9B7 busy=0 dio1=0
DIAG i2c j3 SDA=34 SCL=48: no devices
DIAG ws2812 gpio21=red at 26/255
```

This historical diagnostic predates the final role correction. The printed
switch labels are now authoritative: high GPIO4 is TX and low GPIO4 is RX.
Use a power-off continuity check only to verify the switch wiring.

Not tested: RF path, FEM gain or output, antenna VSWR, receive sensitivity,
binding, link acquisition and range. These require an antenna or terminated
50 ohm RF path.
