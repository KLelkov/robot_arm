# Robot arm project

The code version on the branch `master` is designed for Makerbase MKS DLC32 board (designed for 3d printers and CNC machines).

### MKS pinout

I haven't found the exact pinout for my board online, so here is the quick summary

| Pin group           | Pin function | Pin  |
| :------------------ | :----------- | :--- |
| Common for all axes | Enable       | 8    |
| X axis control      | X axis DIR   | 15   |
| X axis control      | X axis STEP  | 16   |
| X axis control      | X axis LIMIT | 39   |
| Y axis control      | Y axis DIR   | 6    |
| Y axis control      | Y axis STEP  | 7    |
| Y axis control      | Y axis LIMIT | 40   |
| Z axis control      | Z axis DIR   | 4    |
| Z axis control      | Z axis STEP  | 5    |
| Z axis control      | Z axis LIMIT | 41   |
| A axis control      | A axis DIR   | 19   |
| A axis control      | A axis STEP  | 20   |
| A axis control      | A axis LIMIT | ?    |
| LEDs                | Air LED      | 1    |
| LEDs                | Laser LED    | 2    |
| Utility             | Flame        | 36   |
| Utility             | Probe        | 37   |
| Utility             | Door         | 38   |

## Serial communication for TMC2209

These advanced drivers support serial interface for configuration and control. However, on the MKS board the drivers are supplied with 5V, making their logic levels incompatible with ESP's 3.3V logic. There are two possible solutions:

- add the logic shifter board, although this might be tricky due to a single-wire uart communication line on TMC2209 drivers.

- cut the power supply pins from the drivers and wire them directly to ESP's 3.3V

Either way you will need to solder some wires directly to the ESP32 chip's unused pins (13, 14 for example) because the MKS board doesnt have serial connections traced.

> Or you can forgo the serial connection and hardware-configure the drivers microsteps and current with onboard switches and potentiometers. This is the simplest way to get started and that's exactly what I did.

