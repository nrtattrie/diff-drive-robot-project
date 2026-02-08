# Bill of Materials

Full parts list for the differential drive robot. The canonical BOM is maintained in [Google Drive](https://drive.google.com/drive/folders/1yRn0lZib179Eu5fszfl0arMyE6idsfoR); this file mirrors it for quick reference.

**Status key:** Own = purchased/in-hand | Planned = not yet sourced

## Electronics

| Item | Qty | Unit Cost | Source | Status | Notes |
|------|-----|-----------|--------|--------|-------|
| Raspberry Pi 5 (8GB) | 1 | $115.92 | Amazon | Own | Main computer |
| RPi 5 Power Supply | 1 | $14.46 | Vilros | Own | 27W USB-C, 5.1V, 5A |
| M.2 HAT+ | 1 | $23.99 | Amazon | Own | Intermediary between RPi 5 and NVMe for SSD |
| Active Cooler | 1 | $11.89 | Amazon | Own | Thermal management |
| Raspberry Pi NVMe SSD | 1 | $89.99 | Vilros | Own | Ubuntu boot drive |
| 64 GB SSD | 1 | $0.00 | Office | Own | For practicing booting and Linux environment before NVMe SSD arrives |
| ELEGOO Sensor Kit | 1 | $50.87 | Amazon | Own | Miscellaneous sensors |
| RPi Camera Module 3 (Standard) | 1 | $40.00 | Vilros | Own | 12-megapixel Sony IMX708 image sensor module |
| Arduino Mega 2560 | 1 | $0.00 | Elegoo Kit | Own | Motor controller |
| Arduino Mega Prototype Shield V3 | 1 | $0.00 | Elegoo Kit | Own | For wiring and pass through of the motor controller |
| Pololu G2 High-Power Motor Driver 18v17 | 1 | $44.95 | Pololu | Own | Dual motor driver, 6.5V-30V, 17A |
| Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 - STEMMA QT / Qwiic | 1 | $29.95 | Adafruit | Own | 9-DOF sensor, MEMS accelerometer, magnetometer, and gyroscope on a single die with ARM Cortex-M0 processor |
| DROK DC Buck Converter, 5.3V-32V to 1.2V-32V 12A Adjustable Power Supply | 1 | $17.49 | Amazon | Own | Adjustable voltage buck converter for the RPi 5. Set adjustable instead of fixed 5V because RPi 5 can brown out at marginally less than 5V |

## Mechanical Components

| Item | Qty | Unit Cost | Source | Status | Notes |
|------|-----|-----------|--------|--------|-------|
| 50:1 Metal Gearmotor 37Dx70L mm 12V with 64 CPR Encoder (Helical Pinion) | 2 | $70.95 | Pololu | Own | 12V brushed DC motor with 16mm long, 6mm diameter D-shaped shaft, and encoder |
| Pololu Wheel 80x10mm Pair - Black | 1 | $8.75 | Pololu | Own | Silicone tires, 80mm (3.15") diameter, press-fit onto 3mm D shafts |
| Pololu Universal Aluminum Mounting Hub for 6mm Shaft, #4-40 Holes (2-Pack) | 1 | $12.95 | Pololu | Own | Universal aluminum mounting hubs for custom wheels and mechanisms on 6mm motor shafts |
| Pololu Ball Caster with 3/4" Plastic Ball | 1 | $5.49 | Pololu | Own | Light ball caster, assembled height ranges from 0.9" to 1.2" depending on spacer combination |
| Mini Panel Mount SPDT Toggle Switch | 1 | $0.95 | Adafruit | Own | Power on/off toggle switch, up to 1A at 24V recommended |
| TBD Chassis | 1 | $0.00 | 3D Printed | | |

## Wiring and Connectors

| Item | Qty | Unit Cost | Source | Status | Notes |
|------|-----|-----------|--------|--------|-------|
| JST SH-Style Cable, 5-Pin, Female-Female, 16cm | 3 | $3.06 | Pololu | Own | 5-conductor, 16cm (6.3") cable with 28 AWG wires and 5-pin female JST SH-type connectors (1mm pitch) on both ends |
| JST SH-Style Cable, 5-Pin, Single-Ended Female, 12cm | 3 | $1.77 | Pololu | Own | 5-conductor, 12cm (4.5") cable with 28 AWG wires; one end JST SH connector, other end unterminated for custom length |
| 2x20 Extra Tall Female 0.1" Pitch Stacking Header | 8 | $9.79 | Amazon | Own | 23mm tall header that passes through the M.2 HAT+ |
| STEMMA QT / Qwiic JST SH 4-pin Cable - 100mm Long | 3 | $0.95 | Adafruit | Own | JST-SH female 4-pin connectors on both ends |
| STEMMA QT / Qwiic JST SH 4-Pin Cable - 200mm Long | 2 | $1.25 | Adafruit | Own | JST-SH female 4-pin connectors on both ends |
| ALITOVE 3 Wire Splice Connectors 10pcs | 1 | $7.99 | Amazon | Own | For quick prototyping wire harnesses (avoids soldering and crimping) |
| Weewooday 10 Pieces 12-Position Dual Row White Screw Terminal Strip | 1 | $6.99 | Amazon | Own | For quick prototyping wire harnesses (avoids soldering and crimping) |

## Not Yet Sourced

The following items from the project plan have not yet been purchased:

- **RPLidar A1** (360° LIDAR)
- **3S LiPo battery** and charger
