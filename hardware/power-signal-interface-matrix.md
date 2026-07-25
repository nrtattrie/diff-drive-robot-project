# Phase 1 Power and Signal Interface Matrix

**Status:** Q10 complete as a Stage 2A design definition. No connection in this
document is considered electrically validated until it passes the applicable
Stage 2B gate in the [assembly guide](../docs/assembly.md) and
[validation log](stage-2-validation-log.md).

**Last reviewed:** 2026-07-25

This is the authoritative connection-level companion to the
[Stage 2 hardware decision record](stage-2-hardware-decision-record.md). It
turns Q3-Q9 into one buildable pin, power, wire, protection, and safe-state
definition without changing the decisions already made there.

## Scope and Rating Convention

The active Phase 1 baseline includes the battery interface, protected power
distribution, DROK, Raspberry Pi 5, Arduino Due, `#2507`, both `#4753` motors
and encoders, the `74LVC245`, owned Adafruit `#2998` USB hub, BNO055, Camera
Module 3, installed M.2/NVMe and Active Cooler assemblies, and the planned
RPLidar A1M8 development kit.

- A **normal ceiling** is a design or manufacturer limit, not a prediction
  that the device always draws that current.
- A **transient ceiling** is shown where the manufacturer supplies one or a
  conservative system case can be calculated. It is not permission to operate
  continuously at that value.
- Logic signal wires do not carry load power. Their current is input/pull-up
  current only and is described as `logic-only`.
- Values marked **validate** are commissioning evidence, not undecided
  architecture. Stage 2B must measure them.
- `M1` is assigned to the left motor and `M2` to the right motor. The unloaded
  direction test establishes which terminal polarity produces positive wheel
  motion.

The owned INA219, APDS9960, HC-SR04, `#2991`, white terminal strips, and 1 A
toggle are not installed in the baseline matrix. Adding one later
requires a matrix revision and a defined power/interface budget. The battery
connector and undervoltage method remain battery-selection-dependent Stage 4
items.

## System Topology

```text
3S battery
  |
  +-- keyed connector -- 10 A ATO/ATC fuse -- ALITOVE distribution
                                                   |
                                                   +-- #2507 -- M1/M2
                                                   |
                                                   +-- DROK 5.10 V -- Pi 5
                                                                          |
                                                                          +-- USB -- Due
                                                                          |          |
                                                                          |          +-- encoders, 5 V
                                                                          |          +-- LVC245/#2507 VDD, 3.3 V
                                                                          |
                                                                          +-- USB -- #2998 hub -- USB-TTL -- BNO055
                                                                          +-- CAM/DISP -- Camera 3
                                                                          +-- USB -- future RPLidar A1
```

Bench mode replaces the battery/DROK feed to the Pi with the official 27 W
USB-C supply. Never connect the Pi's USB-C supply and 5 V header feed at the
same time.

## Power Interface Matrix

| ID | Source to load; direction and channels | Voltage | Maximum normal / transient current | Connector and wire | Protection and shutdown state | Controller |
|---|---|---:|---:|---|---|---|
| P0 | Official 27 W supply to Pi; power, 1 branch, bench only | 5.1 V | 5 A supply ceiling / supply-managed | Official USB-C cable | Official supply protection; unplug to isolate; mutually exclusive with P6 | Pi power management |
| P1 | Future 3S battery to fused distribution; power, 1 trunk | 11.1 V nominal, 12.6 V full; low endpoint deferred with battery | About 6.8 A normal design budget; theoretical short transient can approach 15 A before protection | Future keyed/polarized battery mate, 16 AWG copper, selected 14 AWG inline holder, ALITOVE junction | 10 A ATO/ATC fuse nearest practical point to battery positive; keyed connector is manual disconnect; undervoltage policy required before Stage 4 | User disconnect; fuse |
| P2 | Fused distribution to `#2507 VIN/GND`; power, 1 branch | 3S battery range | 2.76 A combined normal target / 11 A combined theoretical motor stall | 16 AWG copper into `#2507` 5 mm terminal; common battery-negative junction | Main 10 A fuse plus driver reverse-voltage, thermal, over/undervoltage, and short protection; Due commands coast | Due and `#2507` |
| P3 | `#2507 M1A/M1B` to left `#4753`; bidirectional motor power, 1 channel | PWM from 3S; initial average ceiling about 12.0 V at 95% duty on 12.6 V | 1.38 A normal target / 5.5 A theoretical stall | Motor factory red/black leads, removed from the shared housing or cut/stripped, directly into M1 terminal | Driver protection; command watchdog, fault, or detected stall causes both channels to coast | Due M1 control |
| P4 | `#2507 M2A/M2B` to right `#4753`; bidirectional motor power, 1 channel | Same as P3 | 1.38 A normal target / 5.5 A theoretical stall | Motor factory red/black leads directly into M2 terminal | Same as P3 | Due M2 control |
| P5 | Fused distribution to DROK input; power, 1 branch | 3S battery range, subject to exact-unit input check | Allocate no more than 4 A normal to support a 25.5 W output with conversion loss; startup transient must remain inside the validated converter/main-fuse path | 16 AWG copper into the exact unit's input terminals | Main 10 A fuse; secure adjustment after setting; keyed disconnect removes input | DROK regulation; user disconnect |
| P6 | DROK output to Pi 5 header; power, 1 branch over 4 conductors | 5.10 V set point; validate at Pi header under load | 5 A path ceiling / startup included in Stage 2B load test; approximately 2.5 A per positive and return lead if sharing is even | Four equal, short Elegoo leads through one owned 23 mm 2x20 GPIO extender: positives to physical pins 2/4 and returns to 6/14; each at most about 12 in; cut/strip the DROK ends and verify the actual header/jumper mating sex | No USB-C power simultaneously; reject on polarity error, excessive drop, heating, undervoltage, or reset; main fuse upstream | DROK and Pi power management |
| P7 | Pi USB-A to Due Programming port; power and data, 1 branch | USB 5 V | Conservative 500 mA branch ceiling / USB-managed | One data-capable USB-A-to-micro-B cable | Due USB polyfuse; Pi USB current budget; power loss/reset leaves motor PWM pulled low and motors coasting | Pi USB host; Due |
| P8 | Due 5 V output to both motor encoders; power, 2 loads | Regulated 5 V | 10 mA per encoder, 20 mA combined / same manufacturer maximum | Factory blue/green conductors retained in each 1x6/2.54 mm housing where practical, mating marked male header segments on the Mega-prototype carrier; short soldered carrier wiring and owned jumpers complete the fanout | Inside P7 ceiling; never connect encoder supply to motor battery | Due USB branch |
| P9 | Due 3.3 V output to `74LVC245 VCC` and `#2507 VDD`; power, 2 loads | 3.3 V | Allocate 20 mA combined / 20 mA; far below the Due rail's 800 mA rating | Owned male-to-female Dupont lead to the `#2507`; short jumper/soldered wiring on the owned Mega Prototype Shield V3 carrier; `0.1 uF` ceramic directly across LVC245 pins 20 and 10 | Inside P7 ceiling; P9 loss removes EN/DIAG pull-ups while P7 loss also releases PWM to its board pull-down | Due regulator |
| P10 | Pi USB through owned bus-powered Adafruit `#2998` hub and owned `#954` cable to BNO055 `VIN/GND`; power, 1 load | 5 V input; BNO board creates 3.3 V internally | 50 mA BNO allocation plus uncharacterized hub overhead; cable power output is rated 500 mA | `#2998` captive USB-A lead to one black Pi USB 2.0 port; hub port 1 to `#954`; `#954` red to `VIN`, black to `GND`, using soldered BNO 0.1 in male header | Included in Pi USB budget; keep hub switch on in operation; leave its external DC jack unused; Pi shutdown or hub switch/cable removal powers sensor off | Pi USB host and hub switch |
| P11 | Pi CAM/DISP connector to Camera Module 3; power and data, 1 interface | Pi-managed camera rails; 3.3 V control | Allocate 0.5 A equivalent within the Pi's 5 V whole-system budget for normal/transient planning; the camera connector itself is Pi-managed, not a user-wired power branch | Use the owned **Standard-to-Mini** cable supplied in the camera bag: 22-pin, 0.5 mm Pi end to 15-pin, 1 mm camera end; verify the unequal ends before insertion | Pi camera power-enable and driver shutdown; power removed with Pi | Pi camera subsystem |
| P12 | Pi USB to future RPLidar A1M8 development-kit USB adapter; power and data, 1 branch | 5 V | Up to about 450 mA working (350 mA scanner plus 100 mA motor) / up to about 700 mA startup (600 mA scanner plus 100 mA motor) | Buy the development kit with its USB adapter/cable; connect directly to one blue Pi USB 3.0 port rather than the hub | Included in the validated Pi USB total; stop scanning in software before shutdown; Pi power loss stops unit | Pi USB host and ROS driver |

P1's theoretical transient combines both 5.5 A motor stall values and the 4 A
DROK input allocation. It is a fault-envelope calculation, not a normal load.
The 10 A fuse protects the main harness according to the selected fuse's
time-current behavior; it is not a precise electronic current limiter.

The conservative named-device Pi USB startup allocation is `0.50 A` for the
Due branch, `0.05 A` for the BNO055, and `0.70 A` for the future RPLidar, or
`1.25 A`, plus the `#2998`'s uncharacterized self-consumption. The Pi 5 fan
header shares the downstream peripheral-current budget, so the Active Cooler
must also be present during the combined test. This arrangement is permitted
only with the Pi's `1.6 A` budget from a recognized or deliberately configured
5 A source. The RPLidar may not be added to the mobile configuration until
Q7's full 5 A validation and `PSU_MAX_CURRENT=5000` enablement have passed.
The hub adds communication ports, not current capacity; leave its 1.35 mm
external-power jack unused.

## Signal Interface Matrix

The Due is a 3.3 V-only controller. Its digital input limits at a 3.3 V supply
are approximately `LOW <= 0.99 V` and `HIGH >= 2.31 V`. No 5 V encoder signal
may bypass the translator.

| ID | Source to destination; direction and channels | Levels, thresholds, and rate | Signal current | Connector and wire | Protection and shutdown state | Controller |
|---|---|---|---|---|---|---|
| S1 | Pi USB host to/from Due Programming port; bidirectional, 1 serial link | USB-managed electrical levels; 115200 baud, 8N1, newline-delimited ASCII; signed left/right targets in encoder counts/s; state telemetry at 20 Hz | USB signal-only; P7 carries power | P7 data-capable USB cable | Strict length/shape/range parsing; boot disarmed; only valid `SET` refreshes the recoverable 500 ms watchdog; stale/invalid traffic leads to zero target and coast; driver/stall faults latch until a safe explicit reset | Pi ROS bridge and Due firmware |
| S2 | Due D6 to `M1PWM`, and D7 to `M2PWM`; Due to driver, 2 channels | 0/3.3 V; driver `LOW <= 0.9 V`, `HIGH >= 2.1 V`; 20 kHz hardware PWM | Logic-only | `#2507` board-supplied 2.54 mm male logic header and owned Elegoo male-to-female leads: male into Due, female onto driver | `#2507` pulls PWM low by default, making outputs high impedance; firmware configures both low before other motor pins | Due hardware PWM |
| S3 | Due D8/D9 to `M1INA/M1INB`, and D10/D11 to `M2INA/M2INB`; Due to driver, 4 channels | 0/3.3 V; same driver thresholds as S2 | Logic-only | Same header/lead class as S2 | Direction inputs float during reset, but PWM's hardware-low default keeps outputs coasting; firmware sets direction only after PWM is low | Due GPIO |
| S4 | `M1EN/DIAG` to Due D22 and `M2EN/DIAG` to D23; driver to Due, 2 channels | Normal HIGH pulled to 3.3 V by P9; driver fault LOW; Due thresholds above | Logic-only; pull-up current only | Same header/lead class as S2 | Configure Due pins as **inputs only**. Either LOW disables that hardware channel; firmware responds by coasting both and latching motion off | `#2507` fault hardware and Due |
| S5 | `M1CS` to Due A0 and `M2CS` to A1; driver to Due, 2 analog channels | About 0.140 V/A while driving: about 0.193 V at 1.38 A and 0.77 V at the motor's 5.5 A theoretical stall; 3.3 V would correspond to about 23.6 A | Signal-only in the intended range; the board's existing 10 kΩ series resistor limits abnormal input current to at most a few hundred microamps | Owned male-to-female Dupont leads from the `#2507` male header to A0/A1; no added clamp components | Pololu considers the 10 kΩ-isolated outputs generally safe for 3.3 V MCU inputs. No external clamp is required for these 5.5 A-stall motors; reconsider only if the motor/current envelope changes. Firmware treats current as telemetry; fault/stall policy commands coast | Due ADC and firmware |
| S6 | Left encoder yellow/white to LVC245 A1/A2 and right yellow/white to A3/A4; encoder to translator, 4 channels | 0/5 V; LVC245 at 3.3 V accepts `LOW <= 0.8 V`, `HIGH >= 2.0 V`; up to about 2.67 kHz per channel at 200 rpm | Logic-only | Factory conductors retained in each 1x6 housing where practical, mated to marked male header segments soldered on the translator carrier after cavity-order verification | Translator prevents 5 V reaching Due; loss of valid encoder progress under a motion command invokes the calibrated stall response | Motor encoders |
| S7 | LVC245 B1/B2 to Due D2/D13 and B3/B4 to D5/D4; translator to Due, 4 channels | 0/3.3 V; output meets Due thresholds; up to about 10.67 k count events/s per motor in full quadrature at 200 rpm | Logic-only | `#735` in one `#2204` socket on the owned Mega Prototype Shield V3; short carrier wiring and owned male-ended Dupont leads to Due | `/OE` is permanently low; DIR permanently high; P9 loss removes outputs; firmware count-integrity checks support stall detection | Due timer-counter quadrature blocks |
| S8 | BNO055 SDA (UART TX) to `#954` white (RX into USB), and `#954` green (TX out) to BNO055 SCL (UART RX); bidirectional UART as 2 unidirectional channels, with the `#954` USB end on `#2998` port 1 | BNO breakout and cable use compatible 3.3 V logic; USB 2.0 upstream; 115200 baud, 8N1 on the UART side | Logic-only | `#2998` to `#954` USB-A; soldered BNO 0.1 in male header to the cable's individual female sockets | Keep the hub switch on; Pi/driver timeout marks lost IMU data invalid; power loss stops sensor; no motor-safety behavior depends solely on IMU | Pi ROS BNO055 driver |
| S9 | BNO055 `3VO` to `PS1`; fixed sensor-mode strap, 1 channel | 3.3 V HIGH selects UART while `PS0` remains at its default LOW state | Logic-only | One owned female-to-female Elegoo/Dupont jumper at the BNO header | Fixed configuration; do not connect the STEMMA I2C bus simultaneously | Hardware strap |
| S10 | Camera Module 3 to/from Pi CAM/DISP; camera to Pi data plus bidirectional control, 1 CSI-2 interface | Two MIPI CSI-2 data lanes and clock; Pi-managed 3.3 V control | Signal-only; P11 carries module power | P11 22-to-15-pin FFC | Pi camera driver controls enable and streaming; stop capture before shutdown | Pi camera subsystem |
| S11 | Future RPLidar development-kit USB adapter to/from Pi; bidirectional, 1 serial/USB link | USB-managed externally; scanner-side UART is handled by supplied adapter | USB signal-only; P12 carries power | Supplied development-kit USB cable | ROS driver commands scan/motor stop; stale scans are rejected by navigation software | Pi and RPLidar ROS driver |

## Arduino Due Pin Allocation

| Due resource | Assignment | Required startup configuration |
|---|---|---|
| D2 / `TIOA0` | Left encoder A from LVC245 B1 | Hardware quadrature input |
| D13 / `TIOB0` | Left encoder B from LVC245 B2 | Hardware quadrature input |
| D5 / `TIOA6` | Right encoder A from LVC245 B3 | Hardware quadrature input |
| D4 / `TIOB6` | Right encoder B from LVC245 B4 | Hardware quadrature input |
| D6 / `PWML7` | M1PWM | Set output LOW before configuring motor direction |
| D7 / `PWML6` | M2PWM | Set output LOW before configuring motor direction |
| D8, D9 | M1INA, M1INB | Set only while M1PWM is LOW |
| D10, D11 | M2INA, M2INB | Set only while M2PWM is LOW |
| D22, D23 | M1EN/DIAG, M2EN/DIAG | Inputs only; HIGH normal, LOW fault/disabled |
| A0, A1 | M1CS, M2CS through the `#2507` onboard 10 kΩ resistors | Analog inputs |
| Programming USB | Pi command/telemetry and Due power | No second powered USB host |
| 5 V, 3.3 V, GND | P8/P9 logic supplies and common reference | Outputs only in this design |

No default shield pin mapping or stock shield library pin assignment is
authoritative. Firmware must use this table.

Stage 2B Gate B5 temporarily connects D6 and then D7 to spare timer-capture
input D3 / `TIOA7` for same-controller PWM commissioning. D3 is disconnected
after that gate and has no installed baseline function.

## `74LVC245` Fixed Wiring

| IC pin | Connection |
|---|---|
| 1 `DIR` | Due 3.3 V, selecting A-to-B |
| 2 `A1` / 18 `B1` | Left encoder A / Due D2 |
| 3 `A2` / 17 `B2` | Left encoder B / Due D13 |
| 4 `A3` / 16 `B3` | Right encoder A / Due D5 |
| 5 `A4` / 15 `B4` | Right encoder B / Due D4 |
| 6-9 `A5-A8` | Logic ground |
| 10 `GND` | Common logic ground |
| 11-14 `B8-B5` | Leave unconnected |
| 19 `/OE` | Logic ground, permanently enabled |
| 20 `VCC` | Due 3.3 V |
| Across 20 and 10 | Owned `104`/0.1 uF ceramic capacitor, mounted at the IC |

## Ground and Safe-State Rules

1. Battery negative splits through the high-current distribution to the
   `#2507` power ground and DROK input ground. Motor return current must not
   use Due, Pi, encoder, or jumper-wire ground paths.
2. The DROK output ground establishes Pi ground; the Pi-to-Due USB cable then
   establishes Due logic ground. Join Due logic ground to `#2507` logic ground,
   translator ground, and both encoder grounds.
3. Before battery power, continuity-check that the selected ALITOVE ports form
   the intended common junction and verify whether DROK input/output grounds
   are common. Check polarity and tug-test every termination.
4. On Due reset, the board's PWM pull-downs hold both driver outputs
   high-impedance. Firmware must explicitly establish `PWM LOW`, then direction
   and input/fault configuration, before accepting a motion command.
5. On a stale command, both wheel targets ramp to zero and then coast. On
   either driver fault or a calibrated encoder stall, both channels coast and
   motion remains latched off until explicit reset.
6. The BNO055 is intentionally assigned to UART through the already-owned
   `#954` cable on one switched port of the owned bus-powered `#2998`. This
   avoids adding the sensor's I2C behavior to the Pi bus and matches the
   selected ROS 2 driver's 115200-baud path. Do not use the hub's external
   power jack and do not connect STEMMA I2C simultaneously.
7. Connect the Due to only one powered USB host. Connect the Pi by only one
   power method. These are wiring rules, not software preferences.

## Q10 Procurement Result

Q10 adds no unconditional immediate electronic or cable purchase:

- the `#2507` current-sense outputs connect directly to Due A0/A1 through the
  board's existing 10 kΩ series resistors; no external clamp diodes are
  required for the selected motors; and
- the camera bag already contains two cables. Use the supplied
  Standard-to-Mini cable with the narrow 22-pin Pi 5 end and wider 15-pin
  Camera Module 3 end.

Physically confirm that a data-capable USB-A-to-micro-B cable is already on
hand for the Due. Buy one only if it is missing. The `#2998` is owner-reported
as already owned even though BOM REV 02 omits it.

The battery-mating connector, battery-specific undervoltage solution, and
RPLidar A1 development kit remain deferred. BOM REV 02 records the selected
16 AWG red/black wire as owned.

## Evidence Sources

- [Pololu `#2507` product and electrical limits](https://www.pololu.com/product/2507)
- [Pololu Dual VNH5019 shield user's guide](https://www.pololu.com/docs/0J49/all)
- [ST VNH5019A-E datasheet](https://www.pololu.com/file/0J504/vnh5019a-e.pdf)
- [Pololu `#4753` motor and encoder specifications](https://www.pololu.com/product/4753)
- [Official Arduino Due specifications](https://store.arduino.cc/products/arduino-due)
- [Arduino Due core pin definitions](https://github.com/arduino/ArduinoCore-sam/blob/master/variants/arduino_due_x/variant.cpp)
- [Microchip SAM3X/SAM3A datasheet](https://ww1.microchip.com/downloads/en/devicedoc/atmel-11057-32-bit-cortex-m3-microcontroller-sam3x-sam3a_datasheet.pdf)
- [TI SN74LVC245A datasheet](https://www.ti.com/lit/ds/symlink/sn74lvc245a.pdf)
- [Raspberry Pi 5 power documentation](https://www.raspberrypi.com/documentation/hardware/raspberrypi/power/raspberry-pi-5.html)
- [Raspberry Pi camera connector documentation](https://www.raspberrypi.com/documentation/accessories/camera.html)
- [Adafruit BNO055 pinouts](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/pinouts)
- [Adafruit `#954` USB-to-TTL cable](https://www.adafruit.com/product/954)
- [Adafruit `#2998` USB Mini Hub with Power Switch](https://www.adafruit.com/product/2998)
- [Raspberry Pi USB current and single-TT hub guidance](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#universal-serial-bus-usb)
- [ROS 2 BNO055 UART wiring and parameters](https://github.com/flynneva/bno055)
- [SLAMTEC RPLidar A1 specifications](https://www.slamtec.com/en/lidar/a1spec)
- [SLAMTEC RPLidar A1M8 datasheet](https://wiki.slamtec.com/download/attachments/83066883/LD108_SLAMTEC_rplidar_datasheet_A1M8_v3.0_en.pdf?api=v2)
