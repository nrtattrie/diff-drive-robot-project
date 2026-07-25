# Wiring Diagram Coverage

This ledger cross-checks the WireViz diagrams against the authoritative
[power and signal interface matrix](../power-signal-interface-matrix.md).
Coverage means the intended relationship is shown; it does not mean that the
connection has passed electrical or as-built validation.

## Power Interfaces

| ID | Connection | WireViz coverage |
|---|---|---|
| P0 | Official 27 W USB-C supply to Pi, bench alternative | `pi/pi-wiring.yml` |
| P1 | Future 3S battery, keyed connector, fuse, and distribution | `power/main-power-path.yml` |
| P2 | Distribution to `#2507` VIN/GND | `power/main-power-path.yml` |
| P3 | `#2507` M1 output to left motor conductors | `motor-encoder/motor-wire-groups.yml` |
| P4 | `#2507` M2 output to right motor conductors | `motor-encoder/motor-wire-groups.yml` |
| P5 | Distribution to DROK input | `power/main-power-path.yml` |
| P6 | DROK output through owned 23 mm stacking header to Pi pins 2/4/6/14 | `power/main-power-path.yml`, `pi/pi-wiring.yml` |
| P7 | Pi USB-A to Due Programming port | `power/main-power-path.yml`, `pi/pi-wiring.yml` |
| P8 | Due 5 V/GND to both encoders | `motor-encoder/motor-wire-groups.yml`, `control-logic/control-logic.yml`, `control-logic/encoder-translator.yml` |
| P9 | Due 3.3 V/GND to `#2507` VDD and translator | `control-logic/control-logic.yml`, `control-logic/due-motor-driver.yml`, `control-logic/encoder-translator.yml` |
| P10 | Pi through owned Adafruit `#2998` hub and `#954` to BNO055 power | `pi/pi-wiring.yml` |
| P11 | Pi CAM/DISP to Camera Module 3 | `pi/pi-wiring.yml` |
| P12 | Pi USB to future RPLidar development-kit adapter | `pi/pi-wiring.yml` |

## Signal Interfaces

| ID | Connection | WireViz coverage |
|---|---|---|
| S1 | Pi USB serial link to/from Due | `pi/pi-wiring.yml` |
| S2 | Due D6/D7 to M1PWM/M2PWM | `control-logic/control-logic.yml`, `control-logic/due-motor-driver.yml` |
| S3 | Due D8-D11 to driver direction inputs | `control-logic/control-logic.yml`, `control-logic/due-motor-driver.yml` |
| S4 | Driver M1EN/DIAG and M2EN/DIAG to Due D22/D23 | `control-logic/control-logic.yml`, `control-logic/due-motor-driver.yml` |
| S5 | Driver M1CS/M2CS to Due A0/A1 | `control-logic/control-logic.yml`, `control-logic/due-motor-driver.yml` |
| S6 | Both encoder A/B outputs to translator A1-A4 | `motor-encoder/motor-wire-groups.yml`, `control-logic/control-logic.yml`, `control-logic/encoder-translator.yml` |
| S7 | Translator B1-B4 to Due D2/D13/D5/D4 | `control-logic/control-logic.yml`, `control-logic/encoder-translator.yml` |
| S8 | BNO055 UART through active Adafruit `#954` and the `#2998` USB hub | `pi/pi-wiring.yml` |
| S9 | BNO055 3VO-to-PS1 UART-mode strap | `pi/pi-wiring.yml` |
| S10 | Camera Module 3 CSI/control interface | `pi/pi-wiring.yml` |
| S11 | Future RPLidar USB link | `pi/pi-wiring.yml` |

## Translator Fixed Wiring

| Requirement | WireViz coverage |
|---|---|
| Pin 1 DIR to Due 3.3 V | `control-logic/control-logic.yml`, `control-logic/encoder-translator.yml` |
| Pins 2-5 A1-A4 from left/right encoders | Same |
| Pins 6-9 A5-A8 to logic ground | Same |
| Pin 10 to common logic ground | Same |
| Pins 11-14 B8-B5 left unconnected | Same; the open pins are visible and labeled |
| Pins 15-18 B4-B1 to Due D4/D5/D13/D2 | Same |
| Pin 19 `/OE` to logic ground | Same |
| Pin 20 VCC to Due 3.3 V | Same |
| `104` / 0.1 uF capacitor across pins 20 and 10 | Same |

## System Rules Represented

- High-current battery negative, driver power ground, and DROK input return are
  shown in `power/main-power-path.yml`.
- Due logic ground is joined to the driver logic reference, translator,
  and both encoders in the control-logic diagrams. Notes explicitly prohibit
  motor return current from using these low-current paths.
- Pi USB-C bench power and DROK header power are shown as mutually exclusive in
  `pi/pi-wiring.yml`.
- The Due has one powered USB host, the Pi, in `pi/pi-wiring.yml`.
- The Due stays on a direct black Pi USB 2.0 port. Only the low-current
  BNO055/`#954` branch uses the bus-powered `#2998`; its single switch must
  remain on during operation and its external-power jack stays unused.
- Driver PWM startup LOW, fault-input direction, current-sense protection,
  translator direction/enable straps, and open translator pins are labeled in
  the control-logic diagrams.

## Installed Pi Assemblies Outside the P/S Matrix

The Pi diagram also shows the already-installed Active Cooler with its supplied
fan lead, the M.2 HAT+ with its supplied PCIe FFC, and the NVMe M.2 edge
connection. These are treated as complete managed assemblies rather than
field-wired individual conductors. A STEMMA QT lead must not replace the fan
lead merely because both connectors are small four-pin types.

## Owned Wiring and Connector Disposition

| Owned BOM item | Diagram assignment |
|---|---|
| 16 AWG silicone wire | P1 trunk/fused segment, P2 driver branch, and P5 DROK input branch in `power/main-power-path.yml` |
| Waterproof inline holder plus one 10 A fuse from the standard kit | F1 in `power/main-power-path.yml` |
| ALITOVE 3-wire splice pieces | Separate fused-positive and battery-negative 3-way junctions in `power/main-power-path.yml`, but only after continuity proves the selected ports are common |
| 23 mm 2x20 extra-tall stacking header | One H1 GPIO extension through the M.2 HAT+ in both power and Pi views |
| Assorted Elegoo/Dupont leads | Four short P6 power leads; twelve male-to-female Due/`#2507` logic leads; low-current carrier wiring; one BNO055 mode strap |
| Two `#4753` factory 1x6/2.54 mm harnesses | Motor, encoder-power, and encoder-signal groups in `motor-encoder/` and `control-logic/` |
| `#2507` supplied screw terminals and breakaway male header | Power/motor terminations, the 13-pin logic header, and optionally two marked 1x6 carrier headers for the retained encoder housings |
| Mega Prototype Shield V3, `#2204`, `#735`, and `104` capacitor | Socketed encoder-translator carrier in both translator diagrams |
| Adafruit `#954` and user-reported owned `#2998` | BNO055 UART cable and bus-powered USB expansion in `pi/pi-wiring.yml` |
| Owned camera FFC, cooler lead, and M.2/PCIe parts | Used intact as the complete managed assemblies shown in `pi/pi-wiring.yml` |
| 5-pin Pololu `#5530`/`#5534` JST-SH cables | Spare: no installed endpoint has a matching 5-pin/1 mm socket |
| 4-pin STEMMA QT cables | Spare for future I2C experiments; not used with the selected BNO055 UART connection |
| Weewooday terminal strips | Optional secured bench fixtures only; not part of the mobile battery or motor path |

## Explicitly Provisional or Non-Installed

- The battery, keyed connector family, and battery-specific undervoltage
  solution remain unselected; the provisional path is shown and labeled.
- Motor connector cavity order and final motor terminal polarity require
  physical inspection and the unloaded direction test.
- Exact top/bottom member of each same-color Pi USB pair, CAM/DISP port,
  `#2998` socket label/captive-lead length, cable lengths, and several
  low-current termination/fanout details remain to be marked after placement.
- The future RPLidar connection is shown but is not an installed component.
- INA219, APDS9960, HC-SR04, `#2991`, white terminal strips, and the 1 A toggle
  remain outside the installed baseline.
- The temporary D6/D7-to-D3 PWM commissioning loop is not an installed
  connection. It is deliberately absent from the as-built diagrams and must be
  removed after its test.
