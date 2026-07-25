# Component Selection Audit

**Status:** Motor/driver/IMU/wheel selection history complete; broader hardware revalidation remains in progress

**Started:** 2026-07-16

This document reconstructs how the Phase 1 hardware decisions were made, checks
the claims against primary sources, and separates historical rationale from
verified design facts. It is intentionally not a replacement wiring diagram or
final component recommendation. Current candidate feedback and provisional
recommendations are maintained in the
[Stage 2 hardware decision record](stage-2-hardware-decision-record.md).

## Context and Responsibility

Nathan supplied the source chat after the project documents were found to
contain an incompatible one-channel motor driver and several unverified power
assumptions. At the time of the chat, Nathan was asking introductory questions
and relying on Claude Code to identify products, perform calculations, and
explain the system. A response such as "Sounds good" records acceptance of the
assistant's recommendation; it does not mean Nathan independently verified the
part number, specifications, or equations.

Claims from the old chat and generated summaries are therefore historical
inputs, not validated engineering evidence.

## Evidence Batch 1: Motor and Driver Selection

The supplied chat appears to be from the January 26, 2026 component-selection
work that produced `robotics_project_summary_v5.md`.

### Decision chain in the chat

1. Claude correctly identified the Adafruit DRV8871 as a single H-bridge and
   correctly said that differential drive requires two independent motor
   channels.
2. Claude proposed a "Pololu Dual G2 High-Power Motor Driver 18v25" and described
   it as one board with two 4.5 A channels and 25 A peak capability.
3. The subsequent torque calculation silently changed that name to "Pololu G2
   High-Power 18v17" while retaining the claim that it had two 4.5 A channels.
4. Nathan accepted the recommendation based on that explanation.
5. Claude stated that the selection was finalized and ready to order.
6. The generated v5 summary then listed one real Pololu `#2991` while continuing
   to call it a dual driver with per-channel specifications.

This is the origin of the motor-driver failure: properties from several product
families were combined into a product description that does not exist, and a
real single-channel part number was later attached to it.

### What this excerpt alone does not explain

The chat's final recap names:

- motor `#4751`;
- a BNO085 IMU;
- 65 mm Phase 1 wheels and 80 mm Phase 2 wheels.

The generated v5 summary instead names:

- motor `#4753`;
- a BNO055 IMU;
- 80 mm wheels for both phases.

Those later transitions are not present in the assistant's prose recap, but the
same supplied conversation says Claude edited v5. Comparing that conversation
with the generated v5 file is sufficient to establish the mismatch; another
transcript is not required to understand the failure.

## Evidence Batch 2: Archived Project Summaries

All seven summaries in `docs/_archive/` were reviewed in sequence. They provide
some additional rationale, but they do not contain the source conversation or
manufacturer evidence needed to validate the choices.

### Version timeline

| Version | Hardware-selection state | New evidence relevant to this audit |
|---|---|---|
| v1 | General project concept | No motor, driver, IMU, or wheel selection. |
| v2 | Requirements and candidate families | Introduces two Pololu 37D motors, a Pololu MC33926-or-G2 driver choice, 3 kg maximum mass, 65 mm wheels, and 0.3-0.7 m/s target speed. All remain TBD. |
| v3 | Same as v2 | No relevant component-selection change. |
| v4 | Selection explicitly deferred | The January 24 log says motor selection will wait until after Ubuntu setup and ROS 2 fundamentals. Driver remains MC33926 family vs. G2 series. |
| v5 | All components declared finalized | In one revision, the summary adds `#4753`, one `#2991`, BNO055 `#4646`, 80 mm wheels, the torque/current claims, and the completed shopping carts. |
| v6 | Hardware copied unchanged | Adds a directive calling the summary the single source of truth. It adds no new hardware verification. |
| v7 | Hardware copied unchanged | Adds the four-stage implementation roadmap, including a direct jump from CAD/firmware/wiring to real driving. It adds no hardware-validation stage. |

The important discontinuity is therefore v4 to v5. No archived intermediate
summary records the individual checks that would have been needed to move from
TBD components to a validated system.

### Motor SKU transition

None of the archived summaries mentions `#4751`. The first archived finalized
selection in v5 identifies the intended 50:1 motor as `#4753`, which is the
correct Pololu item number for that ratio.

The v5 summary nevertheless carries forward the approximately 0.75 N m stall
torque from the earlier chat. That value is near the 19:1 motor's rating, not the
2.06 N m extrapolated stall torque of `#4753`. The best-supported interpretation
is that Claude's file edit corrected the item number to match the intended 50:1
ratio while its prose recap still said `#4751` and the associated specifications
and calculations were not refreshed. The user-provided conversation and v5 file
together establish this; the exact internal mechanism of the inconsistent edit
does not need a separate source chat.

### IMU transition

The v5 progress log says BNO055, BNO085, and MPU6050 were evaluated and records
the following rationale for choosing the BNO055:

- claimed better ROS 2 support than BNO085;
- mature drivers and documentation;
- built-in sensor fusion;
- STEMMA QT connection;
- usefulness for odometry fusion and eventual balancing.

This explains the stated reason for the BNO085-to-BNO055 change. The original
summary did not include links, driver/package names, compatibility testing, or
an interface review. The ROS 2 ecosystem portion has now been checked
independently below; physical compatibility and operation still require bench
validation.

### Independent ROS 2 ecosystem check

The narrow claim that BNO055 had the more mature and convenient ROS 2 software
path is supported.

At the January 2026 decision date:

- `flynneva/bno055` had ROS 2 release history going back to 2021 and released
  Humble packages from 2023 and 2024;
- the package supported both I2C and UART, published standard ROS messages, and
  documented parameters, calibration, topics, launch, and source builds;
- `bnbhat/bno08x_ros2_driver` was much newer, with its initial 0.1.0 release
  dated July 13, 2025;
- the BNO08x driver was source-build oriented, indexed for Humble but not
  released into the ROS distribution, with only I2C implemented.

For the current ROS 2 Jazzy project:

- `bno055` 0.5.0 is released and built as a Jazzy/Noble binary package;
- its Jazzy API and package documentation are generated on `docs.ros.org`;
- ROS Index still marks `bno08x_driver` as unreleased and shows Humble rather
  than Jazzy as its indexed distribution;
- the BNO08x source driver now documents up to 400 Hz IMU output, but UART and
  SPI remain unimplemented.

**Verdict:** "BNO055 has better ROS 2 package availability and a more mature
integration path for this project" was and remains a fair claim. It does not
prove that BNO055 is the technically better IMU. BNO085 is newer and its source
driver exposes higher rates, but choosing it would currently require accepting
more integration and maintenance work. The owned BNO055 is therefore a
reasonable Phase 1 choice, subject to bench testing on the Pi and validation of
message conventions, calibration behavior, and update rate.

### Wheel transition

The v5 progress log contains both states of the decision:

1. The morning entry assigns 65 mm wheels to Phase 1 and 80 mm wheels to Phase 2.
2. The afternoon entry changes to 80 mm wheels for both phases to save about
   $8-$10 and eliminate the mechanical wheel swap.

The afternoon decision cites the old 5.9 Phase 1 and 1.26 Phase 2 torque margins
as proof that the simplification is safe. The archive therefore explains why
80 mm wheels were ordered, but the Phase 2 support for that choice remains
invalid because it comes from the dimensionally incorrect balancing calculation.

### Motor-driver transition

The archive provides no additional valid explanation for the driver choice.
Instead, v5 contains the contradiction in two forms at the same time:

- the shopping list orders one `#2991`, correctly using the real product's name
  without the word "Dual";
- the key-decisions and progress-log sections call it a "Pololu Dual G2
  High-Power 18v17" with 4.5 A continuous per channel.

This confirms that the order was generated from a real catalog item while the
system reasoning still used the invented dual-channel description. Versions 6
and 7 copy the contradiction without revisiting it.

### How the error became authoritative

Version 5 labels the design performance "validated" and the shopping carts
"finalized," but records no datasheet review, schematic, interface table, test,
or acceptance criterion. Version 6 then adds a directive requiring future
assistants to treat the summary as the single source of truth. Version 7 builds
the implementation roadmap on top of it and schedules chassis CAD, firmware,
and wiring without an intervening component or electrical validation gate.

The archive therefore explains how an unsupported chat recommendation became
project scope: repetition and document status replaced verification.

### Remaining provenance gaps

The archived summaries still do not establish:

- where the nonexistent 4.5 A-per-channel G2 specification came from;
- any channel-count check before ordering `#2991`;
- any review of encoder voltage, Due `VIN`, Raspberry Pi GPIO power, switching,
  fusing, wiring, or protection.

The motor-SKU mismatch is sufficiently explained by the supplied conversation
and generated file, and the BNO055 ROS 2 claim has now been independently
checked. The remaining questions require other contemporaneous evidence or new
engineering validation.

## Claim Audit

| Historical claim | Assessment | Corrected interpretation |
|---|---|---|
| DRV8871 is a single H-bridge for one motor | Correct | Two boards would be needed for two independently driven motors. |
| DRV8871 supplies 3.6 A continuously and 6.5 A peak | Incorrect | Adafruit and TI specify 3.6 A peak. The Adafruit breakout ships with an approximately 2 A current-limit setting. Continuous capability depends on thermal conditions. |
| Lack of "built-in encoder support" is a DRV8871 disadvantage | Misleading | A motor driver normally drives motor current; the encoder connects separately to the MCU. The proposed Pololu G2 drivers also do not decode the motor encoders. |
| A dual board inherently uses fewer motor-control GPIO pins | Not established | Independent motors still require independent commands. Exact pin count depends on the driver's control interface and which enable, fault, and current-sense signals are used. |
| Pololu Dual G2 18v25, two 4.5 A channels, 25 A peak | Nonexistent combination | Pololu `#2994` is a single-channel G2 18v25. Actual dual G2 products have different names, ratings, form factors, and part numbers. |
| Pololu Dual MC33926 provides about 3 A continuous and 5 A peak per channel | Substantially correct | Pololu specifies almost 3 A continuous and 5 A peak per channel, subject to strong thermal limitations. Price claims in the chat are historical and were not used for validation. |
| `#4751` is the 50:1 encoder motor | Incorrect | `#4751` is the 19:1 encoder version. The purchased 50:1 encoder motor is `#4753`. |
| The 50:1 motor has about 0.75 N m stall torque | Incorrect for `#4753` | Pololu lists 21 kg cm, approximately 2.06 N m, extrapolated stall torque at 12 V for `#4753`. The old value is close to the 19:1 motor's rating and followed the SKU mix-up. |
| Seventy percent of stall torque is a suitable usable-continuous value | Unsafe assumption | Pololu recommends continuous loads around 25% or less of stall torque/current and warns that stalls can quickly damage the motor or gearbox. Stall torque is not an operating rating. |
| One `#2991` can drive both differential-drive motors independently | Incorrect | `#2991` contains one H-bridge for one bidirectional brushed DC motor. The robot needs two independent H-bridges. |
| `#2991` is rated 4.5 A continuous per channel | Incorrect | Pololu specifies one channel and 17 A continuous under its stated test conditions. That rating does not create a second channel. |
| Seven amps is the peak current for both `#4753` motors | Incorrect terminology | Each motor has a 5.5 A theoretical stall current at 12 V, or 11 A combined. Seven amps represented the chat's assumed 70%-torque operating point, not the system peak. |

## Recalculation of the Navigation Example

The original navigation example used:

- mass: 1.5 kg;
- rolling-resistance coefficient: 0.015;
- acceleration: 0.7 m/s^2;
- incline: 10 degrees;
- two equally loaded driven wheels.

Correcting the missed division by two in the rolling-resistance term gives the
following simultaneous per-motor torque estimates:

| Wheel diameter | Rolling | Acceleration | 10-degree incline | Total per motor |
|---|---:|---:|---:|---:|
| 65 mm | 0.0036 N m | 0.0171 N m | 0.0415 N m | 0.0622 N m |
| 80 mm | 0.0044 N m | 0.0210 N m | 0.0511 N m | 0.0765 N m |

For `#4753`, the extrapolated 12 V stall torque is about 2.06 N m. A simple
linear voltage scaling gives about 1.90 N m at 11.1 V. Twenty-five percent is
about 0.48 N m. This is not a formal continuous rating, but it is consistent with
Pololu's general recommendation to keep continuous load near or below 25% of
stall torque.

Under these simplified assumptions, the navigation torque margin is still
large: approximately 7.7 at 65 mm and 6.2 at 80 mm before accounting for losses,
load imbalance, transients, traction, manufacturing variation, or thermal
conditions. The Phase 1 motor conclusion therefore remains plausible even
though the original calculation reached it using the wrong SKU, wrong stall
torque, and unsafe 70% operating assumption.

At 11.1 V, linear no-load-speed scaling gives approximately 185 RPM. That is
about 0.63 m/s with 65 mm wheels and 0.77 m/s with 80 mm wheels. The chat's
150 RPM "realistic" loaded speed was an unsupported assumed operating point,
not a validated speed prediction.

## Why the Balancing Calculation Is Invalid

The chat calculated a supposed per-wheel balancing torque using:

`m * g * h * sin(theta) / wheel_radius`

The numerator is torque in N m. Dividing it by wheel radius produces force in N,
not another torque. With the stated 1.5 kg mass, 0.06 m center-of-mass height,
and 10-degree angle, the numerator is about 0.153 N m. Dividing by a 0.0325 m
radius gives 4.72 N, not 0.47 N m. The chat both mislabeled the units and appears
to have introduced a factor-of-ten arithmetic error.

More importantly, a self-balancing robot cannot ultimately be validated with a
static torque comparison. It is a coupled wheeled inverted pendulum. When the
project reaches full Phase 2 design and validation, that work can include:

- body center of mass above the wheel axle;
- body, wheel, and motor inertia;
- motor torque-speed and electrical dynamics;
- wheel radius and robot translation;
- gearbox efficiency and backlash;
- available voltage and current limiting;
- sensor latency/noise and control-loop rate;
- desired disturbance recovery and operating envelope.

The claim that a low center of gravity is automatically better is also not a
valid balancing rule. The center of mass must be above the axle for an inverted
pendulum, and changing its height changes both gravitational torque and falling
dynamics. Battery and board placement must come from the dynamic model, not the
old static calculation.

Phase 1 Stage 2A Q2 did not attempt that full validation. Its later preliminary
dynamic screen found the owned motors and 80 mm wheels
plausible across illustrative scenarios. Nathan requires their reuse while
plausible. This does not validate final Phase 2 mass, center-of-mass placement,
or physical balancing performance; those properties are intentionally deferred
until Phase 2 exists.

## Current Documented Inventory State

As of this audit, the repository and BOM document ownership of:

- two Pololu 50:1 37D encoder gearmotors, item `#4753`;
- one Pololu G2 High-Power Motor Driver 18v17, item `#2991`;
- one pair of 80 mm Pololu wheels;
- one Adafruit BNO055 breakout, item `#4646`.

This inventory has only one of the two independently controllable motor channels
required by Phase 1 differential drive. The owned `#2991` headers have since
been confirmed as loose and unsoldered. Stage 2A Q1 subsequently established
that user soldering of off-the-shelf products is acceptable, so the headers can
be installed; this does not resolve the board's one-channel limitation. The
owned wiring has been classified in the
[Stage 2 hardware decision record](stage-2-hardware-decision-record.md). Q9
later accepted JST-SH/STEMMA and Elegoo leads for downstream low-current
wiring, plus four short parallel Elegoo leads for the DROK-to-Pi path subject
to Stage 2B load rejection criteria. The factory motor harnesses are usable;
motor and battery-side paths still exclude thin jumper/JST-SH wire and use the
Q9 16 AWG harness design.

## Decisions Still Required

The authoritative resolution order is
[Stage 2A in the phase plan](../docs/phases.md#stage-2a-resolve-hardware-questions-in-order).
Detailed
candidate findings—including the provisional recommendation not to purchase a
second `#2991`—are in the
[Stage 2 hardware decision record](stage-2-hardware-decision-record.md). No
provisional recommendation becomes an architectural decision until its stated
manufacturer review and bench acceptance criteria pass.

## Process Finding

The user asked reasonable early-stage questions. The failure occurred because
assistant-generated recommendations were marked "finalized" and converted into
orders without a part-number-to-datasheet check, dimensional check of the
equations, or interface/channel-count review.

Future component decisions require four distinct states:

1. proposed;
2. manufacturer-verified;
3. procured and physically identified;
4. bench-validated in the intended interface.

Agreement in a chat advances none of those states by itself.

## Primary Sources

- [Pololu G2 and H2 motor-driver product families](https://www.pololu.com/category/254/g2-and-h2-high-power-motor-drivers)
- [Pololu G2 High-Power Motor Driver 18v17, item #2991](https://www.pololu.com/product/2991)
- [Pololu G2 High-Power Motor Driver 18v25, item #2994](https://www.pololu.com/product/2994)
- [Pololu Dual MC33926 Motor Driver Carrier, item #1213](https://www.pololu.com/product/1213)
- [Pololu 50:1 37D encoder gearmotor, item #4753](https://www.pololu.com/product/4753)
- [Adafruit DRV8871 breakout, item #3190](https://www.adafruit.com/product/3190)
- [MIT Underactuated Robotics: cart-pole dynamics](https://underactuated.mit.edu/acrobot.html#cart_pole)
- [ROS 2 Jazzy bno055 package documentation](https://docs.ros.org/en/jazzy/p/bno055/)
- [ROS Index: bno055 package](https://index.ros.org/p/bno055/)
- [bno055 ROS release history](https://github.com/ros2-gbp/bno055-release)
- [ROS Index: bno08x_driver repository](https://index.ros.org/r/bno08x_driver/)
- [BNO08x ROS 2 source driver](https://github.com/bnbhat/bno08x_ros2_driver)
