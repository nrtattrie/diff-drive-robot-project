# Stage 2 Hardware Decision Record

**Status:** Q3-Q11 complete; Stage 2A is complete. Pololu Dual VNH5019 shield
`#2507` is selected and received, its Phase 1 motor-control/protection policy
is defined, and the encoder, Due-power, and Raspberry Pi power paths are
selected below. Physical identity and the bench-test equipment were
owner-accepted at B1 and B2 on 2026-07-25. Assembly, commissioning values, and
downstream Stage 2B acceptance remain pending.

**Last reviewed:** 2026-07-25

This is the concise record of the resolved Stage 2A hardware choices.
[The phase plan](../docs/phases.md) defines their Stage 2B execution order. The
[component selection audit](component-selection-audit.md) preserves the longer
history and calculations behind these findings.

## Verified Constraints

- The robot has two Pololu `#4753` 12 V, 50:1 encoder gearmotors and therefore
  needs two independently controlled H-bridges.
- Each motor is specified at 0.2 A no load, 0.66 A at maximum efficiency, and
  5.5 A theoretical stall at 12 V. Pololu's general guidance to keep continuous
  load near or below 25% of stall corresponds to about 1.38 A per motor; stall
  is not a normal operating point.
- The owned Pololu `#2991` is one 17 A motor channel, not a dual driver. Its
  headers were supplied loose; user soldering is now acceptable, but this does
  not make it a sensible two-channel solution for the present motors.
- The Arduino Due uses 3.3 V I/O and its inputs are not 5 V tolerant.
- The motor encoders require at least 3.5 V. Supplying them at 5 V therefore
  requires a verified four-channel 5 V-to-3.3 V interface before the Due.
- The project uses commercial off-the-shelf products assembled by the user.
  Soldering supplied headers and ordinary wiring is acceptable. Hired custom or
  professional assembly is outside the hobbyist scope. Crimping is not assumed.
- Phase 2 self-balancing suitability cannot be proven from a static torque
  calculation. Q2 uses a preliminary wheeled-inverted-pendulum screen only to
  establish whether the owned motors and wheels remain plausible reuse.

## Q2 Analysis: Phase 2 Reuse Boundary

**Status:** Complete. Nathan approved the narrow reuse boundary below.

The physical-property register, equations, reproducible calculation, scenario
results, assumptions, and limitations are recorded in
the [Phase 2 preliminary dynamic model](phase-2-dynamic-model.md).

The planar model includes both wheel traction (`T/r`) and the motor reaction
torque on the body (`-T`). Across the documented 1.0-2.0 kg body-mass,
0.08-0.20 m center-of-mass-height, and body-inertia scenarios, an idealized
10-degree/0.30-second recovery screens at:

- 0.027-0.094 N·m output torque per motor;
- approximately 0.15-0.44 A per motor using two fits that bracket conflicting
  Pololu no-load-current data;
- 65-133 rpm wheel demand, compared with approximately 150-157 rpm available
  in the 9.6 V reduced-voltage sensitivity case; and
- 52-115 ms open-loop e-fold time.

These assumed scenarios are a plausibility screen, not final Phase 2 properties
or requirements.

### Approved reuse boundary

- **Required reuse while plausible:** both owned Pololu `#4753` motors,
  including their integrated encoders, and the owned 80 mm wheels with 6 mm
  hubs. The screen found no reason to replace them, so no new motors or wheels
  are planned. Replacement is reconsidered only if later physical Phase 2
  evidence demonstrates infeasibility.
- **Outside Q2:** every other Phase 1 component, Phase 2 physical property,
  chassis decision, controller choice, sensor choice, and power decision.
  Select and validate current hardware for Phase 1 without requiring knowledge
  of the unbuilt Phase 2 robot.
- **No Q3 dependency:** Q2 did not select or size a motor driver. Q3 was
  completed separately afterward.

## Q3 Analysis: Two-Channel Motor Drive

**Status:** Complete. Nathan approved Pololu `#2507` on 2026-07-23 with the
implementation conditions recorded below.

### Scope and selection standard

Q3 selects a two-channel driver for the known Phase 1 robot: two `#4753`
motors, an Arduino Due, and a 3S battery architecture that can reach 12.6 V.
The driver should also remain credible for a later Phase 2 that continues to use
the same motors and wheels. This does not assume or decide any other Phase 2
property.

Useful future-readiness means:

- enough electrical and thermal margin that the driver is not the first
  component forced to its limit;
- a Due-safe interface and a command path credible for a faster future control
  loop;
- observable current, voltage, and fault state where practical;
- explicit system-level treatment of current limiting and command loss when
  those functions are not independently provided by the driver; and
- an off-the-shelf installation that does not require custom fabrication or an
  unresolved high-current connector.

Raw current rating by itself is not useful future-readiness. The fixed motors
still draw only what their voltage, winding resistance, speed, and load demand.
An oversized driver is acceptable when its extra capability comes with usable
protection and diagnostics rather than merely increasing the available fault
current.

### Current product comparison

Prices below are manufacturer one-piece prices reviewed on 2026-07-22 and
exclude tax and shipping.

| Exact candidate | Relevant interface and capability | Safeguards, assembly, and cost | Q3 result |
|---|---|---|---|
| Pololu Motoron M2S18v18 with connectors soldered `#5036` | Dual 18 A channels, 6.5-30 V motor supply, 3.0-5.5 V logic, I2C up to 400 kHz, selectable 1-80 kHz internal PWM. A `Set all speeds` command changes both channels together and its `now` mode applies the new output immediately. | Configurable hardware current limit, current and VIN measurement, CRC, configurable error response, command timeout, and motor-supply reverse protection. Factory-soldered stack headers and three motor/power terminal blocks; **$69.95**. It does not have meaningful over-temperature protection. | **Not selected.** Its current limit, timeout, and telemetry are useful, but they do not justify exceeding the under-$50 board budget for this project. |
| **Pololu Dual VNH5019 shield `#2507`** | Direct PWM up to 20 kHz, 12 A continuous/30 A peak per channel, 3.3 V-compatible inputs, and approximately 140 mV/A current sense. Pololu has tested the shield and library with the Due and identifies VNH5019 drivers as a good match for the owned `#4753` motors. | Reverse protection, thermal shutdown, over/under-voltage shutdown, and output-short protection, but no motor-matched configurable current limit or independent command timeout. Included headers and terminals require soldering; **$39.95**. | **Selected.** It meets Phase 1 actuation needs, stays under the board budget, and preserves direct control and adequate margin for the same motors in a plausible later Phase 2. The five implementation conditions below are mandatory. |
| Pololu Dual TB9051FTG shield `#2520` | Direct 20 kHz PWM, 2.6 A continuous/5 A peak per channel, about 500 mV/A current sense, and IOREF-based 3.3 V logic. | Fixed current chopping is typically about 6.5 A, with thermal current reduction, over-temperature/over-current diagnostics, and reverse protection. Included headers and terminals require soldering; **$31.95**. Its current-monitor voltage approaches the Due's 3.3 V ADC limit near fault current. | **Good Phase 1 value, not the best carry-forward choice.** Less transient and thermal margin, and the current-sense interface needs more care. |
| Pololu Dual MC33926 shield `#2503` | Direct 20 kHz PWM, 3 A continuous/5 A peak per channel, 3.3 V logic, and approximately 525 mV/A current sense. | Internal peak limiting around 6.5 A ±1.5 A, thermal and output-short protection, and motor-supply reverse protection. Included headers and terminals require soldering; **$44.95**. Product status is active rather than active-and-preferred. | **Superseded in this comparison.** It costs more than the TB9051 and VNH5019 alternatives without winning either the protection/fit or future-headroom comparison. |
| Pololu Dual G2 18v18 shield `#2515` | Direct PWM up to 100 kHz, 18 A continuous per channel, 3.3 V logic, and approximately 20 mV/A current sense. | Reverse, undervoltage, and short-circuit protection. The default current limit is approximately 50 A; it can be lowered with resistors but is less accurate at low settings. There is no dependable MOSFET over-temperature shutdown. Included connectors require soldering; **$74.95**. | **Not recommended.** Its raw output capacity is not accompanied by a motor-matched default limit, strong low-current telemetry, or the Motoron's independent timeout. |
| A second Pololu G2 18v17 `#2991` | Two separate direct-PWM 17 A boards would be required. The second board alone is **$44.95**; the first is already owned. | Each board has a roughly 40 A default current limit, approximately 20 mV/A current sense, and no dependable MOSFET over-temperature shutdown or command timeout. | **Not selected.** It is electrically usable, but the `#2507` provides two channels, stronger thermal protection, and better low-current feedback for less new expenditure. Reserve the owned `#2991` for later one-channel use. |
| Cytron MDD10A Rev 2.0 | Direct 20 kHz PWM, 10 A continuous/30 A peak per channel, 3.3 V inputs, and factory-soldered power terminals; **$25.90**. | No motor-supply reverse-polarity protection, current feedback, configurable current limit, or independent command timeout. Its supplied logic plug uses loose crimp terminals, while crimping is not assumed. | **Budget option only, not recommended.** It removes the diagnostics and fail-safe features that justify deliberate Phase 1 overengineering. |

Two DRV8871-class boards and Raspberry Pi expansion boards remain excluded:
the former give up transient margin without a compelling installed-cost
advantage, and the latter put the wrong controller in the motor-command path.

### Approved exact implementation

Select **one Pololu Dual VNH5019 Motor Driver Shield for Arduino, item
`#2507`**. Although sold as a shield, install it as a separately mounted
general-purpose motor-driver board. This avoids relying on the shield's 5 V
logic supply for the enable/fault pull-ups and permits deliberate Due pin
allocation for motor control and the two quadrature encoders.

### Connection and assembly boundary

The surface-mount electronics arrive assembled, but Pololu does not offer a
factory-soldered `#2507`. For the selected general-purpose configuration,
Nathan will solder the three included 2-pin motor/power terminal blocks and a
13-pin section of the included logic header: 19 through-hole joints. Do not
install the Arduino stackable headers or the optional `ARDVIN=VOUT` power
jumper.

- Supply driver `VDD` from the Due's 3.3 V rail and join driver and Due ground.
  Connect each channel's PWM, two direction inputs, enable/fault output, and
  current-sense output through the logic header. The exact Due pins are fixed
  in the [Q10 interface matrix](power-signal-interface-matrix.md).
- Connect the two motor-power leads from one `#4753` to `M1A/M1B` and the other
  motor to `M2A/M2B`. Establish final channel names and polarity during the
  unloaded direction test.
- Connect only the future protected motor branch to the driver's large
  `VIN/GND` terminal. Do not use the driver to power the Due or Pi. Q6-Q8
  define their supplies and protection policy; Q9 owns the physical
  distribution design.
- Mount the board using its two #4 mounting holes and provide strain relief for
  the motor and power leads. Do not use the owned `#2991` in parallel or as a
  third channel.

### Mandatory implementation conditions

1. **Separate 3.3 V logic installation:** use the `#2507` as the separately
   mounted general-purpose board described above, with `VDD` connected to the
   Due's 3.3 V rail. This avoids ambiguity about 5 V fault-line pull-ups and
   allows the motor and encoder pins to be allocated deliberately.
2. **Use the board-protected current-sense outputs:** connect `M1CS` and
   `M2CS` directly to Due A0/A1 through the `#2507` logic header. The board
   already places 10 kΩ in series with each output, which Pololu considers
   generally safe for 3.3 V MCU inputs. At the motors' 5.5 A theoretical stall
   current, the signal is only about 0.77 V; reaching 3.3 V would require about
   23.6 A on one channel, far outside these motors' electrical envelope. No
   external clamp diode is required. Reconsider this only if the motor/current
   envelope changes.
3. **Resolve Due PWM and current-sense sampling:** the stock Pololu library
   does not guarantee 20 kHz PWM on the Due, and Pololu warns that its current
   readings are unreliable below about 5 kHz without filtering. Q4 selects
   20 kHz Due hardware PWM, so the low-frequency capacitor is not required;
   Q10 assigns D6/D7 for hardware PWM and A0/A1 for current sense; Stage 2B
   must verify PWM and current readings.
4. **Provide system-level motor protection:** the `#2507` has no configurable
   motor-matched current limit or independent command timeout. Q4 defines the
   firmware command-loss, driver-fault, and encoder-stall responses below,
   while Q8 provides the selected system fuse. These functions may not
   be assumed from the board's 12 A rating or its short-circuit protection.
5. **Keep thin owned wire out of motor power:** JST-SH, STEMMA QT, and Dupont
   leads remain signal-only. The final battery and motor-power harness remains
   a Q9 decision based on limits, the Q8 fuse value, run lengths, and terminals.

Q3 chooses the hardware and connection form only. The Q4 resolution below
defines the associated Phase 1 operating policy.

## Q4 Resolution: Motor Protection and Control Policy

**Status:** Complete as a Stage 2A design decision. Stage 2B still owns
measurement, calibration, and bench acceptance.

### Scope and reference comparison

Phase 1 operates indoors on smooth, substantially level surfaces. It does not
have a steep-slope holding requirement. Q4 does not impose a Phase 2 balancing
policy.

Comparable robots support a local stale-command stop but do not share a
universal electrical-braking sequence. TurtleBot3 sends zero motor velocity
after a 500 ms control timeout; Linorobot2 uses a 200 ms command timeout and
makes short braking optional; Pololu's Balboa balancing example continuously
controls ordinary encoder-equipped DC gearmotors and its zero command brakes
because of that platform's driver topology. None establishes a timed
brake-then-coast requirement for this Phase 1 robot.

- [TurtleBot3 OpenCR timeout configuration](https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_burger/turtlebot3_core/turtlebot3_core_config.h)
- [Linorobot2 firmware and configuration](https://github.com/linorobot/linorobot2_hardware/tree/jazzy/firmware)
- [Pololu Balboa balancing example](https://github.com/pololu/balboa-32u4-arduino-library/tree/master/examples/Balancer)

### Selected Phase 1 policy

| Concern | Decision | Stage 2B evidence still required |
|---|---|---|
| Motor voltage and PWM | Accept a 12.6 V fully charged 3S source and initially cap motor duty at 95%, corresponding to approximately 12.0 V maximum average motor voltage. Generate 20 kHz PWM using Due hardware rather than assuming the stock Pololu library provides it. | Measure frequency, duty, polarity, and motor-terminal behavior first with one unloaded channel. |
| Continuous and transient current | Use approximately 1.38 A per motor, 25% of the 5.5 A theoretical stall current, as the continuous design target. Startup and acceleration may briefly exceed it while encoder progress is present. The 5.5 A stall value is not an allowed operating point. Do not invent a current-versus-time shutdown curve from unsupported data; use current sensing for measurement and diagnosis while encoder-stall logic provides the primary motor-protection response. | Record unloaded, startup, maneuvering, and representative loaded currents. Normal loaded duty must remain near or below the continuous target. |
| Normal commanded stop | Apply a bounded controller deceleration to a zero wheel-speed target, then command the VNH5019 coast state near standstill. Do not apply a fixed active-braking pulse. | Tune the deceleration limit and near-zero transition to avoid wheel slip, excessive stopping distance, or oscillation. |
| Startup, reset, and command loss | Initialize both channels in coast before accepting motion. A 500 ms Due-side serial-command watchdog sets both wheel targets to zero and proceeds to coast; a later valid command may restore motion without a manual reset. Never resume a pre-reset or stale command. | Verify power-up, Due reset, firmware upload, serial disconnect, and reconnect behavior with wheels clear of the surface first. |
| Driver fault | A fault indication from either channel commands both channels to coast and latches motion off until an explicit reset after the cause is removed. Stopping both avoids an unintended powered pivot. | Inject or safely simulate the readable fault states without intentionally shorting the driver. |
| Encoder stall | After an empirically determined startup grace period, a meaningful motion command with inadequate encoder progress commands both channels to coast and latches a stall fault. Current is corroborating telemetry, not the sole trigger. | Determine the arming command, minimum progress, grace period, and detection interval from unloaded and representative loaded tests. |
| Thermal acceptance | Do not assign unsupported fixed motor-case or driver-board temperature limits. A representative loaded duty cycle passes only if current remains consistent with the continuous design target, temperatures approach a stable trend, and no driver fault, controller reset, or progressive performance loss occurs. | Record ambient, motor and driver temperature behavior, current, voltage, run duration, and faults; reject a configuration whose temperatures continue rising without a credible plateau. |
| Current-sense interface | At 20 kHz, do not add the capacitor Pololu recommends only for PWM below 5 kHz. Connect the shield's 10 kΩ-isolated current-sense outputs directly to Due A0/A1 without external clamp parts; the selected motors produce only about 0.77 V at theoretical stall. Firmware may average calibrated samples. | Q10 assigns A0/A1 for current sense and D6/D7 for PWM. Stage 2B verifies ADC level, zero offset, scaling, noise, and useful readings under PWM. |

The unmeasured tuning values above are commissioning parameters rather than
unresolved architecture choices. Keeping them in Stage 2B preserves the
distinction between defining a testable policy and claiming evidence that can
exist only after the hardware arrives.

## Q5 Resolution: Encoder Interface

**Status:** Complete as a Stage 2A design decision. Nathan selected the
Adafruit `74LVC245` product `#735` and Adafruit 20-pin socket `#2204` on
2026-07-23. They have arrived and were owner-accepted at B1; assembly and
powered bench acceptance remain pending.

### Why an interface is required

Each `#4753` encoder requires 3.5-20 V and produces two digital outputs that
swing from ground to its supply voltage. The Due's I/O limit is 3.3 V, so the
encoder cannot be powered within specification and connected directly to the
Due. Power both encoders from a regulated **5 V encoder rail** and translate
their four outputs to 3.3 V before they reach the Due.

The selected `74LVC245` is a fixed-direction digital buffer/level translator,
not an encoder decoder. Operated from the Due's 3.3 V rail, it accepts inputs
up to 5.5 V and produces 3.3 V logic outputs. Its specified propagation delay
of at most 6.3 ns at 3.3 V is negligible relative to the encoder signals:
at the motor's 200 rpm no-load output speed and exact 50:1 ratio, each encoder
channel is approximately 2.67 kHz and full quadrature decoding produces about
10.67 thousand count events per second per motor. Count integrity remains a
Stage 2B measurement rather than an assumed result.

### Selected implementation

- Use one Adafruit `74LVC245` 8-bit logic level shifter, product `#735`, in one
  Adafruit 20-pin 0.3-inch IC socket, product `#2204`. The socket must be
  soldered to a suitable prototyping board; the IC then plugs into the socket.
  The owned Mega Prototype Shield V3 is the preferred carrier if visual and
  continuity inspection confirms its 0.1-inch prototyping area and rail
  routing. Otherwise use a small solderable perfboard.
- Connect `74LVC245` pin 20 (`VCC`) to Due 3.3 V and pin 10 to common logic
  ground. Connect pin 1 (`DIR`) to 3.3 V and active-low pin 19 (`/OE`) to
  ground for permanent A-to-B operation.
- Install one 0.1 uF ceramic bypass capacitor directly between pins 20 and 10.
  Tie unused A-side inputs A5-A8, pins 6-9, to ground; leave unused B-side
  outputs B5-B8, pins 11-14, unconnected.
- Do not substitute a `74HC245`, `74HCT245`, `TXB0108`, or an unspecified
  auto-direction translator. The approved retail item is specifically
  Adafruit `#735`, whose `74LVC245` behavior matches this fixed 5 V-to-3.3 V
  application.

### Encoder and Due mapping

| Motor signal | Translator path | Due hardware input |
|---|---|---|
| Left yellow, encoder A | pin 2 `A1` to pin 18 `B1` | D2, `TIOA0` |
| Left white, encoder B | pin 3 `A2` to pin 17 `B2` | D13, `TIOB0` |
| Right yellow, encoder A | pin 4 `A3` to pin 16 `B3` | D5, `TIOA6` |
| Right white, encoder B | pin 5 `A4` to pin 15 `B4` | D4, `TIOB6` |

D2/D13 form one SAM3X Timer Counter quadrature-decoder pair and D5/D4 form a
second. Direction sign is established during the unloaded rotation test rather
than inferred from chassis orientation. Because D2 and D4 are default `#2507`
shield-control pins, the separately mounted driver's control lines do not use
the default shield mapping; Q10 assigns D6-D11, D22-D23, and A0-A1.

For each motor harness, red and black remain motor-power leads and go to that
channel's `#2507` output terminal. Green joins common logic ground, blue joins
the regulated 5 V encoder rail, and yellow/white enter the translator. The
six-pin motor harness does not plug into either the IC or the motor driver as a
unit. Q9 and the Q10 matrix define the secured connector implementation; Stage
2B still inspects the owned prototype shield and harnesses before power.

Q6 selects the Due's 5 V output as the encoder rail during both bench and mobile
operation. The Due is powered through its Programming USB port, so do not feed
an external supply into the Due 5 V pin. The translator itself always uses Due
3.3 V, and the Due, translator, both encoders, and `#2507` logic must share
ground. Never power an encoder from the 12 V motor rail while it is connected
to the translator, whose inputs are limited to 5.5 V.

### Stage 2B acceptance

Before motor power is applied, verify socket orientation, continuity, absence
of 5 V at every Due input, and correct 5 V/3.3 V logic levels while each wheel
is turned by hand. Then test one unloaded motor and both unloaded motors through
their full allowed speed range, compare observed direction and counts with the
expected 3200 counts per output-shaft revolution, and reject any circuit that
shows missed, extra, or direction-reversed counts that cannot be explained by
the defined polarity convention.

## Q6 Resolution: Arduino Due Power

**Status:** Complete as a Stage 2A design decision. Nathan approved the
USB-powered implementation below on 2026-07-23. No additional Due power
converter is selected or required; cable and bench acceptance remain pending.

### Selected bench and mobile method

Use the Due as a bus-powered Raspberry Pi USB peripheral:

- **Bench:** power the Due through its Programming USB port from the Raspberry
  Pi or, for isolated Due testing, from one development computer. Keep the motor
  driver's current-limited bench supply separate while joining its ground to
  Due ground before control signals are connected.
- **Mobile:** connect one Raspberry Pi USB-A host port to the Due's Programming
  USB port with a data-capable USB cable. This one cable supplies the Due and
  carries its serial command/telemetry connection.
- Use the Due's 5 V output for the two encoders, whose maximum combined draw is
  20 mA. Use the Due's 3.3 V output for the `74LVC245` and the separately
  mounted `#2507` logic `VDD`. Join Due ground to both encoder grounds, the
  translator, and the `#2507` ground. Q9 owns the physical ground distribution
  so motor return current does not travel through logic wiring.

No direct connection from the motor battery or owned DROK converter to the Due
is part of this design. In particular:

- do not feed the historical 5.1 V rail into `VIN`; the specified external
  supply range is 7-12 V;
- do not apply raw 3S battery voltage to `VIN` or the barrel jack; a fully
  charged 3S battery reaches 12.6 V, above the recommended range and near the
  high-dissipation end of the Due's linear-regulator path; and
- do not use the Due's 5 V or 3.3 V pins as power inputs. Arduino advises
  against this because it bypasses onboard regulation and protection.

A separate regulated 7.5 V barrel-jack supply would be electrically valid but
is intentionally rejected as redundant for Phase 1. If a later Phase 2
establishes a measured need for power independence from the Pi, it can add a
separate logic supply without changing the Q5 encoder interface.

### USB coexistence and current boundary

- Only one USB host may connect to and power the Due at a time. Normally that
  host is the Pi. Before connecting a laptop directly, disconnect the Pi-to-Due
  USB cable; do not use the Programming and Native USB ports from two powered
  hosts simultaneously.
- Program and communicate with the Due from the Pi over the same Programming
  USB cable when practical. A Pi reboot does not create a stale-command
  exception: Q4's 500 ms watchdog and reset-safe coast behavior still apply.
- The Due's USB input has a 500 mA resettable polyfuse. Treat 500 mA as the
  conservative unmeasured ceiling for the complete Due/encoder/translator
  branch until Stage 2B records its actual consumption.
- Raspberry Pi 5 limits total downstream USB peripheral current to 600 mA with
  a 3 A supply and permits 1.6 A when it recognizes a 5 A supply such as the
  official 27 W unit. The official unit therefore supports bench work. Q7 must
  account for the measured Due branch and all other USB peripherals; the Q7
  mobile path now does so, subject to Stage 2B load validation.

### Stage 2B acceptance

With motor power initially disconnected, verify the Due enumerates on the Pi,
uploads firmware, maintains serial communication, and presents acceptable 5 V
and 3.3 V rails with both encoders and the translator connected. Record the
branch current if suitable measurement equipment is available. Then verify
Due/Pi startup order, Pi reboot, USB disconnect/reconnect, Due reset, firmware
upload, and motor startup do not cause unintended drive, repeated controller
resets, USB over-current, or corrupted encoder counts. Perform direct-laptop
testing only after disconnecting the Pi USB host.

## Q7 Resolution: Raspberry Pi Power

**Status:** Complete as a Stage 2A design decision. Nathan approved the owned
DROK mobile-power path below on 2026-07-23. No additional Raspberry Pi power
board is selected; physical acceptance remains in Stage 2B.

### Selected bench and mobile methods

- **Bench/tethered:** power the Raspberry Pi 5 directly from the owned official
  27 W USB-C supply. The Pi then powers and communicates with the Due through
  the Q6 USB connection.
- **Mobile:** use a protected 3S battery distribution branch, defined in Q8, to
  feed the owned DROK adjustable buck converter. Set its output to **5.10 V**
  before connecting the Pi, then feed the Pi through its 5 V and ground header
  pins using short, suitably heavy conductors. The Pi continues to power and
  communicate with the Due over USB.
- Do not use the DROK for the motor branch. Do not connect the official USB-C
  supply while the Pi is powered through its header.

Direct header power avoids USB-C Power Delivery negotiation. After the entire
mobile path has demonstrated a sustained 5 A capability, configure the Pi
bootloader with `PSU_MAX_CURRENT=5000` so the Pi permits the higher USB
peripheral-current budget. Do not use that setting as a substitute for proving
the converter, wiring, connections, and battery branch.

The Pi 5's unusual aspect is its optional 5 V/5 A mode: a generic high-wattage
USB-C source may provide its headline power only at higher voltages and leave
the Pi in the lower USB-current mode. Hobbyist mobile robots commonly avoid
that ambiguity either with a capable buck converter connected to the 5 V
header or with a Pi-specific USB-C PD board. The latter adds connector and
automatic-negotiation convenience, but is not necessary if the owned DROK path
passes validation.

### Implementation and acceptance boundary

Before the DROK ever powers the Pi:

1. Identify its exact variant and physical markings, inspect the board, and
   confirm its documented input range and credible sustained-output capability.
2. Set and independently measure 5.10 V with the Pi disconnected. Repeat power
   cycles and reject the unit if its output overshoots or does not return to a
   safe, stable value.
3. Validate regulation and temperature progressively, measuring voltage at the
   end of the actual Pi power connection rather than only at the converter.
4. Exercise Pi boot, CPU, NVMe, camera, Due, and later LiDAR loads, then repeat
   while the motor branch starts and runs. Reject a path that produces Pi
   undervoltage warnings, USB over-current, resets, unsafe temperature, or
   unstable output.
5. Secure and cover the DROK adjustment controls after setting them. Never
   adjust the converter with the Pi connected.
6. Use `PSU_MAX_CURRENT=5000` only after the complete path passes the 5 A
   validation, and verify that the Pi reports the high-current USB mode.

The available information does not establish an independent fail-high output
over-voltage cutoff on the owned DROK. Adjustment control, pre-connection
measurement, repeated power cycling, and physical securing address the
credible setup risks without adding another board. If exact-unit inspection or
testing shows inadequate regulation, thermal performance, sustained current,
or unsafe startup behavior, Q7 must be reopened and a dedicated protected
5 V/5 A supply selected.

Q7 selects the conversion topology only. Q8 supplies the minimal upstream
protection policy below. Q9 owns ground distribution, branch wiring, and
physical parts; Q10 records the completed interface matrix.

## Q8 Resolution: Power Protection Policy

**Status:** Complete. Nathan clarified on 2026-07-23 that cables, wire gauges,
terminals, connectors, splices, and physical distribution hardware belong to
Q9 rather than Q8.

The Q8 protection decisions are:

- install one 10 A standard ATO/ATC fuse in the battery-positive feed, as close
  to the battery as the Q9 harness implementation reasonably permits;
- place both the motor-driver and DROK branches downstream of that fuse; and
- do not add branch fuses, a fused distribution block, or another protection
  board merely to duplicate the main fuse;
- use the keyed battery connector selected in Q9 as the normal manual power
  disconnect rather than adding a separate emergency-stop or high-current
  switch;
- rely on that keyed connector and the selected driver's built-in protection
  rather than add another reverse-polarity board; and
- use the `#2507` as supplied without another local capacitor unless Stage 2B
  measurements reveal an actual supply-transient problem.

The owned 1 A toggle is not approved to interrupt battery current. A
battery-specific undervoltage protection or monitoring method must be selected
with the deferred 3S battery before Stage 4 mobile operation; it does not
justify adding another unspecified board to the Stage 2 Q8 design.

## Q9 Resolution: Physical Power Distribution and Owned-BOM Gap Check

**Status:** Complete by Nathan's direction on 2026-07-23. This closes the
physical-part selection; it does not waive the staged electrical acceptance
tests required before mobile use.

Use the following implementation:

- use 16 AWG stranded copper for the battery trunk and battery-side branches;
- place the selected 14 AWG inline ATO/ATC holder and Q8 10 A fuse near battery
  positive;
- use the eventual battery's keyed/polarized mating connector, rated for at
  least the 10 A fused path, as the normal manual disconnect; its exact SKU is
  intentionally selected with the still-deferred battery;
- use the owned ALITOVE splice connectors for the fused distribution junction
  where their actual internal topology permits it. Nathan reports markings up
  to 10 AWG and 32 A. Continuity-check whether the ports form a common junction
  or independent pairs before wiring, then individually tug-test every wire;
- land the motor factory leads and battery feed directly in the supplied
  `#2507` terminal blocks. Do not extend a motor or battery path with JST-SH or
  Dupont jumper wire;
- feed the Pi from the validated DROK output using two equal-length, short
  Elegoo/Dupont leads in parallel to the two Pi 5 V pins and two equal-length,
  short Elegoo/Dupont leads to separate Pi ground pins. Keep each lead at or
  below approximately 12 inches; and
- use the owned JST/STEMMA cables or Elegoo jumpers for the remaining
  low-current power and signal wiring downstream of the Pi, including the
  controller, level translator, driver logic, encoders, and sensors, subject to
  the Q10 pin/interface matrix. The Pi-to-Due connection remains the Q6
  data-capable USB cable. JST/STEMMA cables are used only where a matching
  socket exists; none is forced into the current baseline.

The Elegoo lead gauge and contact rating remain undocumented. Nathan has
accepted their use for the Pi-power connection with caution. Stage 2B must
therefore check polarity and continuity before connection, measure the DROK
both unloaded and at the Pi header under load, monitor the four jumpers and
contacts for heating, check for Pi undervoltage/reset indications, and reject
the arrangement if it cannot sustain the Q7 load test. Failing this acceptance
test requires replacement with rated 22 AWG pre-terminated leads; it does not
reopen the Q9 topology decision.

Owned-item disposition:

- the ALITOVE splices are the selected distribution/splice method after the
  topology and mechanical checks above;
- the white screw-terminal strips remain optional secured bench fixtures, not
  required mobile distribution hardware;
- the 1 A toggle is available only for a future low-current enable/control
  function and is not in the selected power path;
- the INA219 is optional for controlled low-current measurements and is not
  the main battery/motor current sensor;
- the DROK is assigned exclusively to the Q7 mobile Pi branch; and
- the owned JST/STEMMA and Elegoo leads cover downstream low-current
  power/signals. BOM REV 02 records the selected 16 AWG red/black copper as
  owned; the battery-mating connector remains battery-selection-dependent;
  and
- the owner-reported Adafruit `#2998` is used bus-powered from one black Pi USB
  2.0 port for only the `#954`/BNO055 branch. Its external-power jack remains
  unused; the Due and future RPLidar stay on direct Pi ports.

## Q10 Resolution: Power and Signal Interface Matrix

**Status:** Complete as a Stage 2A design definition on 2026-07-23. Physical
continuity, voltage, timing, current, temperature, reset, and fault behavior
remain Stage 2B evidence.

The authoritative
[Phase 1 power and signal interface matrix](power-signal-interface-matrix.md)
records every installed or planned baseline interface, including direction,
voltage/current limits, thresholds, channel count, Due pins, connector/wire,
protection, shutdown state, and controlling device.

Key allocations are:

- Due D6/D7 for 20 kHz M1/M2 PWM; D8-D11 for direction; D22/D23 as
  input-only driver fault lines; and A0/A1 for current sense;
- direct `#2507` current-sense connections to A0/A1 through the board's
  existing 10 kΩ series resistors, with no external clamp parts;
- the existing Q5 D2/D13 and D5/D4 encoder pairs through the `74LVC245`;
- the owned Adafruit `#954` USB-to-TTL cable for BNO055 UART at 115200 baud;
- the owner-reported owned Adafruit `#2998` as a bus-powered switched hub
  between a Pi USB 2.0 port and the low-current `#954`/BNO055 branch;
- the Pi's CAM/DISP connector and a 22-to-15-pin Standard-to-Mini cable for
  Camera Module 3; and
- a direct Pi USB connection through the future RPLidar A1 development-kit
  adapter, included in the validated Pi USB/power budget.

The matrix explicitly excludes optional parts that have no baseline function.
Q10 adds no unconditional procurement item: external current-sense clamps are
unnecessary for these motors, the camera bag contains the required Pi 5
Standard-to-Mini cable, and the `#2998` is already owned. The
USB-A-to-micro-B data cable for the Due must be physically confirmed and
purchased only if absent.

## Q11 Resolution: Procurement Record

**Status:** Complete by Nathan's direction on 2026-07-23.

[BOM REV 02](Bill%20of%20Materials%20-%20Sheet%20REV%2002.pdf) is the accepted
current procurement snapshot. The previous PDF remains recoverable in
`hardware/_archive`. No further BOM cleanup is required to close Stage 2A;
physical validation remains governed by Stage 2B rather than the BOM.

## Q1 Resolution: Assembly Boundary and Owned Wiring

Q1 is closed. The inventory classification below establishes what is usable
now and includes the later Q9 assignments.

| Owned item | Documented rating or construction | Accepted use and termination |
|---|---|---|
| Two `#4753` factory motor/encoder harnesses | Six color-coded 20 cm leads terminating in one 1x6 female 2.54 mm header. The encoder draws at most 10 mA; Pololu does not publish the individual lead gauges. | The original harness is usable. Keep the four encoder conductors as low-current signal/power leads. The red/black motor leads may be removed from the housing or cut, stripped, and landed in the selected driver's rated terminal or soldered to an appropriate off-the-shelf connector. Do not extend the motor pair with Dupont or JST-SH wire. |
| 3x Pololu `#5534` five-pin female-female cables | 28 AWG, 1 A, 50 V, 16 cm, JST-SH-type 1 mm connectors. | Signal and low-current sensor power only. Plug into matching JST-SH headers; do not use for motors, battery, Pi power, or a shared power bus. |
| 3x Pololu `#5530` five-pin single-ended cables | 28 AWG, 1 A, 50 V, 12 cm, female JST-SH-type connector to bare leads. | Signal and low-current sensor power only. The bare end may be soldered or temporarily clamped in a suitable low-current terminal after pinout verification. |
| 3x 100 mm and 2x 200 mm Adafruit STEMMA QT cables | Four-conductor JST-SH 1 mm I2C cables; Adafruit specifies sensor power, ground, SDA, and SCL use but does not publish a cable current rating. | Use only for intended STEMMA QT/Qwiic sensor interfaces such as the BNO055 and INA219. Not general power wire. |
| Assorted Elegoo/Dupont jumper leads | Gauge and contact-current rating are not documented in the BOM. | Q9 accepts two short parallel positive and two short parallel ground leads for DROK-to-Pi power, subject to explicit Stage 2B voltage-drop, heating, and full-load rejection criteria. Also accepted for low-current power and signals downstream of the Pi. Never use them for a motor or battery-side path. |
| ALITOVE 3-in/3-out lever splice connectors | Nathan reports physical markings up to 10 AWG and 32 A; a traceable manufacturer datasheet is not recorded. | Q9 accepts them on the 10 A fused distribution path after continuity confirms the required internal topology and every termination passes an individual tug test. Reject any connection that loosens or heats during Stage 2B loading. |
| Weewooday 12-position screw-terminal strips | Listing claims 10 A, 380 V, and wire up to 2.5 mm²; no traceable manufacturer datasheet or vibration qualification is available. | Acceptable as a secured, inspected bench terminal below its rating with a current-limited/fused source. Not accepted as the final mobile battery bus. Use properly stripped bare wire under the clamp and perform a tug test; do not tin stranded wire that will be compressed by a screw. |
| Mini SPDT toggle switch | BOM states up to 1 A at 24 V. | Use only as a low-current enable/control input unless a suitable switching design is established. |
| INA219 ±3.2 A breakout | Useful for low-current branches or controlled single-channel measurements; inadequate for the possible 11 A combined motor stall current. | Define its measurement location and fuse it for that branch. |
| DROK adjustable buck converter | Selected by Q7 for the mobile Raspberry Pi power branch; exact variant and physical capability remain unvalidated. | Set to 5.10 V and complete the Q7 staged regulation, thermal, startup, and load tests before connecting the Pi. Do not use it for the motor branch. |
| Official Raspberry Pi 27 W USB-C supply | Appropriate for early bench work. | Keep it independent of motor power during initial bring-up. |
| Adafruit USB Mini Hub with Power Switch `#2998` | Four-port bus-powered USB 2.0 hub with one switch for all downstream ports; owner-reported as owned but omitted from BOM REV 02. Its 1.35 mm external jack is directly tied to USB 5 V. | Connect its captive USB-A lead to one black Pi USB 2.0 port and use one marked downstream socket for `#954`/BNO055. Leave the other three sockets spare and the external-power jack empty. It adds ports, not current budget. |

**Wire verdict:** Q9 assigns the owned Elegoo leads to low-current downstream
wiring, with the special four-lead Pi-power arrangement subject to Stage 2B
rejection criteria. The five-pin JST-SH cables have no matching installed
endpoint and the STEMMA QT cables remain spare for future I2C experiments.
The motors' factory leads remain usable. Use 16 AWG copper for the battery
trunk and battery-side branches, the selected 14 AWG inline fuse holder, and
no thin jumper or JST-SH lead in a motor or battery-side path.

## Resolution Order

The authoritative unchecked list is
[Stage 2A in the phase plan](../docs/phases.md#stage-2a-resolve-hardware-questions-in-order):

1. ~~Confirm the assembly boundary and inventory connectors.~~ Completed: user
   soldering of off-the-shelf products is allowed; owned thin cables are
   signal-only and the high-current harness is a documented later purchase.
2. ~~Establish the Phase 2 reuse boundary.~~ Completed: the owned `#4753`
   motors and 80 mm wheels/hubs are required reuse while plausible; all other
   Phase 2 reuse and physical-property decisions are outside Q2.
3. ~~Select the two-channel driver.~~ Completed: Pololu `#2507` selected for
   separate 3.3 V general-purpose installation under the five mandatory
   implementation conditions above.
4. ~~Define motor limits, fault behavior, and command timeout.~~ Completed:
   20 kHz/95% initial drive policy, continuous-current target, coast-based stop,
   500 ms command watchdog, latched fault/stall response, and trend-based
   thermal acceptance are defined above; empirical tuning remains in Stage 2B.
5. ~~Resolve encoder level shifting.~~ Completed: power both encoders at 5 V
   and translate all four signals through one socketed Adafruit `74LVC245`
   product `#735` into the Due's two hardware quadrature-decoder pin pairs.
   Q6 supplies the final mobile 5 V source, and Q9/Q10 define connectors and
   the complete pin matrix. Stage 2B verifies count integrity.
6. ~~Resolve Due power.~~ Completed: power and communicate with the Due through
   one data-capable Programming USB connection, from the Pi normally or one
   development computer during isolated testing. The Due supplies encoder 5 V
   and translator/driver-logic 3.3 V; no separate Due regulator is selected.
   Q7 includes this USB load in the Pi's protected mobile power budget.
7. ~~Resolve protected mobile Pi power.~~ Completed: use the official 27 W
   USB-C supply for bench work and a protected 3S branch through the owned DROK
   at 5.10 V into the Pi 5 V/ground header for mobile use. No additional Pi
   power board is selected; Stage 2B must validate the complete path before use.
8. ~~Close the power-protection policy.~~ Completed: use one 10 A main fuse,
   the keyed battery connector as the manual disconnect, and no branch fuse
   block, added reverse-polarity board, separate emergency stop, or extra
   driver capacitor. Battery-specific undervoltage handling is required with
   the deferred Stage 4 battery purchase rather than selected speculatively.
9. ~~Select the physical cables, wire gauges, terminals, connectors, splices,
   and distribution hardware; assign or reject every owned BOM component.~~
   Completed: 16 AWG battery-side wiring, the 14 AWG inline fuse holder,
   continuity-checked ALITOVE distribution, `#2507` motor terminals, four short
   parallel Elegoo Pi-power leads, and owned JST/STEMMA or Elegoo downstream
   wiring are selected. Their electrical and mechanical acceptance remains in
   Stage 2B.
10. ~~Produce the complete power/signal interface matrix.~~ Completed: the
    separate Q10 matrix fixes all baseline power/signal paths, Due pins, logic
    thresholds, current budgets, protection, connectors, and safe states while
    leaving measurement evidence to Stage 2B.
11. ~~Update and re-export the BOM.~~ Completed: Nathan accepted BOM REV 02 as
    sufficient for Stage 2A, and the prior version is archived.

Stage 2B permits a modular drivetrain deck only after one-channel acceptance
and permits final electronics-tier CAD only after the two-channel,
peripheral, and Pi-power gates. Final CAD is frozen only after the integrated
loaded thermal test and hardware-gate review.

## Manufacturer Sources

- [Pololu `#4753` motor specifications, harness, and encoder details](https://www.pololu.com/product/4753)
- [Pololu Dual MC33926 carrier `#1213`](https://www.pololu.com/product/1213)
- [Pololu Dual MC33926 Arduino shield `#2503`](https://www.pololu.com/product/2503)
- [Pololu Dual MC33926 Raspberry Pi expansion `#2756`](https://www.pololu.com/product/2756)
- [Pololu Dual TB9051FTG Arduino shield `#2520`](https://www.pololu.com/product/2520)
- [Pololu Dual VNH5019 Arduino shield `#2507`](https://www.pololu.com/product/2507)
- [Pololu Dual G2 18v18 Arduino shield `#2515`](https://www.pololu.com/product/2515)
- [Pololu G2 18v17 `#2991`](https://www.pololu.com/product/2991)
- [Pololu Motoron M2S18v18 `#5036`](https://www.pololu.com/product/5036)
- [Pololu Motoron controller user's guide](https://www.pololu.com/docs/0J84/all)
- [Official Arduino Due specifications](https://store.arduino.cc/products/arduino-due)
- [Adafruit `74LVC245` product `#735`](https://www.adafruit.com/product/735)
- [Adafruit 20-pin IC socket product `#2204`](https://www.adafruit.com/product/2204)
- [TI `SN74LVC245A` datasheet](https://www.ti.com/lit/ds/symlink/sn74lvc245a.pdf)
- [Arduino Due core pin definitions](https://github.com/arduino/ArduinoCore-sam/blob/master/variants/arduino_due_x/variant.cpp)
- [Microchip SAM3/4 quadrature-decoder application note](https://www.microchip.com/en-us/application-notes/an42706)
- [Raspberry Pi 5 power and USB-current documentation](https://www.raspberrypi.com/documentation/hardware/raspberrypi/power/raspberry-pi-5.html)
- [Raspberry Pi USB Power Delivery whitepaper](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf)
- [Adafruit DRV8871 breakout](https://www.adafruit.com/product/3190)
- [Cytron MDD10A](https://www.cytron.io/p-10amp-5v-30v-dc-motor-driver-2-channels)
- [Pololu `#5530` five-pin single-ended cable specifications](https://www.pololu.com/product/5530/specs)
- [Adafruit `#954` USB-to-TTL cable](https://www.adafruit.com/product/954)
- [Adafruit `#2998` USB Mini Hub with Power Switch](https://www.adafruit.com/product/2998)
- [ROS 2 BNO055 UART wiring and parameters](https://github.com/flynneva/bno055)
- [Raspberry Pi camera connector documentation](https://www.raspberrypi.com/documentation/accessories/camera.html)
- [SLAMTEC RPLidar A1 specifications](https://www.slamtec.com/en/lidar/a1spec)
- [Pololu `#5534` five-pin female-female cable specifications](https://www.pololu.com/product/5534/specs)
- [Adafruit 100 mm STEMMA QT cable `#4210`](https://www.adafruit.com/product/4210)
- [Adafruit 200 mm STEMMA QT cable `#4401`](https://www.adafruit.com/product/4401)
