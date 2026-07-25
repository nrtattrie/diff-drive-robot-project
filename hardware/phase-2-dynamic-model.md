# Phase 2 Preliminary Dynamic Model

**Status:** Phase 1 Stage 2A Q2 complete; preliminary screening supports the
required reuse of the owned motors and wheels

**Started:** 2026-07-22

This document is the engineering record for the preliminary Phase 2
self-balancing model. It tracks physical properties, evidence, assumptions,
equations, screening calculations, uncertainty, and the narrow Q2 reuse
decision. It is not an assembly guide, a final controller design, or proof that
the physical robot will balance.

The reproducible calculation is
[`scripts/phase2_dynamic_model.py`](../scripts/phase2_dynamic_model.py).
Accepted system decisions belong in `docs/architecture.md` only after the
required review and bench evidence.

## Q2 Decision

Q2 answers two narrow questions:

1. Do the owned motors and 80 mm wheels have a plausible Phase 2 operating
   envelope across reasonable body mass, center-of-mass, and inertia ranges?
2. Given Nathan's requirement not to replace plausible motors or wheels, should
   those owned components be fixed as required Phase 2 reuse?

The answer to both is yes. The owned `#4753` motors and 80 mm wheels remain
plausible in this preliminary screen and are required reuse unless later
physical evidence demonstrates that they are infeasible.

Q2 does not select or size a motor driver, decide whether any other Phase 1
component must transfer to Phase 2, freeze chassis geometry, or establish final
Phase 2 physical properties. Q3 is a separate later task.

Documented balancing platforms such as Pololu's
[Balboa](https://www.pololu.com/docs/0J70/all) use ordinary bidirectional
brushed DC gearmotors with encoder feedback rather than requiring hobby RC
servos. This supports the general motor class already selected, but it does not
prove the owned units' final suitability: gearbox backlash, thermal behavior,
installed mass properties, traction, and closed-loop disturbance recovery
remain Phase 2 validation questions.

The model is deliberately a range study. No Phase 2 chassis exists yet, so the
body properties below are illustrative scenarios rather than requirements or
facts that must be resolved during Phase 1.

## Evidence States

Every property is assigned one of these states:

- **Manufacturer:** published by the component manufacturer.
- **Measured:** physically measured on the owned component or assembly.
- **Assumed:** a scenario value chosen to explore the design space.
- **Derived:** calculated from stated manufacturer, measured, or assumed inputs.
- **Bench-validated:** demonstrated in the intended electrical and mechanical
  interface.

Agreement in a chat does not change a property's evidence state.

## Physical-Property Register

### Running gear and motor

| Property | Symbol | Value or range | Unit | Evidence | State |
|---|---|---:|---|---|---|
| Motor SKU and quantity | - | 2x Pololu `#4753` | - | Owned inventory and product page | Manufacturer; ownership recorded |
| Nominal motor voltage | `V_nom` | 12 | V | Pololu product page | Manufacturer |
| Gear ratio | `N` | 50:1 | - | Pololu product page | Manufacturer |
| No-load output speed at 12 V | `n_0` | 200 | rpm | Pololu product page and performance curve | Manufacturer |
| No-load current at 12 V | `I_0` | 0.08-0.20 | A | Performance-curve fit says 0.080 A; catalog table says 0.20 A | Manufacturer discrepancy; bracketed |
| Stall output torque at 12 V | `tau_stall` | 21 kgf·cm = 2.059 | N·m | Pololu catalog; theoretical extrapolation | Manufacturer; not an operating point |
| Stall current at 12 V | `I_stall` | 5.4-5.5 | A | Performance curve and product page | Manufacturer; theoretical extrapolation |
| Maximum-efficiency point | - | 180 rpm, 2.2 kgf·cm, 0.66 A | - | Pololu product page | Manufacturer |
| Gearmotor mass, each | - | 0.205 | kg | Pololu product specifications | Manufacturer; verify owned units |
| Encoder resolution | - | 3200 counts/output revolution | counts/rev | 64 CPR at motor shaft times 50:1 ratio | Manufacturer/derived |
| Wheel SKU and quantity | - | 2x Pololu `#1430` | - | BOM and product page | Manufacturer; ownership recorded |
| Wheel radius | `r` | 0.040 | m | 80 mm nominal diameter | Manufacturer; loaded radius unmeasured |
| Wheel mass, each | `m_w` | 0.0198 | kg | Pololu specifies 0.7 oz including tire | Manufacturer |
| Hub SKU and quantity | - | 2x Pololu `#1083` | - | BOM and product page | Manufacturer; ownership recorded |
| Hub mass, each | `m_h` | 0.0068 | kg | Pololu product specifications | Manufacturer |
| Wheel rotational inertia | `J_w` | wheel as thin hoop; hub as solid disk | kg·m² | Conservative geometry approximation | Assumed/derived |
| Combined wheel/hub rotational inertia | `J_running` | 6.46e-5 | kg·m² | Geometry approximation above | Derived |
| Equivalent running-gear axle mass | - | 0.0937 | kg | Translating mass plus `J_running/r^2` | Derived |
| Gearbox backlash and friction | - | unknown | - | No owned-unit test | Unresolved |
| Reflected motor/gear inertia | - | omitted | kg·m² | Not published for the owned gearmotor | Unresolved model omission |

The 0.205 kg gearmotor bodies are part of the balancing body, not the rotating
wheel mass. Their location near the axle can reduce their contribution to body
center-of-mass height, but their exact installed position and inertia remain
unmeasured.

### Illustrative body and operating scenarios

| Property | Symbol | Value or range | Unit | Evidence | State |
|---|---|---:|---|---|---|
| Balancing-body mass, excluding wheels/hubs | `m` | 1.0-2.0 | kg | Preliminary design scenarios | Assumed |
| Body center of mass above axle | `l` | 0.08-0.20 | m | Preliminary design scenarios | Assumed |
| Body pitch inertia about its center of mass | `I_b` | `k m l^2`, `k` = 0.25-0.60 | kg·m² | Distributed-body scenario model | Assumed/derived |
| Low motor-terminal voltage | `V_low` | 9.6 | V | Sensitivity case only; battery architecture is unresolved | Assumed |
| Nominal modeled motor-terminal voltage | `V_nom` | 12.0 | V | Motor rating | Manufacturer/model boundary |
| Tire/floor friction coefficient | `mu` | unknown | - | No traction test | Unresolved |

The 9.6 V case is not a battery cutoff decision. It is a deliberately reduced
motor-terminal-voltage case used to expose wheel-speed sensitivity. It does not
set a battery, power-system, or motor-driver requirement.

## Planar Model

The initial model covers straight fore-aft balancing only. Left and right motor
torques are assumed equal, so yaw is omitted.

```text
                    body center of mass
                             o
                             |  l
                             |
                             |  theta
                  -----------+----------- body
                             O  wheel axle  ---> x
                            / \
                           O   O            radius r

             two motor torques -> total output torque T
```

The body is one rigid mass attached to the wheel axle. Unlike a simple force-on-
cart shortcut, this model includes both effects of geared motor output torque:

- `T/r` produces fore-aft tractive force through the rolling wheels;
- `-T` is the equal and opposite reaction torque applied to the body.

That reaction term is important for a real two-wheeled balancing robot.
With positive wheel rotation defined in the forward-rolling direction, the
ideal motor's virtual work is:

```text
delta W = T delta(x/r - theta)
```

The corresponding generalized inputs are therefore `T/r` in translation and
`-T` in body pitch. This is the sign and scale used by the script.

Define:

- `x`: axle displacement in meters;
- `theta`: body pitch from upright in radians, positive in the forward direction;
- `T`: total output torque from both motors in N·m;
- `A = m + m_running + J_running/r^2`;
- `B = m l`;
- `C = I_b + m l^2`.

With ideal rolling and equal motor torques, the nonlinear planar equations used
as the starting point are:

```text
A x_ddot + m l cos(theta) theta_ddot
  - m l sin(theta) theta_dot^2 = T/r

m l cos(theta) x_ddot + C theta_ddot
  - m g l sin(theta) = -T
```

Linearizing about upright (`theta = 0`) gives:

```text
[ A  B ] [x_ddot    ] = [ T/r                 ]
[ B  C ] [theta_ddot]   [ m g l theta - T     ]
```

The mass-matrix determinant is:

```text
D = A C - B^2
```

With zero motor torque, the positive open-loop pole is:

```text
lambda = sqrt(A m g l / D)
```

Its inverse is the open-loop e-fold time. It is a useful indication of how fast
small tilt errors grow, not a complete sampling-rate requirement.

### Screening recovery maneuver

The calculation uses a 10-degree lean from rest and an ideal symmetric angular
acceleration/deceleration maneuver lasting 0.30 seconds. For that screening
maneuver, the initial target acceleration magnitude is:

```text
theta_ddot_target = -4 theta / t_recovery^2
```

The corresponding initial total torque is:

```text
T = (A m g l theta - D theta_ddot_target) / (A + B/r)
```

This is an initial-condition envelope calculation, not a simulated controller.
It does not claim that constant acceleration, perfect state information, or the
ideal trajectory can be achieved by the physical robot.

## Motor Curve Treatment

Pololu's current sources do not agree exactly. The product table reports
0.20 A no-load and 5.5 A theoretical stall, while the performance-curve fit
prints:

```text
current [A] = 0.080 + 0.026 torque [kgf·mm]
```

The script therefore reports a current range bounded by:

1. the published performance-curve fit; and
2. a straight line through the more conservative catalog no-load and stall
   endpoints.

The published 12 V speed fit is:

```text
speed [rpm] = 200 - 1.0 torque [kgf·mm]
```

For the 9.6 V sensitivity case, no-load speed is scaled linearly with voltage
while the published torque slope is retained. Motor heating, winding
temperature, PWM behavior, battery sag, and unit-to-unit variation are not
represented.

## Screening Scenarios and Results

The scenario labels are not chassis proposals:

| Scenario | Body mass | CoM height | Inertia ratio `k` |
|---|---:|---:|---:|
| Compact / fast-fall | 1.0 kg | 0.08 m | 0.25 |
| Nominal | 1.5 kg | 0.14 m | 0.333 |
| Tall / heavy | 2.0 kg | 0.20 m | 0.60 |

Command used:

```bash
python3 scripts/phase2_dynamic_model.py
```

For a 10-degree lean, 0.30-second ideal recovery, and 9.6 V low-voltage
sensitivity case:

| Scenario | Open-loop pole | E-fold time | Torque / motor | Current estimate | Peak wheel speed | Demand | Available at 9.6 V | Traction `mu` |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Compact / fast-fall | 19.1 rad/s | 52 ms | 0.027 N·m | 0.15-0.27 A | 0.27 m/s | 65 rpm | 157 rpm | 0.13 |
| Nominal | 13.4 rad/s | 75 ms | 0.052 N·m | 0.22-0.33 A | 0.40 m/s | 95 rpm | 155 rpm | 0.17 |
| Tall / heavy | 8.7 rad/s | 115 ms | 0.094 N·m | 0.33-0.44 A | 0.56 m/s | 133 rpm | 150 rpm | 0.23 |

The traction column is the ideal minimum tire/floor coefficient implied by the
initial per-wheel torque and static normal-load split. Dynamic load transfer is
omitted.

### Sensitivity finding

For the tall/heavy case, shortening the same ideal recovery to 0.15 seconds
demands approximately 191 rpm while the first-order motor curve predicts only
about 140 rpm available at 9.6 V and 180 rpm at 12 V at the required torque.
That case is speed-infeasible even though its estimated current remains below
0.8 A.

This is why Phase 2 cannot be reduced to a stall-torque margin. Within this
preliminary envelope, wheel-speed headroom, body geometry, traction, and
unmodeled drivetrain behavior can matter before raw motor current.

## Interpretation

### What the model supports

- The owned `#4753` motors and 80 mm wheels are plausible Phase 2 hardware
  across the three illustrative screening cases.
- Nathan requires those motors and wheels to be reused while they remain
  plausible. This screen found no reason to replace them, so no new motors or
  wheels are planned.
- The examined 0.30-second recovery cases require approximately 0.15-0.44 A per
  motor by the two manufacturer-data fits. This is below the published 0.66 A
  maximum-efficiency current and well below the approximately 1.38 A general
  25%-of-stall continuous planning guideline.
- A lower center of mass is not automatically easier to balance. The compact
  scenario has the shortest open-loop e-fold time because its geometry falls
  faster even though it requires less torque.

### What the model does not support

- It does not prove that the motors and wheels will balance the eventual real
  robot; that requires the real Phase 2 design and physical tests.
- It does not validate a specific center-of-mass height or chassis shape.
- It does not capture gearbox backlash, drivetrain compliance, reflected motor
  inertia, motor inductance, rolling resistance, tire deformation, saturation
  recovery, yaw, or impacts.
- Its current and speed results are screening outputs, not requirements for Q3
  or any other Phase 1 hardware decision.

## Q2 Reuse Boundary

Nathan approved this narrow boundary:

| Component or subsystem | Q2 decision | Scope |
|---|---|---|
| Two Pololu `#4753` motors, including their integrated encoders | **Required reuse while plausible** | The preliminary model supports plausibility. Replacement is considered only if later physical Phase 2 evidence demonstrates infeasibility. |
| 80 mm Pololu wheels and 6 mm hubs | **Required reuse while plausible** | The preliminary model supports plausibility. No new wheel size is being planned. |
| Every other Phase 1 component or subsystem | **Not decided by Q2** | Select and validate it for Phase 1 on Phase 1 requirements. Phase 2 reuse may be considered later without constraining current work. |

This decision does not promise that the entire Phase 1 platform will transfer
unchanged. It fixes only the motors and wheels because Nathan explicitly
requires their reuse and the preliminary model found them plausible.

## Intentionally Deferred Phase 2 Properties

Actual Phase 2 mass, center of mass, inertia, chassis geometry, control design,
sensor behavior, drivetrain losses, backlash, traction, thermal behavior, and
disturbance-recovery targets are unknown because Phase 2 has not been designed
or built. They are intentionally deferred to Phase 2 and do not block Q2 or the
remaining Phase 1 work.

## Reproducibility and Checks

Run:

```bash
python3 scripts/phase2_dynamic_model.py --self-test
python3 scripts/phase2_dynamic_model.py
python3 scripts/phase2_dynamic_model.py --recovery-time 0.15
```

The self-test checks manufacturer-point conversions, brackets the published
maximum-efficiency current, verifies a positive mass matrix, confirms all three
baseline cases retain low-voltage speed headroom, and confirms that the
aggressive tall/heavy case exceeds it.

## Primary Sources

- [Pololu `#4753` product specifications](https://www.pololu.com/product/4753)
- [Pololu 37D gearmotor performance curves](https://www.pololu.com/file/0J1736/pololu%E2%80%9037d%E2%80%90metal%E2%80%90gearmotors%E2%80%90rev%E2%80%901%E2%80%902.pdf)
- [Pololu `#1430` 80x10 mm wheel specifications](https://www.pololu.com/product/1430/specs)
- [Pololu `#1083` 6 mm hub specifications](https://www.pololu.com/product/1083/specs)
- [MIT Underactuated Robotics cart-pole dynamics and linearization](https://underactuated.mit.edu/acrobot.html#cart_pole)
- [University of Michigan inverted-pendulum equation derivation](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling)
- [Pathak, Franch, and Agrawal: wheeled-inverted-pendulum dynamics with motor-torque inputs](https://doi.org/10.1109/TRO.2004.840905)

The cart-pole sources support the rigid-body and linearization structure. The
motor reaction torque in this document follows from actuator work between the
wheel and body coordinates and is retained explicitly for the two-wheeled
robot.

## Revision History

| Date | Change | Evidence state |
|---|---|---|
| 2026-07-22 | Created Q2 property register, planar model, sensitivity cases, current envelope, and proposed reuse boundary | Manufacturer data plus assumed scenarios; no physical validation |
| 2026-07-22 | Corrected Q2 scope: motors and wheels are required reuse while plausible; all other Phase 2 properties and reuse decisions are deferred | User-approved planning constraint supported by preliminary screening |
