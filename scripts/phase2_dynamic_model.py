#!/usr/bin/env python3
"""Preliminary planar dynamics screening for the Phase 2 balancing robot.

This is an engineering screening calculation, not a controller design or a
digital twin.  The companion assumptions and limitations live in
hardware/phase-2-dynamic-model.md.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from math import pi, radians, sqrt


G = 9.80665  # m/s^2
KGF_MM_TO_NM = 0.00980665


@dataclass(frozen=True)
class MotorCurve:
    """Manufacturer data and two fits that bound estimated motor current."""

    voltage_v: float = 12.0
    no_load_speed_rpm: float = 200.0
    catalog_no_load_current_a: float = 0.20
    catalog_stall_current_a: float = 5.5
    catalog_stall_torque_kgf_cm: float = 21.0
    curve_current_intercept_a: float = 0.080
    curve_current_slope_a_per_kgf_mm: float = 0.026
    curve_speed_slope_rpm_per_kgf_mm: float = 1.0

    @property
    def catalog_stall_torque_nm(self) -> float:
        return self.catalog_stall_torque_kgf_cm * 10.0 * KGF_MM_TO_NM

    def current_range_a(self, torque_nm: float) -> tuple[float, float]:
        """Return performance-curve and conservative catalog-endpoint fits."""

        torque_nm = abs(torque_nm)
        torque_kgf_mm = torque_nm / KGF_MM_TO_NM
        curve_fit = (
            self.curve_current_intercept_a
            + self.curve_current_slope_a_per_kgf_mm * torque_kgf_mm
        )
        catalog_fit = self.catalog_no_load_current_a + (
            self.catalog_stall_current_a - self.catalog_no_load_current_a
        ) * torque_nm / self.catalog_stall_torque_nm
        return min(curve_fit, catalog_fit), max(curve_fit, catalog_fit)

    def available_speed_rpm(self, torque_nm: float, terminal_voltage_v: float) -> float:
        """Approximate loaded speed using the published 12 V linear curve.

        No-load speed is scaled with voltage while the published 12 V
        speed-versus-torque slope is retained.  This is deliberately a first
        order estimate; it must be replaced by measured motor data later.
        """

        torque_kgf_mm = abs(torque_nm) / KGF_MM_TO_NM
        no_load_at_voltage = self.no_load_speed_rpm * terminal_voltage_v / self.voltage_v
        return max(
            0.0,
            no_load_at_voltage
            - self.curve_speed_slope_rpm_per_kgf_mm * torque_kgf_mm,
        )


@dataclass(frozen=True)
class RunningGear:
    wheel_radius_m: float = 0.040
    wheel_mass_each_kg: float = 0.7 * 0.028349523125
    hub_mass_each_kg: float = 0.0068
    hub_radius_m: float = 0.0127

    @property
    def translating_mass_kg(self) -> float:
        return 2.0 * (self.wheel_mass_each_kg + self.hub_mass_each_kg)

    @property
    def rotating_inertia_kg_m2(self) -> float:
        # Conservative screening approximations: wheel as a thin hoop and hub
        # as a solid disk.  Fasteners and reflected motor inertia are omitted.
        per_side = (
            self.wheel_mass_each_kg * self.wheel_radius_m**2
            + 0.5 * self.hub_mass_each_kg * self.hub_radius_m**2
        )
        return 2.0 * per_side

    @property
    def equivalent_axle_mass_kg(self) -> float:
        return self.translating_mass_kg + (
            self.rotating_inertia_kg_m2 / self.wheel_radius_m**2
        )


@dataclass(frozen=True)
class Scenario:
    name: str
    body_mass_kg: float
    com_height_m: float
    inertia_ratio: float

    @property
    def body_inertia_kg_m2(self) -> float:
        # I_cm = k * m * l^2.  k is swept because there is no Phase 2 CAD yet.
        return (
            self.inertia_ratio
            * self.body_mass_kg
            * self.com_height_m**2
        )


@dataclass(frozen=True)
class Result:
    scenario: Scenario
    unstable_pole_rad_s: float
    e_fold_ms: float
    torque_per_motor_nm: float
    current_low_a: float
    current_high_a: float
    axle_accel_m_s2: float
    estimated_peak_speed_m_s: float
    demanded_wheel_speed_rpm: float
    available_speed_low_v_rpm: float
    available_speed_nominal_v_rpm: float
    traction_coefficient: float


MOTOR = MotorCurve()
RUNNING_GEAR = RunningGear()
SCENARIOS = (
    Scenario("compact / fast-fall", 1.0, 0.08, 0.25),
    Scenario("nominal", 1.5, 0.14, 1.0 / 3.0),
    Scenario("tall / heavy", 2.0, 0.20, 0.60),
)


def coefficients(scenario: Scenario) -> tuple[float, float, float, float]:
    """Return A, B, C, and determinant for the upright linear model."""

    m = scenario.body_mass_kg
    l = scenario.com_height_m
    a = m + RUNNING_GEAR.equivalent_axle_mass_kg
    b = m * l
    c = scenario.body_inertia_kg_m2 + m * l**2
    determinant = a * c - b**2
    if determinant <= 0.0:
        raise ValueError(f"non-positive mass-matrix determinant for {scenario.name}")
    return a, b, c, determinant


def evaluate(
    scenario: Scenario,
    lean_deg: float,
    recovery_time_s: float,
    low_terminal_voltage_v: float,
) -> Result:
    """Evaluate the initial condition for an ideal symmetric recovery maneuver."""

    if recovery_time_s <= 0.0:
        raise ValueError("recovery time must be positive")
    if low_terminal_voltage_v <= 0.0 or low_terminal_voltage_v > MOTOR.voltage_v:
        raise ValueError("low terminal voltage must be in (0, 12] V")

    m = scenario.body_mass_kg
    l = scenario.com_height_m
    r = RUNNING_GEAR.wheel_radius_m
    theta = radians(lean_deg)
    a, b, c, determinant = coefficients(scenario)

    unstable_pole = sqrt(a * m * G * l / determinant)
    target_pitch_accel = -4.0 * theta / recovery_time_s**2

    # Total actuator output torque T is shared equally by two motors.  Positive
    # T drives the axle forward and applies the opposite reaction torque to the
    # body.  See the companion document for the linear equations.
    total_torque = (
        a * m * G * l * theta - determinant * target_pitch_accel
    ) / (a + b / r)
    torque_per_motor = total_torque / 2.0

    axle_accel = (
        c * (total_torque / r)
        - b * (m * G * l * theta - total_torque)
    ) / determinant

    # For the ideal symmetric, constant-acceleration screening maneuver, the
    # first half determines this approximate maximum speed.
    peak_speed = abs(axle_accel) * recovery_time_s / 2.0
    demanded_rpm = peak_speed / r * 60.0 / (2.0 * pi)
    current_low, current_high = MOTOR.current_range_a(torque_per_motor)

    total_supported_mass = m + RUNNING_GEAR.translating_mass_kg
    normal_force_per_wheel = total_supported_mass * G / 2.0
    traction_coefficient = (
        abs(torque_per_motor) / r / normal_force_per_wheel
    )

    return Result(
        scenario=scenario,
        unstable_pole_rad_s=unstable_pole,
        e_fold_ms=1000.0 / unstable_pole,
        torque_per_motor_nm=torque_per_motor,
        current_low_a=current_low,
        current_high_a=current_high,
        axle_accel_m_s2=axle_accel,
        estimated_peak_speed_m_s=peak_speed,
        demanded_wheel_speed_rpm=demanded_rpm,
        available_speed_low_v_rpm=MOTOR.available_speed_rpm(
            torque_per_motor, low_terminal_voltage_v
        ),
        available_speed_nominal_v_rpm=MOTOR.available_speed_rpm(
            torque_per_motor, MOTOR.voltage_v
        ),
        traction_coefficient=traction_coefficient,
    )


def markdown_table(results: tuple[Result, ...], low_voltage_v: float) -> str:
    rows = [
        "| Scenario | Open-loop pole | E-fold time | Torque / motor | Current estimate | Peak wheel speed | Demand | Available at "
        f"{low_voltage_v:.1f} V | Traction μ |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        rows.append(
            "| {name} | {pole:.1f} rad/s | {efold:.0f} ms | {torque:.3f} N·m | "
            "{low:.2f}–{high:.2f} A | {speed:.2f} m/s | {demand:.0f} rpm | "
            "{available:.0f} rpm | {mu:.2f} |".format(
                name=result.scenario.name,
                pole=result.unstable_pole_rad_s,
                efold=result.e_fold_ms,
                torque=result.torque_per_motor_nm,
                low=result.current_low_a,
                high=result.current_high_a,
                speed=result.estimated_peak_speed_m_s,
                demand=result.demanded_wheel_speed_rpm,
                available=result.available_speed_low_v_rpm,
                mu=result.traction_coefficient,
            )
        )
    return "\n".join(rows)


def self_test() -> None:
    assert abs(MOTOR.catalog_stall_torque_nm - 2.0593965) < 1e-9
    low, high = MOTOR.current_range_a(22.0 * KGF_MM_TO_NM)
    assert low < 0.66 < high  # Published max-efficiency point is bracketed.
    assert abs(MOTOR.available_speed_rpm(22.0 * KGF_MM_TO_NM, 12.0) - 178.0) < 1e-9
    assert 0.08 < RUNNING_GEAR.equivalent_axle_mass_kg < 0.11

    results = tuple(evaluate(item, 10.0, 0.30, 9.6) for item in SCENARIOS)
    assert all(item.unstable_pole_rad_s > 0.0 for item in results)
    assert all(item.current_high_a < 0.50 for item in results)
    assert all(
        item.demanded_wheel_speed_rpm < item.available_speed_low_v_rpm
        for item in results
    )

    # Independently substitute the nominal result back into both rows of the
    # linearized mass-matrix equation.  This catches sign or factor-of-two
    # errors in the motor force/reaction-torque calculation.
    nominal = SCENARIOS[1]
    nominal_result = results[1]
    a, b, c, _ = coefficients(nominal)
    theta = radians(10.0)
    target_pitch_accel = -4.0 * theta / 0.30**2
    total_torque = 2.0 * nominal_result.torque_per_motor_nm
    first_residual = (
        a * nominal_result.axle_accel_m_s2
        + b * target_pitch_accel
        - total_torque / RUNNING_GEAR.wheel_radius_m
    )
    second_residual = (
        b * nominal_result.axle_accel_m_s2
        + c * target_pitch_accel
        - (nominal.body_mass_kg * G * nominal.com_height_m * theta - total_torque)
    )
    assert abs(first_residual) < 1e-12
    assert abs(second_residual) < 1e-12

    aggressive = evaluate(SCENARIOS[-1], 10.0, 0.15, 9.6)
    assert aggressive.demanded_wheel_speed_rpm > aggressive.available_speed_low_v_rpm


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lean-deg", type=float, default=10.0)
    parser.add_argument("--recovery-time", type=float, default=0.30)
    parser.add_argument("--low-voltage", type=float, default=9.6)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        print("phase2_dynamic_model self-test: PASS")
        return

    results = tuple(
        evaluate(
            scenario,
            lean_deg=args.lean_deg,
            recovery_time_s=args.recovery_time,
            low_terminal_voltage_v=args.low_voltage,
        )
        for scenario in SCENARIOS
    )
    print(
        f"Screening case: {args.lean_deg:g}° initial lean, "
        f"{args.recovery_time:g} s ideal recovery"
    )
    print(markdown_table(results, args.low_voltage))


if __name__ == "__main__":
    main()
