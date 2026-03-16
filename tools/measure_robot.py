#!/usr/bin/env python3
"""Interactive measurement guide for ROScar V1.

Walks you through measuring every dimension needed for the chassis_designer.py
calculator. Includes tips on how to measure each value, calculates derived
quantities, and saves results to a JSON file.

Usage:
    python3 tools/measure_robot.py
    python3 tools/measure_robot.py --output my_measurements.json
"""

import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path


def ask(prompt: str, unit: str = "mm", default: float = None) -> float:
    """Ask for a measurement with unit and optional default."""
    dflt = f" [{default}]" if default is not None else ""
    while True:
        val = input(f"    {prompt} ({unit}){dflt}: ").strip()
        if not val and default is not None:
            return default
        try:
            return float(val)
        except ValueError:
            print("    Please enter a number.")


def ask_yn(prompt: str) -> bool:
    """Ask yes/no question."""
    val = input(f"    {prompt} (y/n): ").strip().lower()
    return val in ('y', 'yes')


def section(title: str):
    """Print a section header."""
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


def tip(text: str):
    """Print a measurement tip."""
    print(f"  TIP: {text}")


def main():
    parser = argparse.ArgumentParser(description="ROScar measurement guide")
    parser.add_argument('--output', type=str, default='measurements.json',
                        help='Output JSON file (default: measurements.json)')
    args = parser.parse_args()

    data = {
        'date': datetime.now().isoformat(),
        'robot': 'ROScar V1',
        'notes': '',
    }

    print("\n" + "=" * 60)
    print("  ROScar Measurement Guide")
    print("  Grab a tape measure, calipers, and a kitchen scale!")
    print("=" * 60)

    # ── Wheels ──
    section("WHEEL MEASUREMENTS")
    tip("Use calipers across the mecanum rollers at the widest point.")
    tip("Measure multiple wheels and average if they differ.")
    print()

    wheel_dia = ask("Wheel outer diameter (across rollers)", "mm")
    wheel_radius = wheel_dia / 2.0
    wheel_width = ask("Wheel width (hub face to face incl rollers)", "mm")

    data['wheel'] = {
        'outer_diameter_mm': wheel_dia,
        'outer_radius_mm': wheel_radius,
        'width_mm': wheel_width,
    }
    print(f"\n    Wheel radius: {wheel_radius:.1f} mm")

    # ── Chassis geometry ──
    section("CHASSIS GEOMETRY")
    tip("Measure axle positions center-to-center, not chassis edges.")
    print()

    wheelbase = ask("Wheelbase: front axle to rear axle center-center", "mm")
    track_width = ask("Track width: left wheel center to right wheel center", "mm")
    ground_clearance = ask("Ground clearance: floor to lowest chassis point", "mm")

    data['geometry'] = {
        'wheelbase_mm': wheelbase,
        'track_width_mm': track_width,
        'ground_clearance_mm': ground_clearance,
    }

    tip("Now measure the overall bounding box.")
    print()
    overall_l = ask("Overall length (front to back, including protrusions)", "mm")
    overall_w = ask("Overall width (including wheels)", "mm")
    overall_h = ask("Overall height (floor to highest point)", "mm")

    data['geometry']['overall_length_mm'] = overall_l
    data['geometry']['overall_width_mm'] = overall_w
    data['geometry']['overall_height_mm'] = overall_h

    # Derived: axle height should roughly equal wheel radius
    axle_height = wheel_radius
    print(f"\n    Derived: axle height ≈ {axle_height:.1f} mm (should match wheel radius)")
    print(f"    Chassis bottom at {ground_clearance:.1f} mm")
    print(f"    Chassis top at ~{ground_clearance + 80:.1f} mm (est 80mm plate)")

    # ── Mass measurements ──
    section("MASS MEASUREMENTS")
    tip("A kitchen scale works great. Weigh the whole robot first.")
    tip("Then weigh individual components if you can remove them safely.")
    print()

    total_mass = ask("Total robot mass", "grams")
    data['mass'] = {'total_g': total_mass}

    print("\n    Individual components (enter 0 to skip):")
    components = [
        ("Single motor+wheel assembly", "motor_wheel_g"),
        ("Battery", "battery_g"),
        ("Raspberry Pi 5 + cooler", "pi5_g"),
        ("YB-ERF01-V3.0 board", "stm32_board_g"),
        ("RPLIDAR C1", "rplidar_g"),
        ("Webcam", "webcam_g"),
    ]

    for name, key in components:
        val = ask(name, "grams", default=0)
        data['mass'][key] = val

    measured_sum = sum(v for k, v in data['mass'].items() if k != 'total_g' and v > 0)
    if measured_sum > 0:
        motor_total = data['mass'].get('motor_wheel_g', 0) * 4
        parts_sum = measured_sum - data['mass'].get('motor_wheel_g', 0) + motor_total
        remainder = total_mass - parts_sum
        print(f"\n    Sum of measured parts: {parts_sum:.0f}g (×4 motors)")
        print(f"    Unaccounted (chassis + cables + misc): {remainder:.0f}g")
        data['mass']['unaccounted_g'] = remainder

    # ── Center of gravity ──
    section("CENTER OF GRAVITY")
    print("""
    Method 1 — Balance test (easiest):
    Place a round dowel across the robot's width.
    Slide it front-to-back until the robot balances.
    Measure from the front axle to the balance point.

    Method 2 — Tilt test (for height):
    Place robot on its rear wheels against a wall.
    Slowly tilt forward until it's about to tip.
    Measure the tilt angle.
    """)

    cog_from_front_axle = ask("CoG distance from front axle (+ = toward rear)", "mm", default=0)
    cog_x_from_center = wheelbase / 2.0 - cog_from_front_axle  # + = forward of center
    data['cog'] = {
        'x_from_front_axle_mm': cog_from_front_axle,
        'x_from_center_mm': cog_x_from_center,
    }
    print(f"    CoG is {abs(cog_x_from_center):.1f} mm {'forward of' if cog_x_from_center > 0 else 'behind'} geometric center")

    if ask_yn("Did you do the tilt test for CoG height?"):
        tilt_angle = ask("Tilt angle when robot just tips (degrees from horizontal)", "degrees")
        cog_height = (wheelbase / 2.0) / math.tan(math.radians(tilt_angle))
        print(f"    Calculated CoG height: {cog_height:.1f} mm above ground")
        data['cog']['height_mm'] = cog_height
        data['cog']['tilt_angle_deg'] = tilt_angle
    else:
        cog_height = ask("Estimated CoG height above ground", "mm", default=100)
        data['cog']['height_mm'] = cog_height
        data['cog']['estimated'] = True

    # ── Firmware calibration ──
    section("FIRMWARE CALIBRATION (optional)")
    print("""
    These tests determine what the STM32 firmware assumes for wheel geometry.
    You need the robot powered on and connected.

    Test 1: Drive exactly 2 meters forward. Compare tape measure to odom.
    Test 2: Spin 10 full rotations in place. Compare gyro vs odom count.
    """)

    if ask_yn("Did you run the straight-line calibration test?"):
        actual_dist = ask("Actual distance driven (tape measure)", "mm")
        odom_dist = ask("Distance reported by /odom_raw", "mm")
        radius_correction = actual_dist / odom_dist if odom_dist > 0 else 1.0
        print(f"    Wheel radius correction factor: {radius_correction:.4f}")
        print(f"    Effective wheel radius: {wheel_radius * radius_correction:.2f} mm")
        data['calibration'] = {
            'actual_distance_mm': actual_dist,
            'odom_distance_mm': odom_dist,
            'radius_correction': radius_correction,
            'effective_radius_mm': wheel_radius * radius_correction,
        }
    else:
        data['calibration'] = {'skipped': True}

    # ── Motor performance (optional) ──
    section("MOTOR PERFORMANCE (optional)")

    if ask_yn("Do you want to record motor specs?"):
        battery_v = ask("Battery voltage (idle, fully charged)", "volts", default=12.0)
        data['motor'] = {'battery_voltage_v': battery_v}

        if ask_yn("Did you measure no-load wheel RPM?"):
            noload_rpm = ask("Wheel RPM at full throttle (wheels off ground)", "rpm")
            data['motor']['no_load_rpm'] = noload_rpm
            max_speed = noload_rpm / 60.0 * 2.0 * math.pi * (wheel_radius / 1000.0)
            print(f"    Theoretical max speed: {max_speed:.2f} m/s")
            data['motor']['theoretical_max_speed_ms'] = max_speed

        if ask_yn("Did you measure on-ground max speed?"):
            max_v = ask("Max forward speed on ground (from encoder)", "m/s")
            data['motor']['measured_max_speed_ms'] = max_v
    else:
        data['motor'] = {'skipped': True}

    # ── Summary ──
    section("MEASUREMENT SUMMARY")
    print(f"""
    Wheel:
      Radius:           {wheel_radius:.1f} mm
      Width:            {wheel_width:.1f} mm

    Chassis:
      Wheelbase:        {wheelbase:.1f} mm
      Track width:      {track_width:.1f} mm
      Ground clearance: {ground_clearance:.1f} mm
      Overall:          {overall_l:.0f} × {overall_w:.0f} × {overall_h:.0f} mm

    Mass:
      Total:            {total_mass:.0f} g

    Center of gravity:
      X from center:    {cog_x_from_center:+.1f} mm (+ = forward)
      Height:           {cog_height:.1f} mm

    Stability (quick check):
      Wheelbase/CoG:    {wheelbase/cog_height:.2f}  {'OK' if wheelbase/cog_height >= 1.5 else 'LOW'}
      Track/CoG:        {track_width/cog_height:.2f}  {'OK' if track_width/cog_height >= 1.5 else 'LOW'}
      Max fwd decel:    {9.81 * (wheelbase/1000) / (2 * cog_height/1000):.1f} m/s²
    """)

    # ── Generate chassis_designer input ──
    designer_input = {
        'measurements': {
            'wheelbase_m': wheelbase / 1000.0,
            'track_width_m': track_width / 1000.0,
            'ground_clearance_m': ground_clearance / 1000.0,
            'overall_length_m': overall_l / 1000.0,
            'overall_width_m': overall_w / 1000.0,
            'overall_height_m': overall_h / 1000.0,
            'total_mass_kg': total_mass / 1000.0,
            'cog_x_m': cog_x_from_center / 1000.0,
            'cog_z_m': cog_height / 1000.0,
            'wheel': {
                'outer_radius_m': wheel_radius / 1000.0,
                'width_m': wheel_width / 1000.0,
                'mass_kg': data['mass'].get('motor_wheel_g', 100) / 1000.0,
            },
            'motor': {
                'gear_ratio': 30.0,
                'encoder_cpr': 1320,
                'no_load_rpm': data.get('motor', {}).get('no_load_rpm', 330.0),
                'stall_torque_nm': 0.5,
                'voltage_v': data.get('motor', {}).get('battery_voltage_v', 12.0),
                'motor_count': 4,
            },
            'components': [],  # fill in with per-component z heights if you want detailed CoG
        },
        'constraints': {
            'min_tip_accel_ms2': 5.0,
            'max_speed_ms': 0.8,
            'max_angular_speed_rads': 3.0,
            'target_cog_height_m': 0.070,
            'max_overall_height_m': 0.200,
            'max_overall_length_m': 0.400,
            'max_overall_width_m': 0.350,
        }
    }

    # Save raw measurements
    raw_path = args.output
    with open(raw_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"  Raw measurements saved to {raw_path}")

    # Save chassis_designer compatible file
    designer_path = raw_path.replace('.json', '_designer.json')
    with open(designer_path, 'w') as f:
        json.dump(designer_input, f, indent=2)
    print(f"  chassis_designer.py input saved to {designer_path}")
    print(f"\n  Run:  python3 tools/chassis_designer.py --from-file {designer_path}")
    print()


if __name__ == '__main__':
    main()
