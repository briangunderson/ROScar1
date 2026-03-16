#!/usr/bin/env python3
"""ROScar V2 Parametric Chassis Designer.

Interactive tool that takes robot measurements and design constraints,
then outputs stability analysis, recommended dimensions, extrusion cut
lists, mass budget, and updated URDF values.

Usage:
    python3 tools/chassis_designer.py                 # interactive mode
    python3 tools/chassis_designer.py --from-file measurements.json  # from saved measurements
    python3 tools/chassis_designer.py --example       # run with example values
"""

import argparse
import json
import math
import sys
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional

G = 9.81  # m/s²


# ── Data classes ──────────────────────────────────────────────────────────────

@dataclass
class WheelSpec:
    """Mecanum wheel physical specs."""
    outer_radius_m: float = 0.033   # measured across rollers
    width_m: float = 0.040          # hub face to face incl rollers
    mass_kg: float = 0.100          # single wheel+motor assembly

@dataclass
class MotorSpec:
    """Motor + gearbox specs."""
    gear_ratio: float = 30.0
    encoder_cpr: int = 1320         # counts per output revolution
    no_load_rpm: float = 330.0      # output shaft RPM (motor_rpm / gear_ratio)
    stall_torque_nm: float = 0.5    # at output shaft (estimated)
    voltage_v: float = 12.0
    motor_count: int = 4

@dataclass
class ComponentMass:
    """Mass inventory for CoG calculation."""
    name: str
    mass_kg: float
    x_m: float = 0.0       # longitudinal position (+ = forward of center)
    z_m: float = 0.0       # height above ground

@dataclass
class Measurements:
    """Current robot measurements (fill these in!)."""
    # Chassis geometry
    wheelbase_m: float = 0.200          # front-to-rear axle center-center
    track_width_m: float = 0.240        # left-to-right wheel center-center
    ground_clearance_m: float = 0.050   # floor to lowest chassis point
    overall_length_m: float = 0.250
    overall_width_m: float = 0.280      # including wheels
    overall_height_m: float = 0.180

    # Wheel
    wheel: WheelSpec = field(default_factory=WheelSpec)

    # Motor
    motor: MotorSpec = field(default_factory=MotorSpec)

    # Total mass
    total_mass_kg: float = 2.8

    # Center of gravity (measured or estimated)
    cog_x_m: float = 0.0       # + = forward of geometric center
    cog_z_m: float = 0.12      # height above ground

    # Components (for mass budget)
    components: list = field(default_factory=lambda: [
        ComponentMass("4x motors+wheels", 0.50, 0.0, 0.033),
        ComponentMass("YB-ERF01-V3.0 board", 0.15, 0.0, 0.060),
        ComponentMass("Battery", 0.30, -0.02, 0.055),
        ComponentMass("Raspberry Pi 5 + cooler", 0.12, 0.02, 0.090),
        ComponentMass("RPLIDAR C1", 0.19, 0.0, 0.160),
        ComponentMass("Webcam", 0.10, 0.10, 0.150),
        ComponentMass("Chassis/plates/fasteners", 0.50, 0.0, 0.060),
        ComponentMass("Cables/misc", 0.20, 0.0, 0.080),
    ])


@dataclass
class DockConstraints:
    """Charging dock integration constraints (from dock_design/HANDOFF.md).

    The dock was designed parametrically — these values map directly to
    SCAD variables in dock_design/scad/charging_dock.scad.
    """
    contact_spacing_m: float = 0.060        # horizontal distance between pogo contacts
    contact_plate_width_m: float = 0.100    # rear contact plate: 100mm wide
    contact_plate_height_m: float = 0.050   # rear contact plate: 50mm tall
    contact_plate_thickness_m: float = 0.003  # 3mm thick
    contact_height_from_floor_m: float = 0.035  # contact center 35mm from floor
    bay_clearance_lateral_m: float = 0.030  # 15mm per side
    bay_clearance_longitudinal_m: float = 0.050  # 50mm front-rear
    dock_base_step_m: float = 0.004         # 4mm base plate step-up
    marker_post_height_m: float = 0.120     # ArUco marker post height
    min_rear_flat_width_m: float = 0.100    # need 100mm flat rear for contacts
    wire_routing_holes: int = 2             # 4mm holes behind each contact pad


@dataclass
class DesignConstraints:
    """Design targets for V2."""
    min_tip_accel_ms2: float = 5.0      # don't tip below this deceleration
    max_speed_ms: float = 0.8           # desired top speed
    max_angular_speed_rads: float = 3.0
    target_cog_height_m: float = 0.070  # target CoG height
    max_overall_height_m: float = 0.200 # total robot height limit
    max_overall_length_m: float = 0.400 # footprint constraint
    max_overall_width_m: float = 0.350
    extrusion_options_mm: list = field(default_factory=lambda: [20, 30, 40])
    dock: DockConstraints = field(default_factory=DockConstraints)


# ── Analysis functions ────────────────────────────────────────────────────────

def tip_angle_deg(half_base_m: float, cog_height_m: float) -> float:
    """Angle the robot must tilt to tip over (static)."""
    return math.degrees(math.atan2(half_base_m, cog_height_m))


def max_decel_no_tip(wheelbase_m: float, cog_height_m: float) -> float:
    """Max deceleration (m/s²) before forward tipping."""
    if cog_height_m <= 0:
        return float('inf')
    return G * wheelbase_m / (2.0 * cog_height_m)


def max_lateral_accel_no_tip(track_width_m: float, cog_height_m: float) -> float:
    """Max lateral accel (m/s²) before sideways tipping."""
    if cog_height_m <= 0:
        return float('inf')
    return G * track_width_m / (2.0 * cog_height_m)


def required_wheelbase(target_decel_ms2: float, cog_height_m: float) -> float:
    """Minimum wheelbase to not tip at given deceleration."""
    return 2.0 * cog_height_m * target_decel_ms2 / G


def compute_cog(components: list) -> tuple:
    """Compute center of gravity from component list."""
    total_mass = sum(c.mass_kg for c in components)
    if total_mass == 0:
        return 0.0, 0.0, 0.0
    cog_x = sum(c.mass_kg * c.x_m for c in components) / total_mass
    cog_z = sum(c.mass_kg * c.z_m for c in components) / total_mass
    return cog_x, cog_z, total_mass


def max_speed_from_motor(motor: MotorSpec, wheel: WheelSpec) -> float:
    """Theoretical max speed from motor no-load RPM."""
    wheel_rps = motor.no_load_rpm / 60.0
    return wheel_rps * 2.0 * math.pi * wheel.outer_radius_m


def max_accel_from_motor(motor: MotorSpec, wheel: WheelSpec, total_mass_kg: float) -> float:
    """Max acceleration from motor stall torque (all 4 wheels)."""
    force_per_wheel = motor.stall_torque_nm / wheel.outer_radius_m
    total_force = force_per_wheel * motor.motor_count
    return total_force / total_mass_kg


def stopping_distance(speed_ms: float, decel_ms2: float) -> float:
    """Distance to stop from given speed at given deceleration."""
    if decel_ms2 <= 0:
        return float('inf')
    return speed_ms ** 2 / (2.0 * decel_ms2)


# ── Report generation ─────────────────────────────────────────────────────────

def analyze_current(m: Measurements) -> dict:
    """Analyze current robot stability."""
    results = {}

    results['max_fwd_decel'] = max_decel_no_tip(m.wheelbase_m, m.cog_z_m)
    results['max_lat_accel'] = max_lateral_accel_no_tip(m.track_width_m, m.cog_z_m)
    results['fwd_tip_angle'] = tip_angle_deg(m.wheelbase_m / 2, m.cog_z_m)
    results['lat_tip_angle'] = tip_angle_deg(m.track_width_m / 2, m.cog_z_m)
    results['max_motor_speed'] = max_speed_from_motor(m.motor, m.wheel)
    results['max_motor_accel'] = max_accel_from_motor(m.motor, m.wheel, m.total_mass_kg)
    results['stop_dist_08'] = stopping_distance(0.8, results['max_fwd_decel'])
    results['stop_dist_05'] = stopping_distance(0.5, results['max_fwd_decel'])

    # Computed CoG from components
    cog_x, cog_z, comp_mass = compute_cog(m.components)
    results['computed_cog_x'] = cog_x
    results['computed_cog_z'] = cog_z
    results['computed_total_mass'] = comp_mass

    # L/h ratio (stability indicator)
    results['L_over_h_fwd'] = m.wheelbase_m / m.cog_z_m if m.cog_z_m > 0 else float('inf')
    results['W_over_h_lat'] = m.track_width_m / m.cog_z_m if m.cog_z_m > 0 else float('inf')

    return results


def recommend_v2(m: Measurements, c: DesignConstraints) -> dict:
    """Compute recommended V2 dimensions."""
    rec = {}

    # Minimum wheelbase for stability target
    min_wb = required_wheelbase(c.min_tip_accel_ms2, c.target_cog_height_m)
    # Round up to nearest 10mm
    rec['min_wheelbase_m'] = math.ceil(min_wb * 100) / 100.0

    # Recommended wheelbase (add 30% margin, never shorter than current)
    rec['rec_wheelbase_m'] = max(
        math.ceil(min_wb * 1.3 * 100) / 100.0,
        m.wheelbase_m  # never regress from current
    )

    # Track width (at least 1.2× wheelbase for mecanum stability)
    rec['rec_track_width_m'] = max(
        rec['rec_wheelbase_m'] * 1.2,
        m.track_width_m  # at least as wide as current
    )
    rec['rec_track_width_m'] = math.ceil(rec['rec_track_width_m'] * 100) / 100.0

    # V2 stability check
    rec['v2_max_fwd_decel'] = max_decel_no_tip(rec['rec_wheelbase_m'], c.target_cog_height_m)
    rec['v2_max_lat_accel'] = max_lateral_accel_no_tip(rec['rec_track_width_m'], c.target_cog_height_m)
    rec['v2_fwd_tip_angle'] = tip_angle_deg(rec['rec_wheelbase_m'] / 2, c.target_cog_height_m)

    # Extrusion cut list
    rec['cuts'] = compute_cut_list(rec, c)

    # URDF values
    rec['urdf'] = {
        'chassis_length': rec['rec_wheelbase_m'] + 0.06,  # chassis extends beyond axles
        'chassis_width': rec['rec_track_width_m'] - 2 * m.wheel.width_m,  # between wheels
        'chassis_height': 0.060,  # 60mm internal height for components
        'chassis_z_offset': m.wheel.outer_radius_m - 0.010,  # slight clearance
        'wheel_radius': m.wheel.outer_radius_m,
        'wheel_width': m.wheel.width_m,
        'wheel_x_offset': rec['rec_wheelbase_m'] / 2,
        'wheel_y_offset': rec['rec_track_width_m'] / 2,
    }

    return rec


def compute_cut_list(rec: dict, c: DesignConstraints) -> list:
    """Generate aluminum extrusion cut list."""
    wb = rec['rec_wheelbase_m']
    tw = rec['rec_track_width_m']

    cuts = []

    # Longitudinal rails (3030) — run full length, extend 30mm past each axle
    rail_len = wb + 0.060
    cuts.append({
        'part': 'Side rail (longitudinal)',
        'profile': '3030',
        'length_mm': round(rail_len * 1000),
        'qty': 2,
        'note': 'Main structural members, mount motors at ends'
    })

    # Cross members (2040) — span between rails
    cross_len = tw - 0.060  # inside of rails
    cuts.append({
        'part': 'Cross member (front/rear)',
        'profile': '2040',
        'length_mm': round(cross_len * 1000),
        'qty': 2,
        'note': 'Front and rear, rigidity + motor mounting'
    })

    # Sensor bar (2020) — lightweight top cross
    cuts.append({
        'part': 'Sensor bar (top)',
        'profile': '2020',
        'length_mm': round(cross_len * 1000),
        'qty': 1,
        'note': 'LIDAR + camera mount, centered'
    })

    # Optional risers if needed
    max_component_height = 0.080  # Pi5 stack
    if max_component_height + 0.033 > c.max_overall_height_m - 0.050:
        cuts.append({
            'part': 'Riser post',
            'profile': '2020',
            'length_mm': 40,
            'qty': 4,
            'note': 'Corner posts for sensor deck (only if needed)'
        })

    return cuts


def print_3d_parts(rec: dict) -> list:
    """Generate 3D printing parts list."""
    return [
        {'part': 'Motor bracket (pair)', 'qty': 4, 'material': 'PETG',
         'note': 'Adapts motor to 3030 extrusion slot, needs precision'},
        {'part': 'Pi5 mounting tray', 'qty': 1, 'material': 'PETG',
         'note': 'Slides into extrusion T-slot, holds Pi5 + cooler'},
        {'part': 'Camera tilt mount', 'qty': 1, 'material': 'PETG',
         'note': 'Adjustable tilt with M3 set screw, mounts to 2020'},
        {'part': 'RPLIDAR mount plate', 'qty': 1, 'material': 'PETG',
         'note': 'Vibration-isolated with 4x rubber grommets, mounts to 2020'},
        {'part': 'Battery tray', 'qty': 1, 'material': 'PETG',
         'note': 'Low-mount with velcro strap retention'},
        {'part': 'STM32 board mount', 'qty': 1, 'material': 'PLA',
         'note': 'Standoff tray, M3 screws, low position'},
        {'part': 'Cable clip (T-slot)', 'qty': 10, 'material': 'PLA',
         'note': 'Snap-in cable guides for extrusion T-slot'},
        {'part': 'Side panel (left)', 'qty': 1, 'material': 'PLA',
         'note': 'Cosmetic + cable containment, snap-fit to extrusion'},
        {'part': 'Side panel (right)', 'qty': 1, 'material': 'PLA',
         'note': 'Mirror of left panel'},
        {'part': 'Rear contact plate (dock)', 'qty': 1, 'material': 'PETG',
         'note': '100x50mm, copper tape pads, 4x M3 mounts — from dock_design/scad/robot_contacts.scad'},
    ]


def format_report(m: Measurements, c: DesignConstraints) -> str:
    """Generate full design report."""
    current = analyze_current(m)
    v2 = recommend_v2(m, c)
    parts_3d = print_3d_parts(v2)

    lines = []
    lines.append("=" * 72)
    lines.append("  ROScar V2 — Parametric Chassis Design Report")
    lines.append("=" * 72)

    # ── Current robot analysis ──
    lines.append("\n╔══════════════════════════════════════════════════════════════════════╗")
    lines.append("║  CURRENT ROBOT (V1) STABILITY ANALYSIS                             ║")
    lines.append("╚══════════════════════════════════════════════════════════════════════╝")
    lines.append(f"  Wheelbase:            {m.wheelbase_m*1000:.0f} mm")
    lines.append(f"  Track width:          {m.track_width_m*1000:.0f} mm")
    lines.append(f"  CoG height:           {m.cog_z_m*1000:.0f} mm")
    lines.append(f"  Total mass:           {m.total_mass_kg:.2f} kg")
    lines.append(f"  Wheelbase/CoG ratio:  {current['L_over_h_fwd']:.2f}  {'OK' if current['L_over_h_fwd'] >= 1.5 else 'LOW — want ≥1.5'}")
    lines.append(f"  Track/CoG ratio:      {current['W_over_h_lat']:.2f}  {'OK' if current['W_over_h_lat'] >= 1.5 else 'LOW — want ≥1.5'}")
    lines.append("")
    lines.append("  Tipping thresholds:")
    lines.append(f"    Forward decel:      {current['max_fwd_decel']:.1f} m/s²  ({current['max_fwd_decel']/G:.2f} g)")
    lines.append(f"    Lateral accel:      {current['max_lat_accel']:.1f} m/s²  ({current['max_lat_accel']/G:.2f} g)")
    lines.append(f"    Fwd tip angle:      {current['fwd_tip_angle']:.1f}°")
    lines.append(f"    Lat tip angle:      {current['lat_tip_angle']:.1f}°")

    # Safety assessment
    if current['max_fwd_decel'] < 3.0:
        lines.append(f"  ⚠ DANGER: Tips at only {current['max_fwd_decel']:.1f} m/s² — even 1g braking tips it!")
    elif current['max_fwd_decel'] < 5.0:
        lines.append(f"  ⚠ WARNING: Tips at {current['max_fwd_decel']:.1f} m/s² — marginal stability")
    else:
        lines.append(f"  ✓ Stable up to {current['max_fwd_decel']:.1f} m/s² deceleration")

    lines.append(f"\n  Stopping distance from 0.5 m/s: {current['stop_dist_05']*1000:.0f} mm")
    lines.append(f"  Stopping distance from 0.8 m/s: {current['stop_dist_08']*1000:.0f} mm")
    lines.append(f"  Motor theoretical max speed:     {current['max_motor_speed']:.2f} m/s")
    lines.append(f"  Motor max accel (stall torque):  {current['max_motor_accel']:.1f} m/s²")

    # Component CoG analysis
    lines.append(f"\n  Component-based CoG estimate:")
    lines.append(f"    CoG X (fwd):  {current['computed_cog_x']*1000:+.1f} mm from center")
    lines.append(f"    CoG Z (up):   {current['computed_cog_z']*1000:.1f} mm above ground")
    lines.append(f"    Sum of parts: {current['computed_total_mass']:.2f} kg")

    # ── V2 Recommendations ──
    lines.append("\n╔══════════════════════════════════════════════════════════════════════╗")
    lines.append("║  V2 RECOMMENDED DIMENSIONS                                         ║")
    lines.append("╚══════════════════════════════════════════════════════════════════════╝")
    lines.append(f"  Target CoG height:    {c.target_cog_height_m*1000:.0f} mm")
    lines.append(f"  Target no-tip decel:  {c.min_tip_accel_ms2:.1f} m/s²")
    lines.append(f"  Min wheelbase:        {v2['min_wheelbase_m']*1000:.0f} mm")
    lines.append(f"  Rec wheelbase:        {v2['rec_wheelbase_m']*1000:.0f} mm  (with 30% margin)")
    lines.append(f"  Rec track width:      {v2['rec_track_width_m']*1000:.0f} mm")
    lines.append("")
    lines.append("  V2 stability (predicted):")
    lines.append(f"    Forward decel limit: {v2['v2_max_fwd_decel']:.1f} m/s²  ({v2['v2_max_fwd_decel']/G:.2f} g)")
    lines.append(f"    Lateral accel limit: {v2['v2_max_lat_accel']:.1f} m/s²  ({v2['v2_max_lat_accel']/G:.2f} g)")
    lines.append(f"    Fwd tip angle:       {v2['v2_fwd_tip_angle']:.1f}°")

    improvement = v2['v2_max_fwd_decel'] / current['max_fwd_decel'] if current['max_fwd_decel'] > 0 else 0
    lines.append(f"    Improvement:         {improvement:.1f}× over V1")

    # ── Motor adequacy ──
    lines.append("\n╔══════════════════════════════════════════════════════════════════════╗")
    lines.append("║  MOTOR ADEQUACY CHECK                                              ║")
    lines.append("╚══════════════════════════════════════════════════════════════════════╝")

    # Assume V2 might be heavier (more aluminum)
    v2_mass_est = m.total_mass_kg + 0.5  # extra extrusion weight
    v2_motor_accel = max_accel_from_motor(m.motor, m.wheel, v2_mass_est)
    v2_max_speed = max_speed_from_motor(m.motor, m.wheel)

    lines.append(f"  V2 estimated mass:    {v2_mass_est:.2f} kg  (V1 + ~500g extrusion frame)")
    lines.append(f"  Max accel at stall:   {v2_motor_accel:.1f} m/s²")
    lines.append(f"  Max speed (no-load):  {v2_max_speed:.2f} m/s")

    if v2_motor_accel < c.min_tip_accel_ms2:
        lines.append(f"  ✓ Motors can't even produce {c.min_tip_accel_ms2:.0f} m/s² — tipping won't be motor-limited")
    if v2_max_speed >= c.max_speed_ms:
        lines.append(f"  ✓ Motor speed adequate for {c.max_speed_ms:.1f} m/s target")
    else:
        lines.append(f"  ⚠ Motor max speed {v2_max_speed:.2f} m/s < target {c.max_speed_ms:.1f} m/s")
        lines.append(f"    Consider larger wheels or different gear ratio")

    # ── Extrusion cut list ──
    lines.append("\n╔══════════════════════════════════════════════════════════════════════╗")
    lines.append("║  ALUMINUM EXTRUSION CUT LIST                                       ║")
    lines.append("╚══════════════════════════════════════════════════════════════════════╝")
    for cut in v2['cuts']:
        lines.append(f"  {cut['qty']}×  {cut['profile']}  @ {cut['length_mm']} mm  — {cut['part']}")
        lines.append(f"       {cut['note']}")

    # ── 3D print list ──
    lines.append("\n╔══════════════════════════════════════════════════════════════════════╗")
    lines.append("║  3D PRINT PARTS LIST (Prusa MK4S)                                  ║")
    lines.append("╚══════════════════════════════════════════════════════════════════════╝")
    for part in parts_3d:
        lines.append(f"  {part['qty']}×  {part['part']}  [{part['material']}]")
        lines.append(f"       {part['note']}")

    # ── URDF values ──
    lines.append("\n╔══════════════════════════════════════════════════════════════════════╗")
    lines.append("║  URDF VALUES (paste into roscar.urdf.xacro)                        ║")
    lines.append("╚══════════════════════════════════════════════════════════════════════╝")
    u = v2['urdf']
    lines.append(f'  <xacro:property name="chassis_length" value="{u["chassis_length"]:.3f}"/>')
    lines.append(f'  <xacro:property name="chassis_width" value="{u["chassis_width"]:.3f}"/>')
    lines.append(f'  <xacro:property name="chassis_height" value="{u["chassis_height"]:.3f}"/>')
    lines.append(f'  <xacro:property name="chassis_z_offset" value="{u["chassis_z_offset"]:.3f}"/>')
    lines.append(f'  <xacro:property name="wheel_radius" value="{u["wheel_radius"]:.3f}"/>')
    lines.append(f'  <xacro:property name="wheel_width" value="{u["wheel_width"]:.3f}"/>')
    lines.append(f'  <xacro:property name="wheel_x_offset" value="{u["wheel_x_offset"]:.3f}"/>')
    lines.append(f'  <xacro:property name="wheel_y_offset" value="{u["wheel_y_offset"]:.3f}"/>')

    lines.append("\n  Nav2 footprint (for nav2_params.yaml):")
    half_l = u['chassis_length'] / 2 + 0.01  # 10mm beyond chassis
    half_w = u['wheel_y_offset'] + u['wheel_width'] / 2 + 0.01
    lines.append(f'  robot_footprint: [[{half_l:.3f}, {half_w:.3f}], [{half_l:.3f}, {-half_w:.3f}], '
                 f'[{-half_l:.3f}, {-half_w:.3f}], [{-half_l:.3f}, {half_w:.3f}]]')

    # ── Charging dock compatibility ──
    d = c.dock
    lines.append("\n╔══════════════════════════════════════════════════════════════════════╗")
    lines.append("║  CHARGING DOCK COMPATIBILITY (dock_design/scad/*.scad)              ║")
    lines.append("╚══════════════════════════════════════════════════════════════════════╝")
    lines.append("  Dock parameters to update when chassis dimensions change:")
    lines.append("  (See dock_design/HANDOFF.md for full integration spec)")

    track_outer = u['wheel_y_offset'] * 2 + u['wheel_width']
    dock_bay_w = track_outer + d.bay_clearance_lateral_m
    dock_bay_l = u['chassis_length'] + d.bay_clearance_longitudinal_m
    dock_wall_h = u['chassis_z_offset'] + 0.005  # 5mm above wheel tops
    # Contact height: midway between floor and chassis bottom
    dock_contact_h = u['chassis_z_offset'] / 2

    lines.append(f"\n  robot_track (outer) = {track_outer*1000:.0f} mm")
    lines.append(f"  robot_length        = {u['chassis_length']*1000:.0f} mm")
    lines.append(f"  robot_ground_clr    = {u['chassis_z_offset']*1000:.0f} mm")
    lines.append(f"  bay_width           = {dock_bay_w*1000:.0f} mm  (track + {d.bay_clearance_lateral_m*1000:.0f}mm)")
    lines.append(f"  bay_length          = {dock_bay_l*1000:.0f} mm  (length + {d.bay_clearance_longitudinal_m*1000:.0f}mm)")
    lines.append(f"  wall_height         = {dock_wall_h*1000:.0f} mm  (ground_clr + 5mm)")
    lines.append(f"  contact_height      = {dock_contact_h*1000:.0f} mm  (mid-height under chassis)")

    # Rear contact plate check
    chassis_rear_w = u['chassis_width']
    if chassis_rear_w < d.min_rear_flat_width_m:
        lines.append(f"\n  !! Chassis width {chassis_rear_w*1000:.0f}mm < contact plate {d.min_rear_flat_width_m*1000:.0f}mm")
        lines.append(f"     Need wider chassis rear or extended mounting bracket")
    else:
        lines.append(f"\n  Rear contact plate ({d.contact_plate_width_m*1000:.0f}mm wide) fits on "
                     f"chassis rear ({chassis_rear_w*1000:.0f}mm wide)")

    # Wheel step-up check
    if m.wheel.outer_radius_m < d.dock_base_step_m * 3:
        lines.append(f"  !! Wheel radius {m.wheel.outer_radius_m*1000:.0f}mm very small — "
                     f"add ramp to dock base plate ({d.dock_base_step_m*1000:.0f}mm step)")
    else:
        lines.append(f"  Wheel radius {m.wheel.outer_radius_m*1000:.0f}mm >> "
                     f"dock step {d.dock_base_step_m*1000:.0f}mm — no ramp needed")

    # Camera line of sight
    lines.append(f"\n  ArUco marker top at ~{d.marker_post_height_m*1000 + 80:.0f}mm from floor")
    lines.append(f"  Camera must see forward at this height during docking approach")

    lines.append("\n" + "=" * 72)
    return "\n".join(lines)


# ── Interactive input ─────────────────────────────────────────────────────────

def prompt_float(prompt: str, default: float) -> float:
    """Prompt for a float value with default."""
    val = input(f"  {prompt} [{default}]: ").strip()
    if not val:
        return default
    try:
        return float(val)
    except ValueError:
        print(f"    Invalid number, using default {default}")
        return default


def interactive_input() -> tuple:
    """Gather measurements interactively."""
    print("\n" + "=" * 60)
    print("  ROScar V2 Chassis Designer — Measurement Input")
    print("=" * 60)
    print("  Enter your measurements (press Enter to keep defaults)")
    print("  All dimensions in millimeters, masses in grams\n")

    m = Measurements()

    print("  ── Chassis Geometry ──")
    m.wheelbase_m = prompt_float("Wheelbase (axle-to-axle) [mm]", m.wheelbase_m * 1000) / 1000
    m.track_width_m = prompt_float("Track width (wheel-to-wheel) [mm]", m.track_width_m * 1000) / 1000
    m.ground_clearance_m = prompt_float("Ground clearance [mm]", m.ground_clearance_m * 1000) / 1000
    m.overall_length_m = prompt_float("Overall length [mm]", m.overall_length_m * 1000) / 1000
    m.overall_width_m = prompt_float("Overall width [mm]", m.overall_width_m * 1000) / 1000
    m.overall_height_m = prompt_float("Overall height [mm]", m.overall_height_m * 1000) / 1000

    print("\n  ── Wheels ──")
    m.wheel.outer_radius_m = prompt_float("Wheel outer radius [mm]", m.wheel.outer_radius_m * 1000) / 1000
    m.wheel.width_m = prompt_float("Wheel width [mm]", m.wheel.width_m * 1000) / 1000

    print("\n  ── Mass ──")
    m.total_mass_kg = prompt_float("Total robot mass [grams]", m.total_mass_kg * 1000) / 1000
    m.cog_z_m = prompt_float("CoG height above ground [mm] (estimate OK)", m.cog_z_m * 1000) / 1000

    print("\n  ── Motor (press Enter for defaults if unknown) ──")
    m.motor.no_load_rpm = prompt_float("Motor output RPM (no load)", m.motor.no_load_rpm)
    m.motor.stall_torque_nm = prompt_float("Motor stall torque [N·m] (estimate OK)", m.motor.stall_torque_nm)

    # Design constraints
    print("\n  ── Design Targets (V2) ──")
    c = DesignConstraints()
    c.min_tip_accel_ms2 = prompt_float("Min no-tip decel [m/s²]", c.min_tip_accel_ms2)
    c.target_cog_height_m = prompt_float("Target CoG height [mm]", c.target_cog_height_m * 1000) / 1000
    c.max_speed_ms = prompt_float("Target max speed [m/s]", c.max_speed_ms)

    return m, c


# ── Serialization ─────────────────────────────────────────────────────────────

def save_measurements(m: Measurements, c: DesignConstraints, path: str):
    """Save measurements to JSON for re-use."""
    data = {
        'measurements': asdict(m),
        'constraints': asdict(c)
    }
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"\n  Saved to {path}")


def load_measurements(path: str) -> tuple:
    """Load measurements from JSON."""
    with open(path) as f:
        data = json.load(f)

    m_data = data['measurements']
    m = Measurements(
        wheelbase_m=m_data['wheelbase_m'],
        track_width_m=m_data['track_width_m'],
        ground_clearance_m=m_data['ground_clearance_m'],
        overall_length_m=m_data['overall_length_m'],
        overall_width_m=m_data['overall_width_m'],
        overall_height_m=m_data['overall_height_m'],
        total_mass_kg=m_data['total_mass_kg'],
        cog_x_m=m_data['cog_x_m'],
        cog_z_m=m_data['cog_z_m'],
    )
    m.wheel = WheelSpec(**m_data['wheel'])
    m.motor = MotorSpec(**m_data['motor'])
    m.components = [ComponentMass(**c) for c in m_data['components']]

    c_data = data['constraints']
    c = DesignConstraints(
        min_tip_accel_ms2=c_data['min_tip_accel_ms2'],
        max_speed_ms=c_data['max_speed_ms'],
        max_angular_speed_rads=c_data['max_angular_speed_rads'],
        target_cog_height_m=c_data['target_cog_height_m'],
        max_overall_height_m=c_data['max_overall_height_m'],
        max_overall_length_m=c_data['max_overall_length_m'],
        max_overall_width_m=c_data['max_overall_width_m'],
    )

    return m, c


# ── Sensitivity analysis ─────────────────────────────────────────────────────

def sensitivity_table(m: Measurements) -> str:
    """Show how stability changes with CoG height and wheelbase."""
    lines = []
    lines.append("\n╔══════════════════════════════════════════════════════════════════════╗")
    lines.append("║  SENSITIVITY: Max Fwd Decel (m/s²) vs Wheelbase × CoG Height       ║")
    lines.append("╚══════════════════════════════════════════════════════════════════════╝")

    wheelbases = [0.15, 0.20, 0.25, 0.30, 0.35]
    cog_heights = [0.06, 0.08, 0.10, 0.12, 0.15]

    header = "  CoG\\WB   " + "".join(f" {wb*1000:4.0f}mm " for wb in wheelbases)
    lines.append(header)
    lines.append("  " + "─" * (len(header) - 2))

    for h in cog_heights:
        row = f"  {h*1000:4.0f}mm   "
        for wb in wheelbases:
            a = max_decel_no_tip(wb, h)
            marker = " *" if abs(wb - m.wheelbase_m) < 0.01 and abs(h - m.cog_z_m) < 0.01 else "  "
            row += f" {a:5.1f}{marker}"
        lines.append(row)

    lines.append("\n  * = current robot (approximate)")
    lines.append("  Values > 5.0 m/s²: good stability")
    lines.append("  Values < 3.0 m/s²: tipping risk at moderate braking")

    return "\n".join(lines)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ROScar V2 Parametric Chassis Designer")
    parser.add_argument('--from-file', type=str, help='Load measurements from JSON file')
    parser.add_argument('--save', type=str, help='Save measurements to JSON file')
    parser.add_argument('--example', action='store_true', help='Run with example/placeholder values')
    args = parser.parse_args()

    if args.from_file:
        m, c = load_measurements(args.from_file)
        print(f"\n  Loaded measurements from {args.from_file}")
    elif args.example:
        m = Measurements()  # defaults = current placeholders
        c = DesignConstraints()
        print("\n  Using example/placeholder values (measure your robot for real results!)")
    else:
        m, c = interactive_input()

    # Generate and print report
    report = format_report(m, c)
    print(report)

    # Sensitivity table
    print(sensitivity_table(m))

    # Save if requested
    if args.save:
        save_measurements(m, c, args.save)
    elif not args.from_file and not args.example:
        save_path = input("\n  Save measurements to file? [measurements.json / Enter to skip]: ").strip()
        if save_path:
            save_measurements(m, c, save_path)

    print()


if __name__ == '__main__':
    main()
