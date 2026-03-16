// ============================================================================
// ROScar1 Charging Dock — Parametric OpenSCAD Model
// ============================================================================
// Designed for the 4WD Mecanum robot (YB-ERF01-V3.0 board)
// Print in 3 pieces: base_plate, left_wall, right_wall
// Glue/bolt together after printing
//
// Coordinate system:
//   X = robot approach direction (robot drives in along +X)
//   Y = left-right
//   Z = up
// ============================================================================

// --- Robot dimensions (from URDF, PLACEHOLDER — measure your robot!) -------
robot_length      = 250;   // mm, front to back
robot_width       = 200;   // mm, chassis body
robot_track       = 280;   // mm, outer edge to outer edge including wheels
robot_ground_clr  = 50;    // mm, floor to bottom of chassis
wheel_radius      = 33;    // mm

// --- Dock geometry ---------------------------------------------------------
// Interior bay (where the robot sits)
bay_length        = 300;   // mm, depth robot drives into
bay_width         = 310;   // mm, between inner wall faces (track + clearance)

// Walls
wall_thickness    = 5;     // mm (3-5mm works for PLA/PETG)
wall_height       = 55;    // mm (just above wheel tops, below chassis)

// Funnel (V-guide at entrance)
funnel_flare      = 60;    // mm, extra width per side at mouth
funnel_length     = 120;   // mm, how deep the funnel taper extends

// Base plate
base_thickness    = 4;     // mm
base_extra_front  = 20;    // mm overhang past funnel mouth

// Contact block (rear wall with pogo-pin holes)
contact_wall_thick = 10;   // mm (thicker for strength + pogo pin depth)
contact_height     = 35;   // mm (contact center height from base top)
contact_spacing    = 60;   // mm (center-to-center of the two contacts)
pogo_hole_dia      = 3.2;  // mm (for 3mm pogo pins, slight clearance)

// ArUco marker mount
marker_mount_width  = 80;  // mm
marker_mount_height = 80;  // mm
marker_mount_thick  = 3;   // mm
marker_post_height  = 120; // mm (top of post above base)
marker_post_width   = 10;  // mm

// Cable channel
cable_channel_width = 12;  // mm
cable_channel_depth = 3;   // mm

// Screw holes for assembly
screw_hole_dia     = 3.2;  // mm (M3)
screw_head_dia     = 6.0;  // mm (M3 cap head)

// --- Derived dimensions ----------------------------------------------------
total_length = bay_length + contact_wall_thick + funnel_length + base_extra_front;
total_width  = bay_width + 2 * wall_thickness;
funnel_mouth_width = bay_width + 2 * funnel_flare;

// --- Which part to render --------------------------------------------------
// Set via command line: openscad -D 'part="base"' charging_dock.scad
// Options: "assembled", "base", "left_wall", "right_wall", "contact_wall",
//          "marker_post", "all_printable"
part = "assembled";

// ============================================================================
// MODULES
// ============================================================================

module base_plate() {
    difference() {
        union() {
            // Rectangular bay area
            translate([-base_extra_front - funnel_length, -total_width/2, 0])
                cube([total_length, total_width, base_thickness]);
        }

        // Cable routing channel (center, runs front-to-back)
        translate([0, -cable_channel_width/2, base_thickness - cable_channel_depth])
            cube([bay_length, cable_channel_width, cable_channel_depth + 0.1]);

        // Cable exit hole at rear
        translate([bay_length - 5, -cable_channel_width/2, -0.1])
            cube([contact_wall_thick + 10, cable_channel_width, base_thickness + 0.2]);

        // Screw holes for wall attachment (countersunk from bottom)
        for (side = [-1, 1]) {
            for (x_pos = [30, bay_length/2, bay_length - 30]) {
                translate([x_pos, side * (bay_width/2 + wall_thickness/2), -0.1]) {
                    cylinder(h = base_thickness + 0.2, d = screw_hole_dia, $fn = 20);
                    // Countersink
                    cylinder(h = 2, d = screw_head_dia, $fn = 20);
                }
            }
        }

        // Grip texture on base (parallel grooves for traction)
        for (x = [10 : 15 : bay_length - 10]) {
            translate([x, -bay_width/2 + 10, base_thickness - 1])
                cube([2, bay_width - 20, 1.1]);
        }
    }
}

module side_wall(side = 1) {
    // side: 1 = right (+Y), -1 = left (-Y)
    mirror_y = (side == -1) ? 1 : 0;

    mirror([0, mirror_y, 0]) {
        translate([0, bay_width/2, base_thickness]) {
            difference() {
                union() {
                    // Straight section along the bay
                    cube([bay_length, wall_thickness, wall_height]);

                    // Funnel section (tapers outward toward entrance)
                    hull() {
                        // Inner end (meets bay wall)
                        translate([0, 0, 0])
                            cube([0.1, wall_thickness, wall_height]);
                        // Outer end (flared mouth)
                        translate([-funnel_length, funnel_flare, 0])
                            cube([0.1, wall_thickness, wall_height]);
                    }

                    // Rounded entrance lip (easier for robot to find)
                    translate([-funnel_length, funnel_flare + wall_thickness/2, wall_height/2])
                        rotate([0, -90, 0])
                        cylinder(h = 3, d = wall_thickness, $fn = 20);
                }

                // Screw holes matching base plate
                for (x_pos = [30, bay_length/2, bay_length - 30]) {
                    translate([x_pos, wall_thickness/2, -0.1])
                        cylinder(h = 15, d = screw_hole_dia, $fn = 20);
                }

                // Lightening slots (reduce material, print time)
                for (x = [40 : 60 : bay_length - 60]) {
                    translate([x, -0.1, 15])
                        cube([30, wall_thickness + 0.2, wall_height - 25]);
                }
            }
        }
    }
}

module contact_wall() {
    translate([bay_length, -bay_width/2, base_thickness]) {
        difference() {
            // Solid rear wall
            cube([contact_wall_thick, bay_width, wall_height]);

            // Two pogo pin holes
            for (offset = [-1, 1]) {
                translate([-0.1, bay_width/2 + offset * contact_spacing/2, contact_height]) {
                    rotate([0, 90, 0])
                        cylinder(h = contact_wall_thick + 0.2, d = pogo_hole_dia, $fn = 30);
                }
            }

            // Wire routing holes behind each pogo pin
            for (offset = [-1, 1]) {
                translate([contact_wall_thick - 3, bay_width/2 + offset * contact_spacing/2, contact_height - 8]) {
                    cylinder(h = 6, d = 5, $fn = 20);
                }
            }

            // Label emboss "L" and "R" (or "+" and "-")
            translate([0.5, bay_width/2 - contact_spacing/2, contact_height + 10])
                linear_extrude(1)
                text("-", size = 8, halign = "center", valign = "center");
            translate([0.5, bay_width/2 + contact_spacing/2, contact_height + 10])
                linear_extrude(1)
                text("+", size = 8, halign = "center", valign = "center");
        }
    }
}

module marker_post() {
    // Vertical post + flat plate for ArUco marker
    // Mounts to rear wall center

    post_x = bay_length + contact_wall_thick;

    translate([post_x, -marker_post_width/2, base_thickness]) {
        // Vertical post
        cube([marker_post_width, marker_post_width, marker_post_height]);

        // Angled marker plate (tilted 15° forward for camera visibility)
        translate([marker_post_width/2, marker_post_width/2, marker_post_height])
            rotate([0, 15, 0])
            translate([-marker_mount_thick/2, -marker_mount_width/2, 0])
            difference() {
                cube([marker_mount_thick, marker_mount_width, marker_mount_height]);
                // Mounting holes for zip ties or screws
                for (dy = [-1, 1]) {
                    for (dz = [-1, 1]) {
                        translate([-0.1,
                                   marker_mount_width/2 + dy * (marker_mount_width/2 - 8),
                                   marker_mount_height/2 + dz * (marker_mount_height/2 - 8)])
                            rotate([0, 90, 0])
                            cylinder(h = marker_mount_thick + 0.2, d = 3, $fn = 20);
                    }
                }
            }
    }
}

module robot_ghost() {
    // Transparent robot outline for visualization
    color([0.2, 0.6, 1.0, 0.2]) {
        // Chassis
        translate([robot_length/2 - robot_length + 20, -robot_width/2, robot_ground_clr])
            cube([robot_length, robot_width, 80]);

        // Wheels
        for (fx = [-1, 1]) {
            for (fy = [-1, 1]) {
                translate([fx * 100 + (robot_length/2 - robot_length + 20 + robot_length/2),
                           fy * robot_track/2,
                           wheel_radius])
                    rotate([90, 0, 0])
                    cylinder(h = 40, r = wheel_radius, center = true, $fn = 30);
            }
        }
    }
}

// ============================================================================
// ASSEMBLY / RENDER
// ============================================================================

module assembled() {
    color([0.4, 0.4, 0.4]) base_plate();
    color([0.3, 0.6, 0.3]) side_wall(1);
    color([0.3, 0.6, 0.3]) side_wall(-1);
    color([0.6, 0.3, 0.3]) contact_wall();
    color([0.3, 0.3, 0.6]) marker_post();
    robot_ghost();
}

module all_printable() {
    // Laid out flat for print bed — arrange parts side by side
    base_plate();

    translate([0, total_width + 20, 0])
        rotate([90, 0, 0])
        side_wall(1);

    translate([0, total_width + wall_height + 40, 0])
        rotate([90, 0, 0])
        side_wall(-1);

    translate([0, -total_width/2 - wall_height - 20, -base_thickness])
        rotate([0, -90, 0])
        contact_wall();

    translate([total_length + 30, 0, 0])
        marker_post();
}

// --- Render selected part ---
if (part == "assembled") {
    assembled();
} else if (part == "base") {
    base_plate();
} else if (part == "left_wall") {
    side_wall(-1);
} else if (part == "right_wall") {
    side_wall(1);
} else if (part == "contact_wall") {
    contact_wall();
} else if (part == "marker_post") {
    marker_post();
} else if (part == "all_printable") {
    all_printable();
} else {
    assembled();
}
