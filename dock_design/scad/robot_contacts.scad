// ============================================================================
// ROScar1 Charging Contact Plate — Mounts on Robot Rear
// ============================================================================
// A small plate with two copper-tape contact pads that mates with the
// pogo pins in the dock's contact wall.
//
// Print this and glue it to the robot's rear face, then apply copper tape
// over the two raised pads.
// ============================================================================

// Must match dock parameters
contact_spacing    = 60;    // mm, center-to-center (same as dock)
contact_height     = 35;    // mm from base plate top (same as dock)

// Robot rear plate dimensions
plate_width        = 100;   // mm
plate_height       = 50;    // mm
plate_thickness    = 3;     // mm

// Contact pad (raised area for copper tape)
pad_width          = 15;    // mm
pad_height         = 20;    // mm
pad_raise          = 1.5;   // mm (proud of plate surface)

// Mounting holes
mount_hole_dia     = 3.2;   // mm (M3)

// Wire routing
wire_hole_dia      = 4;     // mm

module contact_plate() {
    difference() {
        union() {
            // Main plate
            cube([plate_width, plate_thickness, plate_height], center = false);

            // Raised contact pads
            for (offset = [-1, 1]) {
                translate([plate_width/2 + offset * contact_spacing/2 - pad_width/2,
                           plate_thickness,
                           plate_height/2 - pad_height/2])
                    cube([pad_width, pad_raise, pad_height]);
            }
        }

        // Mounting holes (4 corners)
        for (dx = [-1, 1]) {
            for (dz = [-1, 1]) {
                translate([plate_width/2 + dx * (plate_width/2 - 8),
                           -0.1,
                           plate_height/2 + dz * (plate_height/2 - 8)])
                    rotate([-90, 0, 0])
                    cylinder(h = plate_thickness + pad_raise + 0.2, d = mount_hole_dia, $fn = 20);
            }
        }

        // Wire holes behind each contact pad
        for (offset = [-1, 1]) {
            translate([plate_width/2 + offset * contact_spacing/2,
                       -0.1,
                       plate_height/2 - 10])
                rotate([-90, 0, 0])
                cylinder(h = plate_thickness + 0.2, d = wire_hole_dia, $fn = 20);
        }
    }
}

contact_plate();
