// =============================================================
//  CYD Desk Stand
//  Single-piece tilted enclosure for ESP32-2432S028R
//  ("Cheap Yellow Display") 2.8" smart display.
//
//  Features:
//    - Open-back case; PCB held by 4 corner standoffs with
//      M3 self-tapping screws driven from behind.
//    - Symmetric USB clearance on both short-edge sides of the
//      PCB so a right-angle USB-C plug fits inside the cavity.
//      (Rotate the plug 180 deg to choose which way the cable
//      exits; the open back accepts cable in either direction.)
//    - Optional flat mount on the rear of the foot for an
//      external SCD41 module, sitting in fresh ambient air
//      behind the unit (away from the warm CYD board).
//    - Landscape or portrait via the `portrait` parameter.
//    - Foot depth auto-scales to fully cover the tilted case.
//
//  Hardware:
//    - 4 x M3 x 6 mm or 8 mm self-tapping pan/button head
//      (display mount; pilot 2.5 mm)
//    - 2 x M2.5 x 4 mm or 6 mm self-tapping pan/button head
//      (sensor mount; pilot 2.0 mm)
//
//  Print on the foot's flat bottom. The 22 deg overhang is mild;
//  a tiny brim helps in portrait.
//
//  Coordinate frame:
//    +X = right
//    +Y = forward (out of display, toward viewer)
//    +Z = up
//  Origin at the back-bottom corner of the case.
// =============================================================

$fn = 64;
EPS = 0.05;

/* [Orientation] */
portrait = false;       // false = landscape (long edge horizontal)
                        // true  = portrait  (long edge vertical)

/* [PCB] */
pcb_long  = 86.0;       // CYD PCB long edge
pcb_short = 50.0;       // CYD PCB short edge
pcb_t     = 1.6;
pcb_slop  = 0.4;        // gap on each side for fit

/* [Component clearance] */
back_depth    = 12;     // behind PCB (ESP32 module + JST headers)
front_depth   = 5;      // between PCB front and bezel back
                        // (5 mm gives standoffs enough meat for M3)

usb_clearance = 15;     // extra space beyond EACH short edge of the
                        // PCB (i.e. along the PCB's long axis) to
                        // accommodate a right-angle USB-C plug body
                        // and the bend of the cable. Applied on
                        // both ends so the PCB is centred and the
                        // SD-card slot has clearance too.

/* [Display window] */
disp_along_long  = 58;  // visible LCD area along PCB long edge
disp_along_short = 44;  // visible LCD area along PCB short edge
disp_win_off_x = 0;     // try -1.5 in landscape if your LCD sits offset
disp_win_off_z = 0;

/* [Case shell] */
wall           = 2.6;
bezel          = 4;
shell_corner_r = 3;

/* [Stand / foot] */
tilt_deg        = 22;   // display lean-back angle from vertical
foot_thick      = 4.0;  // foot thickness (4 mm gives M2.5 sensor-mount
                        // screws ~3 mm of thread engagement)
foot_back_extra = 10;   // depth ADDED beyond what is needed to cover
                        // the tilted case's back-top overhang
                        // (also provides flat area for the sensor mount)
foot_corner_r   = 3;    // MUST match shell_corner_r so the rounded
                        // vertical edges of the foot blend smoothly
                        // into the case at the tilt bend

/* [Mounting - display] */
mounting_holes    = true;
hole_inset        = 4.0;    // hole-center distance from each PCB edge
                            // (DIYmall datasheet: 4.0 mm)
standoff_od       = 5.0;    // outer diameter of plastic standoff post
standoff_pilot_id = 2.5;    // pilot for M3 self-tapping
                            // (use 3.2 + heat-set inserts if preferred)
pilot_into_bezel  = 2.0;    // pilot continues this far into bezel
                            // (screw-tip clearance, blind hole)

/* [Mounting - sensor] */
sensor_mount       = true;  // add SCD41 module mount on rear of foot
sensor_size_y      = 18;    // module depth along stand Y axis
sensor_hole_pitch  = 22;    // distance between mount-hole centres
                            // (default suits common SCD41 breakouts;
                            // measure your module and adjust)
sensor_hole_id     = 2.0;   // pilot for M2.5 self-tapping
                            // (use 1.6 for M2)
sensor_hole_depth  = 3.0;   // hole depth into foot top (foot is 4 mm
                            // thick by default, so blind hole is fine)
sensor_rear_margin = 4;     // gap between rear edge of foot and sensor

// ------------------- Derived -------------------
// Map PCB axes to case axes based on orientation.
case_pcb_x = portrait ? pcb_short : pcb_long;
case_pcb_z = portrait ? pcb_long  : pcb_short;

disp_win_w = portrait ? disp_along_short : disp_along_long;
disp_win_h = portrait ? disp_along_long  : disp_along_short;

// USB clearance is added to whichever case axis carries the PCB long
// edge (because the short edges of the PCB are at the ends of the
// long axis): X in landscape, Z in portrait.
usb_along_x = portrait ? 0 : 2 * usb_clearance;
usb_along_z = portrait ? 2 * usb_clearance : 0;

inner_w = case_pcb_x + 2 * pcb_slop + usb_along_x;
inner_h = case_pcb_z + 2 * pcb_slop + usb_along_z;
inner_d = back_depth + pcb_t + front_depth;

outer_w = inner_w + 2 * wall;
outer_h = inner_h + 2 * wall;
outer_d = inner_d + bezel;

// Auto-compute foot_back to fully cover the tilted case overhang
// plus the user-tunable extra (which provides the sensor-mount area).
foot_back_overhang = outer_h * sin(tilt_deg);
foot_back          = foot_back_overhang + foot_back_extra;

foot_fwd   = outer_d * cos(tilt_deg);
foot_max_h = foot_thick + outer_d * sin(tilt_deg);

// Mounting hole positions track the PCB, NOT the cavity.  With
// usb_clearance > 0 the cavity is wider than the PCB; the PCB sits
// centred and is held by the four corner standoffs.
hole_pitch_x = case_pcb_x - 2 * hole_inset;
hole_pitch_z = case_pcb_z - 2 * hole_inset;

pcb_front_y  = back_depth + pcb_t;
pcb_center_z = wall + inner_h / 2;

// Sensor mount Y position (centred over the rear-foot flat area).
sensor_y = -foot_back + sensor_rear_margin + sensor_size_y / 2;

// =============================================================
//  Helpers
// =============================================================
module rounded_box(w, d, h, r) {
    // Box centred on X, extending Y in [0, d] and Z in [0, h].
    // Vertical edges (along Z) are rounded with radius r.
    hull()
        for (x = [-w/2 + r, w/2 - r])
            for (y = [r, d - r])
                translate([x, y, 0])
                    cylinder(r = r, h = h);
}

module y_cyl(d, h) {
    // Cylinder along +Y axis, of given diameter and length.
    rotate([-90, 0, 0])
        cylinder(d = d, h = h);
}

// =============================================================
//  Display-mount standoffs and pilot holes
// =============================================================
module standoffs() {
    for (sx = [-1, 1]) for (sz = [-1, 1])
        translate([sx * hole_pitch_x / 2,
                   pcb_front_y,
                   pcb_center_z + sz * hole_pitch_z / 2])
            y_cyl(standoff_od, front_depth);
}

module pilot_holes() {
    for (sx = [-1, 1]) for (sz = [-1, 1])
        translate([sx * hole_pitch_x / 2,
                   pcb_front_y - EPS,
                   pcb_center_z + sz * hole_pitch_z / 2])
            y_cyl(standoff_pilot_id,
                  front_depth + pilot_into_bezel + EPS);
}

// =============================================================
//  Sensor-mount pilot holes (drilled into the foot top)
// =============================================================
module sensor_mount_holes() {
    for (sx = [-1, 1])
        translate([sx * sensor_hole_pitch / 2,
                   sensor_y,
                   foot_thick - sensor_hole_depth])
            cylinder(d = sensor_hole_id,
                     h = sensor_hole_depth + EPS);
}

// =============================================================
//  Case body  (built upright; display normal = +Y)
// =============================================================
module case_body() {
    difference() {
        union() {
            // Outer shell with cavity and display window
            difference() {
                rounded_box(outer_w, outer_d, outer_h, shell_corner_r);

                // PCB cavity, open through the back face (Y = 0)
                translate([-inner_w / 2, -EPS, wall])
                    cube([inner_w, inner_d + EPS, inner_h]);

                // Display window through the front bezel
                translate([disp_win_off_x - disp_win_w / 2,
                           outer_d - bezel - EPS,
                           outer_h / 2 + disp_win_off_z - disp_win_h / 2])
                    cube([disp_win_w, bezel + 2 * EPS, disp_win_h]);
            }

            // Add standoffs back into the cavity
            if (mounting_holes) standoffs();
        }

        // Drill the pilot holes through the standoffs (and slightly
        // into the bezel) so the screws have clearance for their tips
        if (mounting_holes) pilot_holes();
    }
}

// =============================================================
//  Foot wedge
// =============================================================
module foot_wedge() {
    difference() {
        translate([0, -foot_back, 0])
            rounded_box(outer_w,
                        foot_back + foot_fwd,
                        foot_max_h + EPS,
                        foot_corner_r);

        // Cut #1: above the tilted case-bottom plane (Y_case > 0)
        translate([0, 0, foot_thick])
            rotate([tilt_deg, 0, 0])
                translate([-outer_w / 2 - 1, 0, -EPS])
                    cube([outer_w + 2,
                          outer_d * 3,
                          foot_max_h * 3]);

        // Cut #2: above z = foot_thick behind the case
        translate([-outer_w / 2 - 1, -foot_back - EPS, foot_thick])
            cube([outer_w + 2,
                  foot_back + EPS,
                  foot_max_h * 2]);

        // Sensor-mount blind holes
        if (sensor_mount) sensor_mount_holes();
    }
}

// =============================================================
//  Final assembly
// =============================================================
module stand() {
    foot_wedge();

    translate([0, 0, foot_thick])
        rotate([tilt_deg, 0, 0])
            case_body();
}

stand();
