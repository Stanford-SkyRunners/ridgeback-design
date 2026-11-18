"""Ridgeback SR-3 (airborne relay): single source of truth for the CAD. Values trace to ridgeback/analysis/D2_RELAY_SIZING.md
(+ aero.py / structures.py runs with the relay geometry). The pod-carrying SR-2 values are kept in design_sr2_pods.py.

Frame: x aft from the wing-root leading edge, y right, z up; wing root lower surface at z = 0
(high wing), fuselage hangs below. Units: mm, g. Any number not from an analysis is tagged [A].
"""
import math

# ---------------------------------------------------------------- top-level (D_SIZING.md, latest closure)
MTOW_G = 13800                     # [C] sizing_relay.py: quad U8 II, 12S5P, relay payload
SPAN = 3350.0                      # [C] S 0.866 m2, AR 13 (stall 13.5 m/s EAS, C_Lmax 1.4)
TAPER = 0.65
WING_AREA_M2 = 0.866
ROOT_CHORD = 2*WING_AREA_M2*1e6/(SPAN*(1 + TAPER))     # 313 mm (was mis-commented 396, an SR-2 value)
TIP_CHORD = ROOT_CHORD*TAPER                           # 204 mm
LE_SWEEP_FRAC = 0.03               # LE moves aft by 3 % of (root - tip) per semispan [aero.py]
AIRFOIL = "sd7037"                 # 9.2 % thick [structures.py]
DIHEDRAL_DEG = 3.0                 # outer panels only; the centre section (|y| <= BOOM_Y) is flat so the booms clamp square
CENTER_HALF = 600.0                # = BOOM_Y: centre-section / outer-panel joint
WING_INCIDENCE_DEG = 2.0
WASHOUT_DEG = 1.5
FLAP_SPAN = (150.0, 900.0)         # y range [A]
AILERON_SPAN = (950.0, 1580.0)     # [A] semispan is 1675
CONTROL_CHORD_FRAC = 0.25

# ---------------------------------------------------------------- structure (E_STRUCTURES.md)
SPAR_X_FRAC = 0.28                 # at the thickest section
INNER_SPAR = (24.0, 2.0)           # CF tube OD, wall; margin 1.45, root allows <= 24.5 mm [structures.py relay run]
OUTER_SPAR = (18.0, 2.5)          # was 18x1.5: margin 1.01 at y=430 at 3.8 g; 18x2.5 gives 1.42 (docs/SPAR_CHECK_2026-10-18.md). OD unchanged
BOOM_TUBE = (30.0, 1.5)            # 1st bending 28 Hz, 1P 60 Hz
REAR_ROD = 6.0                     # [A] torsion/alignment rod at 65 % chord

# ---------------------------------------------------------------- CG and lift system (aero.py, D_SIZING.md)
CG_X = 131.0                       # [C] aero.py relay run, 12 % MAC static margin
BOOM_Y = 600.0                     # [C] rear lift discs clear the 16" pusher disc by ~40 mm in plan view
BOOM_Z = -25.0                     # boom centre just under the wing lower surface
FRONT_MOTOR_X = -458.0             # [C] motor centroid on the CG (aero.py relay run)
REAR_MOTOR_X = 720.0               # rear disc clears the root TE (313) by ~51 mm
LIFT_PROP_D = 711.0                # 28 in (T-Motor G28x9.2) [S63]
COAX_TOP_PROP_Z = 70.0             # [A] prop plane (single motor per position on the relay airframe)
COAX_BOT_PROP_Z = None             # no lower props on SR-3 (quad)
LIFT_MOTORS_PER_POS = 1
LIFT_MOTOR = dict(d=88.0, h=36.0, g=272)    # T-Motor U8 II envelope [A dims], 272 g [S63]
BOOM_FRONT_X = FRONT_MOTOR_X - 90.0
TAIL_LE_X = 1150.0                 # rear disc aft edge at 1076
BOOM_REAR_X = TAIL_LE_X + 210.0

# ---------------------------------------------------------------- arch tail (aero.py sweep)
ARCH_CHORD = 160.0                 # [C] aero.py relay run: V_h 0.63, V_v 0.041
ARCH_APEX_H = 550.0                # above the boom axis
ARCH_INCIDENCE_DEG = 0.0           # [C] trim at 24 m/s EAS, 3,100 m (relay run)
ARCH_AIRFOIL = "naca0010"
RUDDERVATOR_CHORD_FRAC = 0.30

# ---------------------------------------------------------------- fuselage + pusher
FUSE_X0, FUSE_X1 = -550.0, 800.0
FUSE_W, FUSE_H = 210.0, 210.0      # no bay: battery 225x135x75, duplexer 175x114x35, avionics
FUSE_N = 2.4                       # superellipse exponent at the nose and tail
FUSE_N_MID = 3.0                   # no bay any more: a rounder, lower-drag section
FUSE_MID_X = (-250.0, 350.0)
PUSHER_X = FUSE_X1                 # firewall face
PUSHER_Z = -95.0                   # thrust line
PUSH_PROP_D = 406.0                # 16" folding [A]
PUSH_MOTOR = dict(d=50.0, h=55.0, g=330)    # smaller pusher class for 13.8 kg [A]

# ---------------------------------------------------------------- internal layout (F2_bay_and_pods.md)
BAY_CENTER_X = CG_X
SLOT = dict(x=210.0, y=136.0, z=120.0)       # per-slot envelope (pod 130 wide + 6 mm guides)
SLOT_PITCH = dict(x=220.0, y=142.0)          # bay 278 mm wide vs 288 mm inner fuselage width at the bay floor [C]
BAY_Z_FLOOR = -FUSE_H + 25.0               # 25 mm above the keel, where the mid-body is wide enough
BATTERY = dict(x=225.0, y=135.0, z=75.0, g=4700, cells="12S5P Molicel P45B")   # [C] 60 cells upright, 12 % overhead (same pack)
BATTERY_Z_TOP = -12.0                        # under the wing carry-through
TURRET = dict(x=-400.0, z=-FUSE_H - 40.0, ball_d=110.0)   # Trillium HD25-LV class envelope [A]
NADIR_CAM_X = 300.0
AVIONICS_BAY = (-500.0, -230.0)              # Jetson/ARK carrier, MANET, satcom, receivers
GROUND_Z = -360.0                            # legs under the motors: turret ball ~55 mm, pusher tip ~60 mm above ground

# ---------------------------------------------------------------- RidgePod envelope (F2)
POD = dict(x=200.0, y=130.0, z=110.0, g=1780)
POD_DUPLEXER = dict(x=175.0, y=114.0, z=35.0, g=1000)     # Telewave TMND3 [S126]
POD_WHIP_LEN = 450.0
POD_LEG_LEN = 450.0                          # legs double as quarter-wave radials

def chord_at(y):
    return ROOT_CHORD + (TIP_CHORD - ROOT_CHORD)*abs(y)/(SPAN/2)

def le_x(y):
    return LE_SWEEP_FRAC*(ROOT_CHORD - TIP_CHORD)*abs(y)/(SPAN/2)

def dihedral_z(y):
    return max(0.0, abs(y) - CENTER_HALF)*math.tan(math.radians(DIHEDRAL_DEG))

def fuse_n(x):
    """Section exponent along the fuselage: boxy mid-body, rounder ends."""
    a, b = FUSE_MID_X
    if a <= x <= b:
        return FUSE_N_MID
    d = (a - x) if x < a else (x - b)
    return max(FUSE_N, FUSE_N_MID - (FUSE_N_MID - FUSE_N)*min(1.0, d/250.0))

def section_xz(chord, incidence_deg=WING_INCIDENCE_DEG, x_le=0.0, z_le=None, name=AIRFOIL, n=60):
    """Placed airfoil (x, z) points, TE->upper->LE->lower->TE. z_le=None lifts the lowest point to z=0."""
    import numpy as _np, aerosandbox as _asb
    p = _np.asarray(_asb.Airfoil(name).repanel(n_points_per_side=n).coordinates)*chord
    t = math.radians(incidence_deg)
    x = p[:, 0]*math.cos(t) + p[:, 1]*math.sin(t)
    z = -p[:, 0]*math.sin(t) + p[:, 1]*math.cos(t)
    if z_le is None:
        z_le = -z.min()
    return _np.stack([x + x_le, z + z_le], 1)

# ---------------------------------------------------------------- relay payload (requirements section 9, D2)
RELAY = dict(duplexer=dict(x=175.0, y=114.0, z=35.0, g=1000), radio=dict(x=120.0, y=80.0, z=30.0, g=300),
             x=-150.0)                      # duplexer + repeater radio ahead of the battery, near the CG
BATTERY_X = 131.0                           # starting point; assembly.py solves it

# ---------------------------------------------------------------- SR3_CONFIG=mini: smaller own design (2026-11-02)
# Values from ridgeback/analysis (mini_sizing, mini_layout row tail 0.12 x 0.36, mini_spar, mini_boom) and
# docs/OWN_SMALL_DRONE_2026-10-30.md. Only active when the env var is set; relay and scout builds see the values above.
import os as _os
if _os.environ.get("SR3_CONFIG", "").strip().lower() == "mini":
    MTOW_G = 8870                      # [C] mini_sizing / mini_spar (9.04 kg used for loads)
    SPAN = 2240.0                      # [C] S 0.50 m2, AR 10
    WING_AREA_M2 = 0.50
    ROOT_CHORD = 2*WING_AREA_M2*1e6/(SPAN*(1 + TAPER))     # 270.6 mm
    TIP_CHORD = ROOT_CHORD*TAPER                           # 175.9 mm
    CENTER_HALF = 390.0                # = BOOM_Y; washout runs from here to the tip (mini_layout twist 2.0 -> 0.5)
    TIP_BREAK_Y = 870.0                # [C] mini_polyhedral: 10 deg tip polyhedral outboard of y 870
    TIP_DIHEDRAL_DEG = 10.0
    AILERON_SPAN = (500.0, 865.0)      # [A] mini_servo: outboard of the prop wash to the tip break
    INNER_SPAR = (21.0, 1.0)           # [C] mini_spar: 21/19 centre
    OUTER_SPAR = (17.0, 1.0)           # 17/15 outer
    TIP_SPAR_T = (13.0, 1.0)           # 13/11 tip
    BOOM_TUBE = (30.0, 1.0)            # [C] mini_boom: 30/28 HaoZhong, jointed with a 27/25 sleeve
    REAR_ROD = 6.0                     # 6/4 tube
    CG_X = 94.0                        # [C] mini_layout (15 % MAC static margin)
    BOOM_Y = 390.0
    BOOM_Z = -20.0                     # [A] mini_layout BZ
    FRONT_MOTOR_X = -360.0             # [C] mini_layout
    REAR_MOTOR_X = 549.0               # [C] rear disc 50 mm behind the root TE
    LIFT_PROP_D = 457.0                # 18 in, T-Motor P18x6.1
    LIFT_MOTOR = dict(d=55.6, h=31.0, g=150)   # T-Motor MN5008 envelope (MEASURE; mass NOT FOUND, [A])
    BOOM_FRONT_X = FRONT_MOTOR_X - 30.0     # boom 1358 long (mini_layout), two 1 m HaoZhong pieces per boom
    TAIL_LE_X = 828.0                  # [C] mini_layout
    BOOM_REAR_X = TAIL_LE_X + 140.0
    ARCH_CHORD = 120.0                 # [C] mini_layout pick
    ARCH_APEX_H = 360.0
    ARCH_INCIDENCE_DEG = 0.0           # [C] trim -0.12 deg, rounded to 0
    FUSE_X0, FUSE_X1 = -380.0, 560.0   # [A] 6S6P pack + Sky4 avionics + Jetson; pusher ahead of the arch apex
    FUSE_W, FUSE_H = 170.0, 165.0      # [A] pack 140 x 135 x 75 with walls
    FUSE_MID_X = (-180.0, 280.0)
    PUSHER_X = FUSE_X1
    PUSHER_Z = -80.0
    PUSH_PROP_D = 305.0                # 12 in (APC 12x6E planned, BUY_LIST)
    PUSH_MOTOR = dict(d=42.0, h=52.0, g=221)    # T-Motor AT3520 KV550, 221 g [S reseller]; dims (MEASURE)
    BAY_CENTER_X = CG_X
    BAY_Z_FLOOR = -FUSE_H + 25.0
    BATTERY = dict(x=140.0, y=135.0, z=75.0, g=2720, cells="6S6P Molicel P50B")   # [C] 36 cells upright + 12 % overhead
    TURRET = dict(x=-300.0, z=-FUSE_H - 40.0, ball_d=0.0)    # no turret on the smaller design
    AVIONICS_BAY = (-340.0, -160.0)
    GROUND_Z = -260.0                  # [A] pusher tip (z -80 - 152) about 28 mm above the ground
    BATTERY_X = CG_X

    def dihedral_z(y):
        return max(0.0, abs(y) - TIP_BREAK_Y)*math.tan(math.radians(TIP_DIHEDRAL_DEG))
