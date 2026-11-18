"""SR-3 Scout v1 electronics: catalog + placement. PROVISIONAL (team has not agreed the Scout list).

Single source of truth for the Scout electronics. Used by f_elec.py (mounts), harness.py (wiring) and
assemble.py (purchased envelopes, mass, CG, interference). Aircraft frame of design.py (mm, x aft, y right, z up).

Tags (ridgeback/PLAN.md): [S] value read at the manufacturer page / datasheet given in `src`
(read 2026-10-15 by the research pass; "[S?]" = only a reseller or search snippet, not the maker),
[C] calculated here, [A] assumption. Dimensions not in a datasheet are (ASSUMED) or (MEASURE).
"""
import math
from core import d
import f_fuse as FF
import f_wing as FW
import f_boom as FB
import f_tail as FT
import f_misc as FM

TRAY_TOP, TRAY_BOT = FF.TRAY["ztop"], FF.TRAY["ztop"] - FF.TRAY["t"]          # battery tray -91 / -92.5
ZA = FF.AV_TRAYS["A"]                                                         # avionics tray A top -72
MOUNT_T = 3.0                                                                  # printed adapter plate thickness on the trays
BAT_X = FF.BAT["x"]

# ------------------------------------------------------------------ fixed layout points
FC = dict(x=131.0, plate_top=-127.0)                       # FC on its isolator plate under the battery tray, at the CG
PDB = dict(x0=-245.0 + 1.0, x1=-80.0, hw=48.0, standoff=30.0, t=3.0)        # PDB plate under the tray on the 4 ex-radio holes
PDB["top"] = TRAY_BOT - PDB["standoff"]                    # -122.5
PDB["bot"] = PDB["top"] - PDB["t"]
ALT = FF.ALTUM
DECK = dict(z_bot=FF.AV_TRAYS["B"] - FF.AV_T, t=3.0)        # Altum deck sits on the tray-B lugs (bottom -129)
ALT_TOP = DECK["z_bot"]                                     # camera top face against the deck
ALT_C = (ALT["x"], 0.0, ALT_TOP - ALT["h"]/2)
PUSH_ESC = dict(x=650.0, z=-112.0)
MAST = dict(x=FF.ANT_MAST_X, h=200.0)
DLS = dict(x=FF.DLS_X, ped_h=8.0)
PROBE = dict(x_tip=-760.0, z=-85.0, tube=(8.0, 1.0))        # SHT45 probe: 8x6 CF tube from the nose cap, shield at the tip
GNSS_PAD = FM.GNSS

def nose_top(x):
    return FF.top_z(x)

def gnss_z():
    return FW.z_bounds(FM.GNSS["y"], FM.GNSS["x"])[1] + FM.GNSS["t"]

# ------------------------------------------------------------------ catalog
# key: name, model, dims (x, y, z as installed), mass_g, w_avg, w_peak, supply, sub, centre, src, tag, notes
U = {
    "fc_ark": "https://arkelectron.com/product/arkv6x/ ; https://docs.arkelectron.com/products/flight-controller/ark-pixhawk-autopilot-bus-carrier/hardware.md",
    "ark_gps": "https://docs.arkelectron.com/products/gps/ark-gps/hardware.md",
    "rid": "https://docs.holybro.com/radio/remote-id/overview-and-spec.md",
    "ping": "https://uavionix.com/products/pingrx-pro/",
    "rfd": "https://files.rfdesign.com.au/Files/documents/RFD900x%20DataSheet%20V1.2.pdf",
    "elrs": "https://www.radiomasterrc.com/products/rp4td-expresslrs-2-4ghz-diversity-receiver",
    "pm08d": "https://holybro.com/products/pm08d-digital-power-module-14s-200a",
    "mega": "https://www.mouser.com/datasheet/2/240/Littelfuse_MEGA70V_Datasheet-1532099.pdf",
    "as150": "https://www.jameco.com/z/AS150U-M-Amass-DC-Supply-Male-Plug-AS150U-70A-500V-Copper-Gold-Flash-Straight-Cable-Mount-with-Leads-6AWG-Yellow_2844591.html",
    "alpha": "https://store.tmotor.com/goods.php?id=582",
    "flame": "https://store.tmotor.com/goods.php?id=830 ; dims https://robu.in/product/t-motor-flame-80a-12s-v2-0-esc/",
    "kst": "https://cdn.shopifycdn.net/s/files/1/0570/1766/3541/files/X10_V8.0_Technical_Specifcation_ee6acc4d-79de-4d61-b686-3288dc5d4153.pdf",
    "arcv": "https://www.firehousetechnology.com/store/p115/Firehouse_ARC_V_%28Version_2%29__Drone_Strobe_Spotlight_1100_Lumens_3.5_SM_%2F_FAA_107_Compliant.html",
    "altum": "https://support.micasense.com/hc/en-us/articles/4419868608407 (Altum-PT Integration Guide v02)",
    "sht45": "https://sensirion.com/resource/datasheet/sht4x ; https://www.adafruit.com/product/5665",
    "pizero": "https://pip-assets.raspberrypi.com/categories/584-raspberry-pi-zero-2-w/documents/RP-008358-DS-1-raspberry-pi-zero-2-w-mechanical-drawing.pdf",
    "u8": "https://store.tmotor.com/goods.php?id=561",
}

def items():
    """Scout electronics placed in the aircraft frame. Returns list of dicts."""
    L = []
    def add(key, name, dims, g, w_avg, w_peak, supply, sub, c, src, tag, notes="", qty=1):
        L.append(dict(key=key, name=name, dims=tuple(dims), mass_g=g, w_avg=w_avg, w_peak=w_peak, supply=supply, sub=sub,
                      c=tuple(float(v) for v in c), src=src, tag=tag, notes=notes, qty=qty))
    zA = ZA + MOUNT_T
    # --- avionics on tray A (printed adapters on the M3 20 mm grid)
    add("telem", "Telemetry radio RFDesign RFD900x (1 W, 902-928 MHz)", (57.7, 30.0, 12.8), 14, 1.5, 5.0, "5 V (PM08D aux)", "rf",
        (-410.0, 35.0, zA + 6.4), U["rfd"], "[S] dims/mass/TX 1 A; avg power [A] 30 % TX duty", "2x RP-SMA; ANT1 -> mast on fuse_seg5 (RG316)")
    add("adsb", "ADS-B In uAvionix pingRX Pro", (32.0, 31.0, 9.0), 8, 0.75, 0.75, "5 V", "rf", (-428.0, -33.0, zA + 4.5), U["ping"], "[S]",
        "MMCX 1090 MHz dipole, vertical in the nose ahead of F1")
    add("rid", "Remote ID Holybro RemoteID (ESP32-C3, CAN/UART)", (38.0, 26.5, 11.5), 27.5, 0.5, 1.0, "5 V", "rf", (-380.0, -35.0, zA + 5.75),
        U["rid"], "[S] dims/mass; power [A] (n.p.)", "SMA stub antenna up, under the printed avionics hatch")
    add("elrs", "ELRS 2.4 GHz receiver RadioMaster RP4TD (true diversity)", (23.7, 16.3, 4.0), 3.5, 0.5, 0.5, "5 V", "rf", (-337.0, -35.0, zA + 2.0),
        U["elrs"], "[S] 1.4 g + 2x T-antennas [A] 1 g each", "antennas 90 deg apart under the avionics hatch")
    add("pi", "Logger Raspberry Pi Zero 2 W (+ 64 GB card)", (65.0, 30.0, 10.0), 12, 1.75, 3.0, "5 V", "payload", (-325.0, 30.0, zA + 4.0 + 5.0),
        U["pizero"], "[S] footprint/holes 58x23 M2.5, 1.75 W; mass [A] ~10 g + card", "logs SHT45 (I2C) + MAVLink; Altum centre-of-frame pulse on a GPIO")
    # --- FC on the isolator under the battery tray, near the CG
    add("fc", "Flight controller ARKV6X on ARK PAB carrier", (74.0, 43.5, 17.0), 27, 2.5, 2.5, "5.2 V (PM08D)", "signal",
        (FC["x"], 0.0, FC["plate_top"] + 8.5), U["fc_ark"], "[S] ARKV6X 5 g 36x29x5 + carrier 22 g 74x43.5x12; height with module [A]",
        "carrier hole pattern n.p. (MEASURE, STEP available); IMU 0 mm from CG in x")
    # --- power
    add("pm", "Power module Holybro PM08D (2-14S, 200 A, INA228, 2x 5.2 V 3 A)", (101.0, 45.0, 26.0), 151, 1.0, 1.0, "battery", "power",
        (-132.0, 0.0, PDB["top"] + 13.0), U["pm08d"], "[S]", "mounting 79 x 38.1; has a fan")
    add("fuse", "Main fuse Littelfuse MEGA 70 V, 150 A (0898150) + bolt-down holder", (60.0, 32.0, 22.0), 45, 0.0, 0.0, "-", "power",
        (-214.0, 0.0, PDB["top"] + 11.0), U["mega"], "[S?] 70 V DC / 2.5 kA rating (search snippet); holder dims+mass [A] (MEASURE)",
        "MRBF (58 V) rejected: too little margin over 50.4 V")
    add("bus", "Bus-bar PDB: 2x Cu 3x10 bars on 2 mm G10, 6x M5 studs", (80.0, 40.0, 12.0), 70, 0.0, 0.0, "-", "power",
        (-140.0, 0.0, PDB["bot"] - 6.0), "-", "[A] own design", "under the PDB plate; feeds 4 lift ESCs, pusher ESC, BEC")
    add("bec", "BEC 12S in: 12 V 3 A (Altum) + 8 V 5 A (servos), Matek BEC12S-PRO class", (40.0, 25.0, 10.0), 15, 1.5, 3.0, "battery", "power",
        (-203.0, -28.0, PDB["bot"] - 5.0), "-", "[A] product not verified", "")
    add("as150", "Anti-spark connector Amass AS150U pair (battery lead)", (52.0, 24.0, 18.0), 41, 0.0, 0.0, "-", "power",
        (BAT_X - FF.BAT["l"]/2 - 32.0, 40.0, TRAY_TOP + 9.0), U["as150"], "[S?] 70-75 A cont / 140 A brief, 41 g/pair (reseller); dims [A]",
        "UNDERSIZED for the 190 A worst case: see open problems")
    add("caps", "Low-ESR bulk caps 2x 220 uF 63 V at each lift ESC (long DC leads)", (20.0, 12.5, 25.0), 40, 0.0, 0.0, "-", "power",
        (0, 0, 0), "-", "[A] 10 g per ESC", "mass only (inside each nacelle with the ESC)", qty=1)
    # --- propulsion electronics
    for i, (dx, dy, _) in enumerate([(0, 0, 0)] + FB._four()):
        xm, ym = d.FRONT_MOTOR_X + dx, d.BOOM_Y + dy
        E = FB.ESC
        add(f"esc{i}", "Lift ESC T-Motor ALPHA 60A 12S V1.2", (E["l"], E["t"], E["w"]), 73, 0.0, 0.0, "battery", "power",
            (xm + E["x0"] + E["l"]/2, ym, d.BOOM_Z + E["zc"]), U["alpha"], "[S] 78.7x32.5x16.2, 73 g with leads, 60 A / 80 A 10 s",
            "on its side under the boom tube, in boom_esc_cradle; leads power 14 AWG 160 mm, motor 16 AWG 65 mm [S]")
    add("esc_push", "Pusher ESC T-Motor FLAME 80A 12S V2", (77.0, 35.0, 18.0), 94, 0.0, 0.0, "battery", "power",
        (PUSH_ESC["x"], 0.0, PUSH_ESC["z"]), U["flame"], "[S] 80 A cont; dims/mass [S?] reseller", "fuse_pusher_esc_mount, belly inlets / side outlets in fuse_seg6")
    # --- servos (KST X10 V8)
    c, _, xle, _ = FW.station(FW.SERVO_Y); xc = 0.5*(xle + FW.SPLIT*c + FW.HINGE_X - FW.AIL_GAP/2)
    zl = FW.z_bounds(FW.SERVO_Y, xc)[0]
    for s in (1, -1):
        add(f"servo_ail_{'R' if s > 0 else 'L'}", "Aileron servo KST X10 V8 (10 mm)", (30.0, 35.5, 10.0), 28, 1.0, 11.0, "8 V BEC", "signal",
            (xc, s*FW.SERVO_Y, zl + 6.0), U["kst"], "[S] 28 g, 4.8-8.4 V; stall ~1.3-1.5 A [A]", "")
    for s in (1, -1):
        add(f"servo_rv_{'R' if s > 0 else 'L'}", "Ruddervator servo KST X10 V8 (10 mm)", (30.0, 35.5, 10.0), 28, 1.0, 11.0, "8 V BEC", "signal",
            tuple(FT_servo_c(s)), U["kst"], "[S]", "")
    # --- GNSS
    add("gnss", "GNSS ARK GPS (u-blox M9N, DroneCAN, mag/baro/safety switch)", (50.0, 50.0, 10.0), 16, 0.6, 0.6, "5 V (CAN)", "rf",
        (FM.GNSS["x"], FM.GNSS["y"], gnss_z() + 5.0), U["ark_gps"], "[S]", "on misc_gnss_pad_R (wing top, y +400); mounting n.p. (MEASURE vs the 4x M3 pad)")
    # --- payload
    add("altum", "MicaSense Altum-PT (5-band + pan + LWIR)", (ALT["l"], ALT["w"], ALT["h"]), 406, 10.0, 12.0, "12 V (BEC), 7-25.2 V", "payload",
        ALT_C, U["altum"], "[S] 110x80x69, 460 g with DLS 2 (camera alone 406 [C]); 10 W cont / 12 W peak",
        "hangs under pay_altum_deck on 4x M3 (cross pattern 60 / 35); has a fan, nadir window 70x56 (MEASURE lens cluster)")
    add("dls", "MicaSense DLS 2 light sensor (with GPS)", (63.5, 46.0, 14.0), 54, 0.0, 0.0, "from Altum", "payload",
        (DLS["x"], 0.0, nose_top(DLS["x"]) + DLS["ped_h"] + 7.0), U["altum"], "[S] 63.5x46x14.03, 54 g, 2x M2 40.75 apart, 60 cm cable",
        "on elec_dls_pedestal (avionics hatch top, level): no prop disc overhead (front disc edge 245 mm outboard), cable ~0.3 m to the camera")
    add("sht45", "Air T/RH: Sensirion SHT45 on Adafruit #5665 in a printed 4-plate shield", (17.7, 4.8, 25.5), 1.7, 0.01, 0.01, "3.3 V (Pi)", "payload",
        (PROBE["x_tip"], 0.0, PROBE["z"] - 2.0), U["sht45"], "[S] +-0.1 C / +-1 %RH typ, tau63 2 s T / 4 s RH", "at the tip of a 200 mm nose probe, ahead of the prop discs")
    # --- RF
    add("ant_telem", "915 MHz half-wave dipole (RFD900x ANT1) on printed mast", (12.0, 12.0, MAST["h"] - 20.0), 15, 0, 0, "-", "rf",
        (MAST["x"], 0.0, FF.top_z(MAST["x"]) + 20.0 + (MAST["h"] - 20.0)/2), "-", "[A] 15 g", "vertical, 245 mm from the nearest prop disc edge, 600 mm from the carbon booms")
    add("ant_adsb", "1090 MHz dipole (pingRX kit)", (8.0, 8.0, 137.0), 8, 0, 0, "-", "rf", (-495.0, -40.0, -82.0), U["ping"], "[S] included; mass [A]",
        "vertical, taped to the nose skin ahead of F1, no carbon within 250 mm")
    # --- wing-break connector (JST-GH 6 receptacle in c2 + plug in o1), sits in the pocket at y = +/-600
    for s in (1, -1):
        add(f"conn_break_{'R' if s > 0 else 'L'}", "Wing-break connector JST-GH 6-pin pair (mated)", (8.75, 11.4, 4.15), 0.6, 0, 0, "-", "signal",
            (FW.CONN["x"], s*FW.CH, FW._mid_z(FW.CONN["x"], FW.CH)), "https://www.jst-mfg.com/product/pdf/eng/eGH.pdf", "[S] 1 A/pin, 8.75 x 4.15 x 5.7 receptacle",
            "S, 2x +, 2x -, spare; mass counted in the wire table")
    # --- lights (self-powered)
    for s in (1, -1):
        add(f"strobe_{'R' if s > 0 else 'L'}", "Anti-collision strobe Firehouse ARC V v2 (internal battery)", (38.0, 15.0, 25.0), 13, 0, 0, "internal Li-ion, USB-C", "lights",
            (94.0, s*(d.SPAN/2 + 3.0 + 1.6 + 7.5), 16.0), U["arcv"], "[S] 38x25x15, 13 g, ~6 h, 4 SM", "elec_wingtip_strobe (also closes the open wing-tip end)")
    add("strobe_belly", "Anti-collision strobe Firehouse ARC V v2", (38.0, 25.0, 15.0), 13, 0, 0, "internal", "lights",
        (460.0, 0.0, FF.params(441.0)[2] - FF.params(441.0)[1] - 8.5), U["arcv"], "[S]", "elec_belly_strobe; no wiring, charge before flight")
    return L


def FT_servo_c(s):
    loc = FT.frame_at(FT.SERVO["u"], FT.SERVO["f"])
    p = loc*__import__("build123d").Location((0, 0, -3))
    v = p.position
    return (v.X, s*v.Y, v.Z)


# ------------------------------------------------------------------ relay items removed in the Scout config (for the mass/CG delta)
REMOVED_RELAY = ["jetson_ark_fc", "manet_radio", "gnss_rx_x2", "satcom_rockblock", "pdb_power_module", "lidar_lw20", "adsb_remote_id",
                 "mesh_vhf_scanner", "relay_duplexer_TMND3", "relay_repeater_radio", "eoir_turret_HD25", "nadir_thermal_boson",
                 "gnss_antenna_R", "gnss_antenna_L", "wiring_harness", "strobes_nav_lights", "vhf_element_coax", "esc_pusher",
                 "esc_alpha60_0", "esc_alpha60_1", "esc_alpha60_2", "esc_alpha60_3",
                 "servo_aileron_R", "servo_aileron_L", "servo_ruddervator_R", "servo_ruddervator_L"]
REMOVED_PARTS = ["fuse_relay_cradle", "fuse_turret_mount", "fuse_avionics_tray_B", "misc_gnss_pad_L"]
