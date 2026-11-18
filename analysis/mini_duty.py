"""Import duty on the China-origin parts of the smaller own design (docs/OWN_SMALL_DRONE_2026-10-30.md §4j).
Rates read 2026-10-30 [V] in the official tariff schedule, HTS 2026 Revision 22 (current from 10-30), hts.usitc.gov:
the reststop/search API for the product lines and the Chapter 99 PDF for the extra China duties. Item values are the
cart totals of 10-30 (§4i). Run: python3 analysis/mini_duty.py -> out/mini_duty.md
"""
from pathlib import Path
OUT = Path(__file__).resolve().parent / "out"
S301 = {"List 1": 0.25, "List 2": 0.25, "List 3": 0.25}   # 9903.88.01 / .02 / .03, U.S. note 20(b)/(d)/(f) [V]
FORCED_LABOR = 0.125                                        # 9903.05.31, products of China, from 2026-07-24 [V]
ITEMS = [  # name, store, value $, HTS line [A, our classification], normal rate [V], Section 301 list [V]
    ("Carbon tubes (jointed boom set + spars)", "HaoZhong (China)", 197.01, "6815.13.00 other articles of carbon fibres", 0.0, "List 3"),
    ("4 x MN5008 KV340 lift motors", "unmannedrc (ships from China)", 4 * 89.99, "8501.31.40 DC motor 74.6-735 W", 0.04, "List 2"),
    ("AT3520 KV550 pusher motor", "unmannedrc", 79.99, "8501.31.40 (8501.32.20 at 2.9 % if rated over 750 W)", 0.04, "List 2"),
    ("2 pairs P18x6.1 props", "unmannedrc", 2 * 82.90, "8807.10.00 propellers, parts of aircraft", 0.0, "List 1"),
    ("5 x ALPHA 40A ESCs", "unmannedrc", 5 * 69.99, "8504.40.95 static converters, other", 0.0, "List 3"),
]
BROKER = (20, 60)   # [A] carrier clearance fees for 2 parcels (de minimis is suspended, so every parcel is a formal or informal entry)
L = ["# Import duty, smaller own design [C, `analysis/mini_duty.py`; rates V 2026-10-30, HTS 2026 Rev 22]\n",
     "| Item | Store | Value $ | Tariff line [A] | Normal | Section 301 | Forced-labor duty | Total rate | Duty $ |",
     "|---|---|---|---|---|---|---|---|---|"]
tv = td = 0.0
for name, store, v, hts, mfn, lst in ITEMS:
    r = mfn + S301[lst] + FORCED_LABOR
    tv += v; td += v * r
    L.append(f"| {name} | {store} | {v:.2f} | {hts} | {100*mfn:.1f} % | 25 % ({lst}) | 12.5 % | {100*r:.1f} % | {v*r:.2f} |")
L.append(f"| **Total** | | **{tv:.2f}** | | | | | **{100*td/tv:.1f} %** | **{td:.2f}** |")
L.append(f"\nPlus carrier clearance fees about ${BROKER[0]}-{BROKER[1]} [A]. Worst case if customs reads the ESCs as drone parts under "
         f"9903.08.21 (+100 %): +${5*69.99:.0f}. Duty is on the goods value; shipping excluded when billed separately [A].")
us = sum(v * (mfn + S301[l] + FORCED_LABOR) for n, s, v, h, mfn, l in ITEMS if s.startswith("unmannedrc"))
L.append(f"If motors, props and ESCs come from US stock at the same price (only IR-LOCK found, out of stock 10-30; insightfpv ships from China), "
         f"we pay no duty on them: -${us:.2f}, leaving ${td-us:.2f} on the tubes.")
(OUT / "mini_duty.md").write_text("\n".join(L) + "\n")
print("\n".join(L))
