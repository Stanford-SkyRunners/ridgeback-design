import formulas, json, glob, os, openpyxl
FULL = json.load(open("calc_values.json")); FN = "RIDGEBACK_MINI_VTOL_DESIGN_2026-11-17.XLSX"
tot_bad = 0
for f in sorted(glob.glob("split/*.xlsx")):
    sol = formulas.ExcelModel().loads(f).finish().calculate()
    base = os.path.basename(f).upper()
    wbf = openpyxl.load_workbook(f)
    errs = cmp = bad = 0
    for k, v in sol.items():
        try: x = v.value[0][0]
        except Exception: continue
        if "Error" in type(x).__name__ or (isinstance(x, str) and x.startswith("#")): errs += 1; print("ERR", k, x); continue
        sh_cell = k.split("]")[1]; sh, cell = sh_cell.split("'!")
        if sh in ("README", "LINKED_INPUTS"): continue
        full = FULL.get(f"'[{FN}]{sh}'!{cell}")
        if isinstance(x, (int, float)) and isinstance(full, (int, float)):
            cmp += 1
            if abs(x - full) > 1e-9 * max(1, abs(full)): bad += 1; print("DIFF", k, x, full)
    tot_bad += bad + errs
    print(os.path.basename(f), "errors", errs, "numbers compared", cmp, "differences", bad)
print("TOTAL problems", tot_bad)
