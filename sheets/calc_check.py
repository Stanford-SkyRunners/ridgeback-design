import formulas, sys, re, json
fn = "Ridgeback_mini_VTOL_design_2026-11-17.xlsx"
xl = formulas.ExcelModel().loads(fn).finish()
sol = xl.calculate()
vals = {}
errs = []
for k, v in sol.items():
    try: x = v.value[0][0]
    except Exception: x = v
    vals[k.upper()] = x
    if isinstance(x, str) and x.startswith("#") or type(x).__name__ in ("XlError",):
        errs.append((k, x))
    if hasattr(x, "__class__") and "Error" in type(x).__name__: errs.append((k, str(x)))
print("cells", len(vals), "errors", len(errs))
for e in errs[:30]: print("ERR", e)
json.dump({k: (float(v) if isinstance(v, (int, float)) else str(v)) for k, v in vals.items()}, open("calc_values.json", "w"))
