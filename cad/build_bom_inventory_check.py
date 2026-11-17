"""
Cross-check the HIVE-QUEEN v12 BOM against the shop inventory and write a buy-list workbook.

Reads  cad/out/HIVE_QUEEN_149_BOM_CHECKLIST.csv  (written by cad/build_bom_v12.py)
       cad/Inventory(Motion).csv                 (shop stock, "Part name,# we have")
Writes cad/out/HIVE_QUEEN_149_BOM_INVENTORY_CHECK.xlsx

  uv run --with openpyxl python cad/build_bom_inventory_check.py

Column order: Check, Item, Price, Qty, Link, then the rest.

Column A ("Have?") is the only cell a human edits - TRUE means we have it. Everything else is
derived from it so checking a part off is ONE cell, not four:
  - row colour comes from conditional formatting (green = ticked, amber = partial, grey = field
    element), so it repaints the moment you tick;
  - line cost is =IF($A{n}=FALSE,$N{n},0) - a blank or FALSE cell still costs money, TRUE (or any
    text you type in it) zeroes the row;
  - the TOTAL cell is =SUM of that live column, so it re-adds itself on every tick.
Values are real booleans, so Excel 365's Insert > Checkbox and Google Sheets' Insert > Tick box
both turn column A into one-click checkboxes.

Team rule (2026-11): all M4 socket head cap screws are assumed on hand even though the
inventory sheet does not list fasteners. M4 nylocks and heat-set inserts are NOT covered.
"""
import csv
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
BOM_CSV = os.path.join(OUT, "HIVE_QUEEN_149_BOM_CHECKLIST.csv")
INV_CSV = os.path.join(HERE, "Inventory(Motion).csv")
XLSX = os.path.join(OUT, "HIVE_QUEEN_149_BOM_INVENTORY_CHECK.xlsx")

# --------------------------------------------------------------- inventory
inv = {}
with open(INV_CSV, encoding="utf-8-sig") as f:
    for row in csv.reader(f):
        if len(row) >= 2 and row[0].strip():
            try:
                inv[row[0].strip()] = int(row[1])
            except ValueError:
                inv[row[0].strip()] = 0

# --------------------------------------------------------------- BOM
with open(BOM_CSV, encoding="utf-8-sig") as f:
    src = list(csv.reader(f))
hdr_in, body = src[0], [r for r in src[1:] if r[4].strip()]      # drop the CSV's own TOTAL row
I = {h: i for i, h in enumerate(hdr_in)}

# "all M4 screws" per the team: cap screws only, not nuts/inserts
M4_SCREW = re.compile(r"^M4 x \d+ mm socket head cap screw")


def classify(r):
    """-> (check, reason). Reason explains what the inventory says."""
    st, item, notes = r[I["Status"]], r[I["Item / Part Name"]], r[I["Notes"]]
    m = re.search(r"IN STOCK: have (\d+) in Inventory", notes)
    if M4_SCREW.match(item):
        return "YES", "Assumed on hand - team stocks all M4 cap screws (not on the inventory sheet)"
    if st.startswith("Field"):
        return "N/A", "Field element - supplied with the field, never purchased"
    if st.startswith("Verify Owned"):
        return "YES", "Team-owned kit / shop tooling - verify before the first meet"
    if st.startswith("In Stock (Motion Inv)"):
        return "YES", f"Inventory has {m.group(1)} (need {r[I['Qty']]})" if m else "In Inventory(Motion).csv"
    if st.startswith("In Stock (cut)"):
        return "YES", "Cut from 8 mm REX shaft stock already in inventory"
    if st.startswith("Partial"):
        h = re.search(r"have (\d+) in Inventory.*?ORDER (\d+)", notes)
        return "PARTIAL", (f"Inventory has {h.group(1)} of {r[I['Qty']]} - order {h.group(2)} more" if h
                           else "Partial stock - order the balance")
    if st.startswith("To Print") or st.startswith("To CNC"):
        return "NO", "Custom part - make it (filament / CNC quote), not a stock item"
    return "NO", "Not in Inventory(Motion).csv"


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return v if v else ""


rows = []
for r in body:
    check, why = classify(r)
    cost = num(r[I["Line cost (USD)"]])
    if check == "YES" and not isinstance(cost, float):
        cost = 0.0
    rows.append({
        "check": check, "why": why,
        "item": r[I["Item / Part Name"]],
        "price": num(r[I["Price per pack (USD)"]]),
        "qty": num(r[I["Qty"]]),
        "link": r[I["Link"]],
        "status": r[I["Status"]],
        "category": r[I["Category"]],
        "stage": r[I["Stage / Subsystem"]],
        "vendor": r[I["Vendor / Process"]],
        "sku": r[I["SKU / Part Number / Spec"]],
        "pack": num(r[I["Pack qty"]]),
        "packs": num(r[I["Packs to buy"]]),
        "cost": cost if cost != "" else 0.0,
        "cad": r[I["CAD Occurrence / Body Names"]],
        "notes": r[I["Notes"]],
        "basis": r[I["Price basis / notes"]],
    })

HDR = ["Have?", "Item / Part Name", "Price (USD)", "Qty", "Link", "Inventory says (reference)",
       "BOM status (generated)", "Category", "Stage / Subsystem", "Vendor / Process",
       "SKU / Part Number / Spec", "Pack qty", "Packs to buy", "Cost if bought (USD)",
       "Line cost (USD)", "CAD Occurrence / Body Names", "Notes", "Price basis / notes"]
KEYS = ["have", "item", "price", "qty", "link", "why", "status", "category", "stage", "vendor",
        "sku", "pack", "packs", "cost", None, "cad", "notes", "basis"]
# Column A is the ONLY cell anyone edits: TRUE = we have it, FALSE/blank = still buying it.
# Row colour, line cost and the totals are all derived from it by conditional formatting and
# formulas, so checking a part off is one cell, not four.
for r in rows:
    r["have"] = (r["check"] == "YES")

to_buy = sum(r["cost"] for r in rows if r["check"] != "YES" and isinstance(r["cost"], float))
owned = sum(r["cost"] for r in rows if r["check"] == "YES" and isinstance(r["cost"], float))
print(f"rows {len(rows)} | YES {sum(r['check'] == 'YES' for r in rows)} "
      f"PARTIAL {sum(r['check'] == 'PARTIAL' for r in rows)} "
      f"NO {sum(r['check'] == 'NO' for r in rows)} "
      f"N/A {sum(r['check'] == 'N/A' for r in rows)}")
print(f"total to buy ${to_buy:,.2f} | saved by inventory + M4 rule ${owned:,.2f}")

# --------------------------------------------------------------- workbook
import openpyxl
from openpyxl.formatting.rule import FormulaRule
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter

GREEN, AMBER, GREY, HEAD = "FFD8F3DC", "FFFFF3C4", "FFEFEFEF", "FF1F2937"
wb = openpyxl.Workbook()
ws = wb.active
ws.title = "BOM vs Inventory"
ws.append(HDR)
for n, r in enumerate(rows, start=2):
    ws.append([("" if k is None else r[k]) for k in KEYS])
    # A blank or FALSE cell still costs money; TRUE (or any text you type) checks the row off.
    ws.cell(n, 15).value = f"=IF($A{n}=FALSE,$N{n},0)"           # Line cost
last = len(rows) + 1

tot = last + 2
ws.cell(tot, 13).value = "TOTAL to buy"
ws.cell(tot, 15).value = f"=ROUND(SUM($O$2:$O${last}),2)"
ws.cell(tot, 16).value = "Everything still unchecked, qty + pack sizes included. Excludes tax, shipping, CNC quotes."
ws.cell(tot + 1, 13).value = "Covered (checked off)"
ws.cell(tot + 1, 15).value = f"=ROUND(SUM($N$2:$N${last})-SUM($O$2:$O${last}),2)"
ws.cell(tot + 1, 16).value = f'=COUNTIF($A$2:$A${last},TRUE)&" of {len(rows)} rows checked off"'
for rr in (tot, tot + 1):
    for cc in range(13, 17):
        ws.cell(rr, cc).font = Font(bold=(rr == tot), size=12 if rr == tot else 11)
        ws.cell(rr, cc).fill = PatternFill("solid", fgColor="FFFDE68A" if rr == tot else GREEN)
    ws.cell(rr, 15).number_format = '"$"#,##0.00'

# styling
thin = Side(style="thin", color="FFD1D5DB")
for c in ws[1]:
    c.font = Font(bold=True, color="FFFFFFFF")
    c.fill = PatternFill("solid", fgColor=HEAD)
    c.alignment = Alignment(wrap_text=True, vertical="center", horizontal="center")
ws.row_dimensions[1].height = 32

for n, r in enumerate(rows, start=2):
    row = ws[n]
    for c in row:
        c.alignment = Alignment(wrap_text=True, vertical="top")
        c.border = Border(bottom=thin)
    row[0].font = Font(bold=True)
    row[0].alignment = Alignment(horizontal="center", vertical="center")
    for ci in (2, 13, 14):                                       # price, cost if bought, line cost
        ws.cell(n, ci + 1).number_format = '"$"#,##0.00'
    if r["link"].startswith("http"):
        lc = ws.cell(n, 5)
        lc.hyperlink = r["link"]
        lc.value = (r["vendor"] or "link") + " \u2192"
        lc.font = Font(color="FF1D4ED8", underline="single")

# Live row colour, driven only by column A (plus the generated status for the amber/grey cases).
# First rule wins, so a checked row is green even if the BOM called it Partial.
band = f"A2:{get_column_letter(len(HDR))}{last}"
for formula, colour in (
    ("$A2<>FALSE", GREEN),                                       # checked off -> pastel green
    ('AND($A2=FALSE,LEFT($G2,7)="Partial")', AMBER),             # own some, buying the balance
    ('AND($A2=FALSE,$G2="Field Element")', GREY),                # not purchased at all
):
    # In a differential (conditional) style Excel paints the cell from bgColor, NOT fgColor -
    # a solid patternFill with only fgColor set renders black. Match what Excel itself writes.
    ws.conditional_formatting.add(band, FormulaRule(formula=[formula], stopIfTrue=True,
                                                    fill=PatternFill(bgColor=colour)))

for col, w in zip(range(1, len(HDR) + 1),
                  (9, 46, 11, 6, 18, 42, 20, 28, 20, 24, 28, 8, 8, 13, 13, 50, 70, 70)):
    ws.column_dimensions[get_column_letter(col)].width = w
ws.freeze_panes = "C2"
ws.auto_filter.ref = f"A1:R{last}"
from openpyxl.comments import Comment
ws["A1"].comment = Comment(
    "Tick this box (TRUE) and the whole row turns pastel green, its line cost drops to 0 and the "
    "TOTAL at the bottom re-adds itself. This is the ONLY column to edit - everything else is "
    "generated.\n\n"
    "Excel 365: select A2:A117, Insert > Checkbox, then click to tick.\n"
    "Google Sheets: select A2:A117, Insert > Tick box.\n"
    "Anywhere else: type TRUE (or x) and press Enter.\n\n"
    "Filter this column to FALSE to see only what is still left to buy.", "build_bom_inventory_check.py")

# ---- judgement calls: inventory lines that MIGHT cover a BOM row. Not auto-checked, because the
# inventory uses shop nicknames and a wrong match ships a robot with the wrong bore. Eyeball these.
REVIEW = [
    ("Servo rex adapter x1", "Clamping servo-to-shaft coupler, 25T spline to 8 mm REX (x3, $9.99 ea)",
     "Likely the same goBILDA 4001-0025-4008. If it is, order 2 instead of 3 (-$9.99).", "$9.99"),
    ("Sonic Hubs (small) x12, (big) x8", "Hyper Hub 1310-0016-4008 (x4) + 1313-1632-4008 (x2), $57.94",
     "Sonic Hub != Hyper Hub, but check the bore: a REX-bore Sonic Hub with the right bolt circle can "
     "replace the plain hubs. The 1313 flywheel hub needs the 16+32 mm pattern.", "$57.94"),
    ("Bearings (non-rec) x14", "8 mm REX flanged ball bearing 1611-0514-4008 (38 needed, 10 matched)",
     "Only 'Bearings (rec)' (10) were counted. If the 14 non-REX ones are 14 mm OD flanged they still "
     "fit round-shaft hubs and cut the order.", "up to $30"),
    ("5m belt x2; 300/228/252/236/257/233 tooth belts x2 each",
     "HTD-5M 43T/49T/52T + HTD-3M 71T/86T closed-loop belts, $44.43",
     "No tooth count in the inventory matches the exact C-C ledger, so all belts stay on the order. "
     "Measure the two unlabelled '5m belt' coils before ordering.", "$0-44.43"),
    ("Bevel Gears (big) x10", "1:1 steel miter gear MOD 1.25 30T 8 mm REX, 2315-4008-0030 (x8)",
     "Auto-marked IN STOCK. Confirm they are the 8 mm REX 2315 series and not a round-bore/other-MOD "
     "set - the whole drivetrain torque path depends on it.", "-$127.92 if wrong"),
    ("Boot Wheel big x2 / Stretch intake wheel big x8 / gobuild compliant wheels medium x2",
     "96 mm Rhino, 72 mm gecko feeders, 72 mm compliant counter-wheel",
     "Nickname matches made by build_bom_v12.py. Check diameter and bore (14 mm) before trusting them.",
     "-$36.95 if wrong"),
    ("80/104/144/264/288/312 mm shaft stock (10 pcs)", "8 mm REX shaft, 20 cut lengths",
     "Marked IN STOCK via the bin-packing cut plan (2 mm kerf). Confirm the stock is REX, not round.",
     "-$17.99 if wrong"),
    ("(fasteners are not on the inventory sheet)", "M4 x 8/10/12/14/16/25/35/40/60 cap screws",
     "Checked YES on your call that the team stocks all M4 cap screws. M4 nylock nuts (101) and M4 "
     "heat-set inserts (89) are NOT covered and stay on the order.", "+$60.84 if wrong"),
]
rv = wb.create_sheet("Verify these matches")
rv.append(["Inventory line", "BOM row it might cover", "What to check", "$ at stake"])
for r_ in REVIEW:
    rv.append(list(r_))
for c in rv[1]:
    c.font = Font(bold=True, color="FFFFFFFF")
    c.fill = PatternFill("solid", fgColor=HEAD)
    c.alignment = Alignment(wrap_text=True, vertical="center")
for row in rv.iter_rows(min_row=2):
    for c in row:
        c.alignment = Alignment(wrap_text=True, vertical="top")
        c.fill = PatternFill("solid", fgColor=AMBER)
for col, w in zip("ABCD", (44, 52, 76, 16)):
    rv.column_dimensions[col].width = w
rv.freeze_panes = "A2"

# ---- inventory reference sheet
iv = wb.create_sheet("Inventory(Motion)")
iv.append(["Part name", "# we have"])
for k, v in inv.items():
    iv.append([k, v])
for c in iv[1]:
    c.font = Font(bold=True, color="FFFFFFFF")
    c.fill = PatternFill("solid", fgColor=HEAD)
iv.column_dimensions["A"].width = 38
iv.column_dimensions["B"].width = 12
iv.freeze_panes = "A2"

wb.save(XLSX)
print("wrote", XLSX)
