"""
Vendor links + prices for the HIVE-QUEEN BOM (read by cad/build_bom_v12.py).
Every price below was read from the live product page on 2026-11-16 (USD, before tax/shipping).
Keys match the BOM row's SKU column (exact) or, for custom parts, the row's process text.
  price  = price per purchase unit (one pack), USD   pack = units per purchase unit
  sku    = corrected SKU when ours was wrong (the BOM row is rewritten and the change noted)
Re-check before ordering; vendors change prices and stock.
"""
CHECKED = "2026-11-16"
GB = "https://www.gobilda.com/"

BY_SKU = {
    "1120-0017-0432": dict(url=GB + "1120-series-u-channel-17-hole-432mm-length/", price=20.99, pack=1),
    "1120-0010-0264": dict(url=GB + "1120-series-u-channel-10-hole-264mm-length/", price=13.99, pack=1),
    "1120-0005-0144": dict(url=GB + "1120-series-u-channel-5-hole-144mm-length/", price=8.99, pack=1),
    "1120-0004-0120": dict(url=GB + "1120-series-u-channel-4-hole-120mm-length/", price=7.99, pack=1),
    "1201-0043-0002": dict(url=GB + "1201-series-quad-block-pattern-mount-43-2/", price=6.99, pack=1),
    "5203-2402-0019": dict(url=GB + "5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/", price=54.99, pack=1),
    "5203-2402-0005": dict(url=GB + "5203-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-8mm-rex-shaft-1150-rpm-3-3-5v-encoder/", price=54.99, pack=1),
    "2000-0025-0003": dict(url=GB + "2000-series-dual-mode-servo-25-2-torque/", price=36.99, pack=1, sku="2000-0025-0002",
                           note="SKU fixed: 2000-0025-0003 is the 25-3 SPEED servo; the TORQUE servo is 2000-0025-0002 (same price)"),
    "REV-31-1595": dict(url="https://www.revrobotics.com/rev-31-1595/", price=375.00, pack=1),
    "REV-31-1153": dict(url="https://www.revrobotics.com/rev-31-1153/", price=275.00, pack=1),
    "14-0014": dict(url="https://www.revrobotics.com/rev-31-1302/", price=60.00, pack=1,
                    note="Matrix 14-0014 is DISCONTINUED / sold out everywhere (goBILDA & ServoCity pages). Still FTC-legal if you own one. "
                         "Buy instead: REV Slim Battery REV-31-1302 (12 V 3000 mAh NiMH, XT30, 20 A fuse, FTC-legal Table 12-4), $60.00. "
                         "Check its fit on the battery tray (different pack shape from the CAD model)"),
    "3110-0002-0002": dict(url=GB + "pinpoint-v2-odometry-computer-imu-sensor-fusion-for-2-wheel-odometry/", price=79.99, pack=1),
    "3110-0001-0001": dict(url=GB + "swingarm-odometry-pod-48mm-wheel/", price=99.99, pack=1,
                           note="3110-0001-0001 is the SWINGARM pod with a 48 mm wheel (the CAD model). The 4-Bar pod (32 mm wheel) is 3110-0001-0002, also $99.99. "
                                "Bundle 3203-3110-0001 (2 swingarm pods + Pinpoint) is $279.99, no saving"),
    "REV-31-1557": dict(url="https://www.revrobotics.com/rev-31-1557/", price=20.75, pack=1, note="each includes a 30 cm 4-pin JST-PH cable"),
    "REV-31-1505": dict(url="https://www.revrobotics.com/rev-31-1505/", price=31.50, pack=1, note="each includes a 30 cm 4-pin JST-PH cable"),
    "SEN0305": dict(url="https://www.dfrobot.com/product-1922.html", price=34.90, pack=1, note="HuskyLens 2 (SEN0638, $89.90) is a newer model, not required"),
    "3125-0001-0001": dict(url=GB + "6v-servo-power-injector-6-channel-8-15v-input/", price=69.99, pack=1),
    "REV-31-1387": dict(url="https://www.revrobotics.com/rev-31-1387/", price=13.00, pack=1),
    "3606-0001-0096": dict(url=GB + "96mm-mecanum-wheel-set-70a-durometer-bearing-supported-rollers/", price=169.99, pack=4, sku="3213-3606-0002",
                           note="SKU fixed: 3606-0001-0096 does not exist; the 96 mm mecanum SET of 4 is 3213-3606-0002. FTC team price shown on the page: $127.49"),
    "2305 Series": dict(url=GB + "2315-series-steel-miter-gear-mod-1-25-set-screw-8mm-rex-bore-30-tooth/", price=15.99, pack=1, sku="2315-4008-0030",
                        note="goBILDA's 1:1 REX miter gear is the 2315 Series steel MOD 1.25 30T (sold singly; 2 make a pair). "
                             "CAD GAP: the modelled miter has a ~32 mm pitch diameter; the 2315 is 37.5 mm, so the gear pair must be re-modelled before cutting the rails"),
    "2315-4008-0030": dict(url=GB + "2315-series-steel-miter-gear-mod-1-25-set-screw-8mm-rex-bore-30-tooth/", price=15.99, pack=1, note="sold singly; 2 per corner"),
    "3100-0012-0020": dict(url=GB + "12v-nimh-nested-battery-3000mah-mh-fc-xt30-connector/", price=64.99, pack=1),
    "1209-0001-0001": dict(url=GB + "battery-mount-for-12v-nimh-nested-battery/", price=7.99, pack=1),
    "1613-0516-0008": dict(url=GB + "8mm-id-thrust-ball-bearing-15-5mm-od-5mm-thickness/", price=3.99, pack=1),
    "1611-0514-4008": dict(url=GB + "1611-series-flanged-ball-bearing-8mm-rex-id-x-14mm-od-5mm-thickness-2-pack/", price=5.99, pack=2),
    "2100-0008 series": dict(url=GB + "8mm-rex-shaft-with-e-clip-stainless-steel-624mm-length/", price=17.99, pack=1, sku="2106-4008-6240 (624 mm stock)",
                             note="REX shaft is the 2106 series (2100 is round shaft). 624 mm is the longest stock; 432 mm is 2106-4008-4320, $13.49"),
    "3412-0008-0016 (HTD-5M 16T)": dict(url=GB + "3417-series-5mm-htd-pitch-set-screw-pinion-timing-belt-pulley-8mm-rex-bore-16-tooth/", price=9.99, pack=1,
                                        sku="3417-4008-0016", note="SKU fixed (3412 is goBILDA's belt prefix; 3414 16T is discontinued)"),
    "3412-0008-0020 (HTD-5M 20T)": dict(url="https://www.amazon.com/dp/B0FNQTJZMX", price=9.99, pack=1, sku="Amazon B0FNQTJZMX (20-5M-15, 8 mm bore)",
                                        note="goBILDA sells NO 20T HTD-5M pulley (REX pinions are 16T/24T only). This aluminium 20T has an 8 mm round bore with 2x M5 set screws "
                                             "(seat them on REX flats). It is cut for 15 mm belts, so it is WIDER than the modelled 9 mm-belt pulley: check clearance, "
                                             "or print 20T HTD-5M REX pulleys in PETG-CF"),
    "3412-0008-0032 (HTD-5M 32T)": dict(url="https://www.amazon.com/dp/B0BGCH8RZ2", price=15.61, pack=1, sku="Amazon B0BGCH8RZ2 (5M 32T, 8 mm bore, 16 mm tooth width)",
                                        note="goBILDA sells NO 32T HTD-5M pulley. 8 mm round bore (set screws on REX flats); 16 mm wide, wider than modelled: check clearance"),
    "3416-0009-0215": dict(url=GB + "5mm-pitch-htd-timing-belt-9mm-width-215mm-pitch-length-43-tooth/", price=4.79, pack=1, sku="3412-0009-0215", note="SKU fixed (3412 series)"),
    "3416-0009-0245": dict(url=GB + "3412-series-5mm-htd-pitch-timing-belt-9mm-width-245mm-pitch-length-49-tooth/", price=4.99, pack=1, sku="3412-0009-0245", note="SKU fixed (3412 series)"),
    "3416-0009-0260": dict(url="https://www.amazon.com/dp/B0D4F5K5TQ", price=9.84, pack=1, sku="Goodyear 260-5M-9 (Amazon B0D4F5K5TQ)",
                           note="goBILDA has no 260 mm 9 mm belt (215/225/245/265...). The belt ledger needs exactly 260 mm / 52T: Goodyear 260-5M-9"),
    "213-3M-06": dict(url="https://www.mdsofmi.com/product/3m-htd-timing-belt-213-3m-06/", price=4.99, pack=1, sku="Gates 213-3M-06"),
    "258-3M-06": dict(url="https://www.mdsofmi.com/product/3m-htd-timing-belt-258-3m-06/", price=4.99, pack=1, sku="Gates 258-3M-06"),
    "3614-0014-0096": dict(url=GB + "3614-series-rhino-wheel-14mm-bore-96mm-diameter-30a-durometer/", price=9.99, pack=1,
                           note="14 mm bore: mounts on a 1310-0016-4008 Hyper Hub (see the hub row)"),
    "82 mm OD x 10 mm, 8 mm REX": dict(url=GB + "steel-flywheel-82mm-diameter-152g-1651-g-cm/", price=15.99, pack=1, sku="goBILDA 3628-0032-0082 Steel Flywheel 82 mm",
                                       note="Off-the-shelf 82 mm steel flywheel (152 g, 1651 g*cm^2) bolts on via a 32 mm M4 pattern. Lighter than the 10 mm-thick disc in the spec "
                                            "(~415 g): stack 2-3 or make a custom disc"),
    "3613 series": dict(url=GB + "gripforce-gecko-wheel-14mm-bore-72mm-diameter-30a-durometer/", price=8.99, pack=1, sku="3613-0014-0072",
                        note="72 mm Gecko is 14 mm bore only: mounts on a 1310-0016-4008 Hyper Hub"),
    "9654K series, 6 mm OD": dict(url="https://www.amazon.com/dp/B0GTW1KZMB", price=6.99, pack=5, note="McMaster pages show no prices to bots; Amazon 6 mm OD x 30 mm 304 SS, 5-pack"),
    "9271K series": dict(url="https://www.amazon.com/dp/B0CR864MWT", price=7.02, pack=10, note="10 mm OD x 0.8 mm wire, 180 deg (ID ~8.4 mm clears the 8 mm shaft), 10-pack"),
    "98381A series": dict(url="https://www.amazon.com/dp/B07MB8T17Q", price=7.49, pack=10, note="6 x 20 mm 304 SS dowel, 10-pack"),
    "91259A series": dict(url="https://www.amazon.com/dp/B077GWBKC1", price=9.29, pack=5, note="ISO 7379 8 mm shoulder x 12 mm, M6 thread, 5-pack"),
    "2800-0004-0012": dict(url=GB + "m4-x-0-7mm-zinc-plated-socket-head-screw-12mm-length/", price=6.58, pack=1,
                           note="$3.59 / 25 M4x12 SHCS + $2.99 / 25 M4 nylock nuts (" + GB + "m4-x-0-7mm-nylock-nut/)"),
    "94180A353 or RX-M4x8.1": dict(url="https://www.amazon.com/dp/B07YSV66Y5", price=9.99, pack=50, sku="ruthex RX-M4x8.1 (50-pack)"),
    "94180A331 or RX-M3x5.7": dict(url="https://www.amazon.com/dp/B0CDH36ZMX", price=19.48, pack=100,
                                   note="$9.99 / 100 ruthex M3 inserts + $9.49 750-pc M3 SHCS kit (https://www.amazon.com/dp/B0GHMM79M9). Exact RX-M3x5.7: ruthex.de, EUR 8.99 / 100"),
    "BIOBUZZ-NECTAR": dict(url="https://www.andymark.com/products/biobuzz-scoring-elements", price=2.30, pack=1, sku="AndyMark am-5852",
                           note="Sold out when checked; full field set am-5850_FullSE $89.00"),
    "BIOBUZZ-POLLEN": dict(url="https://www.andymark.com/products/biobuzz-scoring-elements", price=2.00, pack=1, sku="AndyMark am-5851", note="Sold out when checked"),
}

# REX spacers / collars / couplers row: priced per kind
SPACER_KINDS = {
    "collar": dict(url=GB + "2910-series-aluminum-clamping-collar-8mm-rex-id-x-20mm-od-9mm-length/", price=4.99, pack=1, sku="2910-0920-4008"),
    "spacer": dict(url=GB + "1522-series-8mm-id-spacer-10mm-od-6mm-length-4-pack/", price=3.49, pack=4, sku="1522-0010-0060"),
    "coupler": dict(url=GB + "4007-series-hyper-coupler-8mm-rex-bore-to-8mm-rex-bore/", price=9.99, pack=1, sku="4007-4008-4008"),
}

# extra purchasable rows the CAD doesn't model
HYPER_HUB = dict(url=GB + "1310-series-hyper-hub-8mm-rex-bore/", price=7.99, pack=1, sku="1310-0016-4008")

# filament, $ per 1 kg spool (single-roll price, with spool), Bambu US store
FILAMENT = {
    "PETG-CF": dict(url="https://us.store.bambulab.com/products/petg-cf", per_kg=31.99, density=1.25),
    "TPU": dict(url="https://us.store.bambulab.com/products/tpu-95a-hf", per_kg=25.19, density=1.22, note="sale price; typical $41.99"),
    "clear": dict(url="https://us.store.bambulab.com/products/petg-translucent", per_kg=18.99, density=1.27),
    "PETG": dict(url="https://us.store.bambulab.com/products/petg-basic", per_kg=16.99, density=1.27),
    "AMS": dict(url="https://us.store.bambulab.com/products/pla-basic-filament", per_kg=17.99, density=1.24),
}
FILL = 0.55          # printed part mass ~ 55 % of solid (walls + 40 % infill)
CNC = dict(url="https://sendcutsend.com/materials/6061-aluminum/",
           note="Price by instant quote: upload the part's STEP from cad/STEP. 6061-T6 comes in .250 in (6.35 mm) and .187 in (4.75 mm)")


# ---- v12.6 second live-pricing pass (2026-11-16): cad/bom_prices_v126.json, keyed by exact SKU (url, price, pack, name,
# note). Only SKUs not already priced above are added; SVC-* entries are cutting services (quote only, price null).
import json as _json
import os as _os
_NEWP = _json.load(open(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "bom_prices_v126.json"), encoding="utf-8"))
for _k, _v in _NEWP.items():
    # screws/nuts: the old 2800-0004-0012 entry was a combined screw + nut kit priced as one unit; fasteners are now
    # separate rows per length, so the live per-SKU pack price wins for every goBILDA screw/nut SKU
    if _v.get("price") is not None and (_k not in BY_SKU or _k.startswith(("2800-", "2812-"))):
        BY_SKU[_k] = dict(url=_v["url"], price=_v["price"], pack=_v["pack"])
# v12.6 flywheel row now uses the real SKU (same live page/price as the spec-keyed entry above)
BY_SKU["3628-0032-0082"] = dict(url=GB + "steel-flywheel-82mm-diameter-152g-1651-g-cm/", price=15.99, pack=1)
# v12.6: plates stay metric (6 mm deck/side plates, 5 mm shear plates: the deck is pinned between the tower tops and
# the parts on it, so it cannot change thickness). SendCutSend/OSH Cut/Protocase stock only .187"/.250" aluminium;
# JLCCNC cuts 5052 in 5.0 and 6.0 mm and taps holes.
CNC = dict(url=_NEWP["SVC-JLCCNC"]["url"],
           note="quote only: upload cad/CNC/<plate>.dxf + .step to JLCCNC, 5052-H32 aluminium (5 mm / 6 mm metric stock), "
                "tap the holes listed in cad/CNC/HOLE_TABLE.md. Fallback: Xometry EU (6061 5/6 mm). Not SendCutSend (inch stock only)")
