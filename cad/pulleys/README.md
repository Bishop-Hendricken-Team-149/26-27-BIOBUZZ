# HTD timing pulleys (v12), printed, goBILDA 8 mm REX bore

Generator: `cad/pulleys/make_htd_pulleys.py`. It is parametric and uses manifold3d and shapely, then checks every file it writes with trimesh. The measurements go to `cad/pulleys/verification.json`.

```bash
uv run -q --with numpy --with trimesh --with manifold3d --with shapely \
    --with lxml --with networkx --with scipy python cad/pulleys/make_htd_pulleys.py
```

Each 3MF holds one object. It lies on a flange face (Z = pulley axis), with the nut-slot openings facing up. The STL files hold the same mesh.

| Pulley | 3MF / STL | Qty | Belt | Replaces CAD bodies |
|---|---|---|---|---|
| HTD-3M 32T | `cad/3MF/v12/pulleys/htd3m_32t_6mm_rex8.3mf`, `cad/STL/v12/pulleys/htd3m_32t_6mm_rex8.stl` | 2 | HTD 3M, 6 mm wide | magazine_b2_m8_pulley_32t, magazine_b2_j2_pulley_32t |
| HTD-3M 33T | `.../htd3m_33t_6mm_rex8.3mf` / `.stl` | 4 | HTD 3M, 6 mm wide | magazine_b1_m8_pulley_33t, magazine_b1_gecko1_pulley_33t, magazine_b3_j2_pulley_33t, magazine_b3_gecko2_pulley_33t |
| HTD-5M 20T | `.../htd5m_20t_9mm_rex8.3mf` / `.stl` | 6 | HTD 5M, 9 mm wide | intake_stage1_m7_pulley_20t, intake_stage1_pivot_pulley_20t, intake_stage2_pivot_pulley_20t, intake_stage2_floating_pulley_20t, shooter_left_m5_pulley_20t, shooter_left_front_wheel_pulley_20t |
| HTD-5M 32T | `.../htd5m_32t_9mm_rex8.3mf` / `.stl` | 1 | HTD 5M, 9 mm wide | shooter_right_m6_pulley_32t |

## Geometry (nominal, as built)

| | PD | OD | Root dia | Flange dia | Width (flanges) | Volume | Mass at 1.30 g/cm³ |
|---|---|---|---|---|---|---|---|
| 3M 32T | 30.558 | 29.796 | 26.89 | 32.796 | 8.5 (2 × 0.9) | 4956 mm³ | 6.4 g |
| 3M 33T | 31.513 | 30.751 | 27.85 | 33.751 | 8.5 (2 × 0.9) | 5326 mm³ | 6.9 g |
| 5M 20T | 31.831 | 30.688 | 25.82 | 35.088 | 12.0 (2 × 1.0) | 7096 mm³ | 9.2 g |
| 5M 32T | 50.930 | 49.787 | 45.01 | 54.187 | 12.0 (2 × 1.0) | 20936 mm³ | 27.2 g |

- **Tooth profile:** droftarts "Parametric Pulley" HTD 3mm / 5mm curvilinear profiles (Thingiverse 16627, CC BY-SA 3.0, from Gates HTD data). The groove is 1.289 / 2.199 mm deep and 2.270 / 3.781 mm wide at the OD. OD = PD − 0.762 mm (3M) or PD − 1.143 mm (5M). The script header gives the full source.
- **Groove clearance:** `GROOVE_CLEARANCE = 0.12` mm normal offset. This makes the root 0.24 mm smaller in diameter and each groove 0.24 mm wider.
- **Bore:** REX rounded hex, made from a hexagon with flats at r = 3.50 intersected with a circle of r = 4.00. `BORE_CLEARANCE = 0.15` is added to both, so the printed bore is 7.30 mm across flats and 8.30 mm across the trimmed corners.
- **Set screws:** 2 × M3, on opposite REX flats (0° / 180°). On a hexagon, flat normals are 60° apart, so 90° apart is not possible. Each screw runs radially at mid-width through the toothed band, exiting at a groove root. That keeps the flanges out of the way.
  - On the 33T pulley the second screw exits on a land, because 33 is odd.
- **Captive nut:** M3 nut pocket, 5.7 mm across flats × 2.6 mm thick, sitting at r = 5.4–8.0 mm. That leaves 1.75 mm of plastic between the pocket and the bore flat.
  - The pocket has flats facing tangentially and corners facing axially. Its slot is open to the top face (as printed), so the nut drops in from above.
  - The 8.5 mm wide 3M pulleys are wide enough for it: the nut pocket spans z = 0.96–7.54, so all four types use captive nuts.
- **Screw hole diameter:** with a captive nut, the radial hole is 3.2 mm clearance (`SET_SCREW_CLEAR_D`), not a 2.5 mm pilot.
  - Reason: a 2.5 mm pilot in series with a nut would cut plastic threads out of phase with the nut, and the screw would bind.
  - For a nut-less, thread-forming variant, set `USE_CAPTIVE_NUT = False`. The hole then becomes a 2.5 mm pilot (`SET_SCREW_PILOT_D`).
- **Screw:** M3 × 8 cup-point grub screw, driven with a 1.5 mm hex key through the radial hole. When tight it sits below the tooth root.

## Print settings

- PETG-CF, hardened 0.4 mm nozzle, **100 % infill**.
  - At this size, 4–5 walls plus solid infill amounts to the same thing.
- Layer height 0.12–0.16 mm, for tooth fidelity on the top flange underside and on the nut slot.
  - The grooves are vertical prisms, so their fidelity comes from XY accuracy. Use arachne walls and keep "detect thin walls" on. The 3M teeth are about 1 mm wide at mid-depth and about 0.4 mm flat at the tip.
- Place a flange face down, as the file is oriented. **No supports.**
- The nut slots open upward, and the bore and grooves are vertical.
- The 3.2 mm radial holes are small horizontal holes and bridge cleanly.
- **Top flange overhang:** the top flange's underside is an unsupported horizontal overhang.
  - It sticks out 1.5 mm (3M) or 2.2 mm (5M) beyond the OD, and bridges across each groove mouth (about 2.5 mm wide on 3M, 4.0 mm on 5M).
  - PETG-CF handles this with overhang cooling and slow overhang speed. Any droop is on the belt-side face of the flange: deburr it if the belt edge rubs.
- Turn on elephant-foot compensation (about 0.1–0.15 mm), or the bottom of the bore and the bottom flange edge will be tight.
- Seam: aligned or rear, not on the teeth.

## Test procedure (do this before printing the full set)

1. Print **one HTD-5M 20T** first.
2. **REX fit:** slide it onto a goBILDA 8 mm REX shaft. It should be a light push fit with no visible rotational slop.
   - Too tight: raise `BORE_CLEARANCE` by 0.05.
   - Loose or rattling: lower it by 0.05.
3. **Nut and screw:** drop an M3 nut into each slot. It should sit flush and its hole should line up with the radial hole. Run in an M3 × 8 grub screw and tighten it onto the REX flat.
   - If the nut will not seat, the pocket was printed undersize: raise `NUT_AF` or `NUT_THK` by 0.1.
4. **Belt engagement:** wrap an HTD-5M 9 mm belt about 180° round the pulley and pull it tight by hand. The teeth should seat fully, without riding on the lands, and the belt should not click or skip when you rotate it under tension.
   - Belt rides high or the teeth bind: raise `GROOVE_CLEARANCE` (up to 0.15–0.18).
   - Visible backlash: lower it (to 0.08–0.10).
5. Print a 3M 33T and repeat steps 2–4 with an HTD-3M 6 mm belt. Rerun the generator if you changed anything; it re-verifies every file.
6. Print the full quantities: 2 × 3M 32T, 4 × 3M 33T, 6 × 5M 20T, 1 × 5M 32T.
   - Each type's full quantity fits on one 256 mm plate.
