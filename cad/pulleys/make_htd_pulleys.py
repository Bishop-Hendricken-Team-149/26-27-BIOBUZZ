#!/usr/bin/env python3
"""
make_htd_pulleys.py - printable HTD-3M / HTD-5M timing pulleys with a goBILDA
8 mm REX (rounded-hex) bore, two radial M3 set screws through REX flats and
captive M3 nut pockets.  FTC team 26-27 BIOBUZZ robot, CAD v12.

Run (from any folder; needs uv):
    uv run -q --with numpy --with trimesh --with manifold3d --with shapely \
        --with lxml --with networkx --with scipy python cad/pulleys/make_htd_pulleys.py

Outputs (repo-relative):
    cad/3MF/v12/pulleys/<name>.3mf    one object per pulley type, lying on a flange face (Z = axis)
    cad/STL/v12/pulleys/<name>.stl    same mesh
    cad/pulleys/verification.json     trimesh measurements of every written file

----------------------------------------------------------------------------
TOOTH (GROOVE) PROFILE SOURCE
----------------------------------------------------------------------------
The groove shapes are the HTD 3mm and HTD 5mm curvilinear tooth profiles of
the "Parametric Pulley - lots of tooth profiles" OpenSCAD library by droftarts
(Thingiverse thing:16627, Jan 2012, CC BY-SA 3.0), point lists copied verbatim
from modules tooth_profile_HTD_3mm / tooth_profile_HTD_5mm as republished at
    https://github.com/rbuckland/openscad.parametric-pulley
    (Pulley_T-MXL-XL-HTD-GT2_N-tooth.scad, parametric-pulley-generator.scad).
droftarts drew these from the Gates HTD (High Torque Drive) belt tooth data;
nominal values used by that library (and here):
    HTD 3M: pitch 3.000 mm, pitch-line offset 0.381 mm  (OD = PD - 0.762 mm),
            groove depth 1.289 mm, groove width at OD 2.270 mm
    HTD 5M: pitch 5.000 mm, pitch-line offset 0.5715 mm (OD = PD - 1.143 mm),
            groove depth 2.199 mm, groove width at OD 3.781 mm
For comparison, Gates lists the HTD belt tooth height as ~1.14 mm (3M) and
~2.06 mm (5M), so the groove is ~0.14 mm deeper than the belt tooth, as on a
Gates HTD sprocket.  Placement follows the droftarts generator: the two groove
corners (x = +-w/2, y = 0) sit on the OD circle, y points toward the axis.

Printing clearance: the groove polygon is offset outward (normal offset,
round joins) by GROOVE_CLEARANCE, i.e. root diameter -2c, groove width +2c.
----------------------------------------------------------------------------
"""
from __future__ import annotations

import json
import math
import os
import sys
import zipfile
from dataclasses import dataclass, asdict
from pathlib import Path

import numpy as np
import manifold3d as m3d
import trimesh
from shapely.geometry import Polygon, Point, LineString
from shapely.geometry.polygon import orient
from shapely.ops import unary_union
from shapely import affinity

# ------------------------------------------------------------------ parameters
GROOVE_CLEARANCE = 0.12      # mm, normal (radial) offset added to every groove
BORE_CLEARANCE = 0.15        # mm, added to both REX flat radius and corner radius
REX_FLAT_R = 3.50            # goBILDA 8 mm REX: centre -> flat (7.00 mm across flats)
REX_CORNER_R = 4.00          # corners trimmed to this radius
SCREW_COUNT = 2              # radial M3 set screws
SCREW_ANGLES = (0.0, 180.0)  # deg; must be REX flat normals (multiples of 60) -> opposite flats
SET_SCREW_PILOT_D = 2.5      # mm, thread-forming pilot (used when no captive nut)
SET_SCREW_CLEAR_D = 3.2      # mm, M3 clearance used when a captive nut carries the thread
USE_CAPTIVE_NUT = True       # captive M3 nut pocket on each screw
NUT_AF = 5.7                 # mm, pocket across flats (M3 nut 5.5 + clearance)
NUT_THK = 2.6                # mm, pocket thickness (M3 nut 2.4 + clearance)
NUT_INNER_R = 5.4            # mm, radius of the nut pocket face nearest the bore
CIRCLE_SEGS = 720            # facets on the OD / flange circles
PLATE = 256.0                # mm, printer bed (Bambu X1/P1 class)

REPO = Path(__file__).resolve().parents[2]
OUT_3MF = REPO / "cad" / "3MF" / "v12" / "pulleys"
OUT_STL = REPO / "cad" / "STL" / "v12" / "pulleys"
OUT_JSON = REPO / "cad" / "pulleys" / "verification.json"

# droftarts profiles (x = across groove, y = depth, +y toward the axis)
HTD3_PTS = [(-1.135062, -0.5), (-1.135062, 0), (-1.048323, 0.015484), (-0.974284, 0.058517), (-0.919162, 0.123974),
            (-0.889176, 0.206728), (-0.81721, 0.579614), (-0.800806, 0.653232), (-0.778384, 0.72416), (-0.750244, 0.792137),
            (-0.716685, 0.856903), (-0.678005, 0.918199), (-0.634505, 0.975764), (-0.586483, 1.029338), (-0.534238, 1.078662),
            (-0.47807, 1.123476), (-0.418278, 1.16352), (-0.355162, 1.198533), (-0.289019, 1.228257), (-0.22015, 1.25243),
            (-0.148854, 1.270793), (-0.07543, 1.283087), (-0.000176, 1.28905), (0.075081, 1.283145), (0.148515, 1.270895),
            (0.219827, 1.252561), (0.288716, 1.228406), (0.354879, 1.19869), (0.418018, 1.163675), (0.477831, 1.123623),
            (0.534017, 1.078795), (0.586276, 1.029452), (0.634307, 0.975857), (0.677809, 0.91827), (0.716481, 0.856953),
            (0.750022, 0.792167), (0.778133, 0.724174), (0.800511, 0.653236), (0.816857, 0.579614), (0.888471, 0.206728),
            (0.919014, 0.123974), (0.974328, 0.058517), (1.048362, 0.015484), (1.135062, 0), (1.135062, -0.5)]
HTD5_PTS = [(-1.89036, -0.75), (-1.89036, 0), (-1.741168, 0.02669), (-1.61387, 0.100806), (-1.518984, 0.21342),
            (-1.467026, 0.3556), (-1.427162, 0.960967), (-1.398568, 1.089602), (-1.359437, 1.213531), (-1.310296, 1.332296),
            (-1.251672, 1.445441), (-1.184092, 1.552509), (-1.108081, 1.653042), (-1.024167, 1.746585), (-0.932877, 1.832681),
            (-0.834736, 1.910872), (-0.730271, 1.980701), (-0.62001, 2.041713), (-0.504478, 2.09345), (-0.384202, 2.135455),
            (-0.259708, 2.167271), (-0.131524, 2.188443), (-0.000176, 2.198511), (0.131296, 2.188504), (0.259588, 2.167387),
            (0.384174, 2.135616), (0.504527, 2.093648), (0.620123, 2.04194), (0.730433, 1.980949), (0.834934, 1.911132),
            (0.933097, 1.832945), (1.024398, 1.746846), (1.108311, 1.653291), (1.184308, 1.552736), (1.251865, 1.445639),
            (1.310455, 1.332457), (1.359552, 1.213647), (1.39863, 1.089664), (1.427162, 0.960967), (1.467026, 0.3556),
            (1.518984, 0.21342), (1.61387, 0.100806), (1.741168, 0.02669), (1.89036, 0), (1.89036, -0.75)]

PROFILES = {
    "3M": dict(pitch=3.0, plo=0.381, depth=1.289, width=2.270, pts=HTD3_PTS),
    "5M": dict(pitch=5.0, plo=0.5715, depth=2.199, width=3.781, pts=HTD5_PTS),
}


@dataclass
class PulleySpec:
    name: str
    profile: str
    teeth: int
    belt_width: float
    total_width: float
    flange_t: float
    flange_extra_r: float
    qty: int
    replaces: tuple

    @property
    def pd(self):
        return self.teeth * PROFILES[self.profile]["pitch"] / math.pi

    @property
    def od(self):
        return self.pd - 2 * PROFILES[self.profile]["plo"]

    @property
    def flange_d(self):
        return self.od + 2 * self.flange_extra_r


PULLEYS = [
    PulleySpec("htd3m_32t_6mm_rex8", "3M", 32, 6.0, 8.5, 0.9, 1.5, 2,
               ("magazine_b2_m8_pulley_32t", "magazine_b2_j2_pulley_32t")),
    PulleySpec("htd3m_33t_6mm_rex8", "3M", 33, 6.0, 8.5, 0.9, 1.5, 4,
               ("magazine_b1_m8_pulley_33t", "magazine_b1_gecko1_pulley_33t",
                "magazine_b3_j2_pulley_33t", "magazine_b3_gecko2_pulley_33t")),
    PulleySpec("htd5m_20t_9mm_rex8", "5M", 20, 9.0, 12.0, 1.0, 2.2, 6,
               ("intake_stage1_m7_pulley_20t", "intake_stage1_pivot_pulley_20t",
                "intake_stage2_pivot_pulley_20t", "intake_stage2_floating_pulley_20t",
                "shooter_left_m5_pulley_20t", "shooter_left_front_wheel_pulley_20t")),
    PulleySpec("htd5m_32t_9mm_rex8", "5M", 32, 9.0, 12.0, 1.0, 2.2, 1,
               ("shooter_right_m6_pulley_32t",)),
]


# ------------------------------------------------------------------ 2D helpers
def circle(r, n=CIRCLE_SEGS):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return Polygon(np.c_[r * np.cos(a), r * np.sin(a)])


def groove_polygon(prof, od, angle_deg, clearance):
    """One groove (the cutting tool) centred on angle_deg, in global XY."""
    pts = np.array(prof["pts"], float)
    pts[pts[:, 1] < 0, 1] = -3.0                       # extend the tool well past the OD
    g = Polygon(pts).buffer(clearance, quad_segs=8, join_style="round")
    R = od / 2
    d = math.sqrt(R * R - (prof["width"] / 2) ** 2)    # droftarts tooth_distance_from_centre
    # local (x, y) -> radial distance d - y along u, tangential x along t
    a = math.radians(angle_deg)
    u = np.array([math.cos(a), math.sin(a)])
    t = np.array([-math.sin(a), math.cos(a)])
    xy = np.array(g.exterior.coords)
    glob = np.outer(d - xy[:, 1], u) + np.outer(xy[:, 0], t)
    return Polygon(glob)


def toothed_outline(spec, clearance=GROOVE_CLEARANCE):
    prof = PROFILES[spec.profile]
    # groove 0 is centred on the first set-screw angle so the hole exits in a root
    grooves = [groove_polygon(prof, spec.od, SCREW_ANGLES[0] + i * 360.0 / spec.teeth, clearance)
               for i in range(spec.teeth)]
    return circle(spec.od / 2).difference(unary_union(grooves))


def rex_bore(clear=BORE_CLEARANCE):
    rf, rc = REX_FLAT_R + clear, REX_CORNER_R + clear
    # flats normal to 0, 60, 120 ... deg  -> set screws at 0/180 are perpendicular to flats
    hex_r = rf / math.cos(math.radians(30))
    hexagon = Polygon([(hex_r * math.cos(math.radians(30 + 60 * k)),
                        hex_r * math.sin(math.radians(30 + 60 * k))) for k in range(6)])
    return hexagon.intersection(circle(rc, 360))


def cs(poly):
    poly = orient(poly, 1.0)
    rings = [list(map(tuple, np.array(poly.exterior.coords)[:-1]))]
    rings += [list(map(tuple, np.array(r.coords)[:-1])) for r in poly.interiors]
    return m3d.CrossSection(rings, m3d.FillRule.EvenOdd)


def extrude(poly, z0, h):
    return m3d.Manifold.extrude(cs(poly), h).translate((0, 0, z0))


# ------------------------------------------------------------------ 3D build
def frame(angle_deg, z, r0):
    """3x4 matrix: local X = tangent, local Y = +Z (axial), local Z = radial out, origin at r0 on the ray."""
    a = math.radians(angle_deg)
    u = np.array([math.cos(a), math.sin(a), 0.0])
    t = np.array([-math.sin(a), math.cos(a), 0.0])
    zax = np.array([0.0, 0.0, 1.0])
    R = np.c_[t, zax, u]
    o = u * r0 + zax * z
    return np.c_[R, o]


def build_pulley(spec):
    W, tf = spec.total_width, spec.flange_t
    band_h = W - 2 * tf
    body = extrude(circle(spec.flange_d / 2), 0, tf)
    body = body + extrude(toothed_outline(spec), tf, band_h)
    body = body + extrude(circle(spec.flange_d / 2), W - tf, tf)
    body = body - extrude(rex_bore(), -1, W + 2)

    zmid = W / 2
    hole_d = SET_SCREW_CLEAR_D if USE_CAPTIVE_NUT else SET_SCREW_PILOT_D
    for ang in SCREW_ANGLES[:SCREW_COUNT]:
        # radial hole from inside the bore to beyond the OD
        hole = m3d.Manifold.cylinder(spec.flange_d, hole_d / 2, hole_d / 2, 48)  # along local Z
        body = body - hole.transform(frame(ang, zmid, 1.0))
        if USE_CAPTIVE_NUT:
            ac = NUT_AF / math.cos(math.radians(30))
            # hexagon: flats across local X (tangential), corners along local Y (axial)
            hexa = Polygon([(ac / 2 * math.cos(math.radians(90 + 60 * k)),
                             ac / 2 * math.sin(math.radians(90 + 60 * k))) for k in range(6)])
            # insertion slot from the nut centre to the top face (local +Y = +Z), merged in 2D so the
            # coplanar slot walls / hex flats leave no pinched vertices
            pocket = unary_union([hexa, Polygon([(-NUT_AF / 2, 0), (NUT_AF / 2, 0), (NUT_AF / 2, W), (-NUT_AF / 2, W)])])
            pocket = pocket.simplify(1e-6)
            body = body - m3d.Manifold.extrude(cs(pocket), NUT_THK).transform(frame(ang, zmid, NUT_INNER_R))
    return body


def to_trimesh(man):
    mesh = man.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    f = np.asarray(mesh.tri_verts)
    return trimesh.Trimesh(v, f, process=True)


# ------------------------------------------------------------------ 3MF writer
def write_3mf(path, tm, name):
    v, f = tm.vertices, tm.faces
    verts = "\n".join(f'<vertex x="{x:.5f}" y="{y:.5f}" z="{z:.5f}"/>' for x, y, z in v)
    tris = "\n".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in f)
    model = f'''<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
<metadata name="Title">{name}</metadata>
<metadata name="Designer">26-27 BIOBUZZ make_htd_pulleys.py</metadata>
<resources>
<object id="1" name="{name}" type="model">
<mesh>
<vertices>
{verts}
</vertices>
<triangles>
{tris}
</triangles>
</mesh>
</object>
</resources>
<build>
<item objectid="1" transform="1 0 0 0 1 0 0 0 1 {PLATE/2:.3f} {PLATE/2:.3f} 0"/>
</build>
</model>'''
    ct = ('<?xml version="1.0" encoding="UTF-8"?>\n<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
            'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", ct)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)


# ------------------------------------------------------------------ verification
def slice_polys(tm, z):
    """Planar section at height z as shapely polygons in world XY (rings nested by containment)."""
    sec = tm.section(plane_origin=(0, 0, z), plane_normal=(0, 0, 1))
    rings = [Polygon(d[:, :2]) for d in sec.discrete if len(d) >= 4]
    rings = [r if r.is_valid else r.buffer(0) for r in rings]
    rings.sort(key=lambda r: -r.area)
    depth = [sum(1 for o in rings[:i] if o.contains(r.representative_point())) for i, r in enumerate(rings)]
    out = []
    for i, r in enumerate(rings):
        if depth[i] % 2:
            continue
        holes = [rings[j] for j in range(i + 1, len(rings))
                 if depth[j] == depth[i] + 1 and r.contains(rings[j].representative_point())]
        out.append(Polygon(r.exterior.coords, [h.exterior.coords for h in holes]))
    return out


def count_grooves(poly, r_od, r_root):
    ring = np.array(poly.exterior.coords)
    ang = np.linspace(0, 2 * np.pi, 7200, endpoint=False)
    rr = np.empty_like(ang)
    for i, a in enumerate(ang):
        ray = LineString([(0, 0), (2 * r_od * math.cos(a), 2 * r_od * math.sin(a))])
        inter = ray.intersection(poly.exterior)
        pts = [inter] if inter.geom_type == "Point" else list(getattr(inter, "geoms", []))
        rr[i] = max(math.hypot(p.x, p.y) for p in pts if p.geom_type == "Point")
    thr = 0.5 * (r_od + r_root)
    low = rr < thr
    # count falling edges on the circular sequence
    n = int(np.sum(low & ~np.roll(low, 1)))
    return n, float(rr.min()), float(rr.max())


def bore_af(poly):
    hole = max(poly.interiors, key=lambda r: Polygon(r).area)  # only interior ring on a clean slice
    c = np.array(hole.coords)
    widths = []
    for a in np.radians(np.arange(0, 180, 0.25)):
        p = c @ np.array([math.cos(a), math.sin(a)])
        widths.append(p.max() - p.min())
    corner_d = 2 * np.max(np.hypot(c[:, 0], c[:, 1]))
    return float(min(widths)), float(corner_d)


def verify(spec, path3mf, pathstl):
    res = {"file_3mf": str(path3mf), "file_stl": str(pathstl)}
    tm = trimesh.load(path3mf, force="mesh")
    tm.apply_translation(-np.r_[tm.bounds.mean(axis=0)[:2], tm.bounds[0][2]])   # centre XY, z0 = 0
    stl = trimesh.load(pathstl, force="mesh")
    res["watertight"] = bool(tm.is_watertight)
    res["stl_watertight"] = bool(stl.is_watertight)
    res["bodies"] = len(tm.split(only_watertight=False))
    res["volume_mm3"] = round(float(tm.volume), 1)
    res["mass_g_petgcf_1.30"] = round(float(tm.volume) * 1.30e-3, 1)
    ext = tm.bounds[1] - tm.bounds[0]
    res["bbox_mm"] = [round(float(x), 3) for x in ext]
    W, tf = spec.total_width, spec.flange_t
    # toothed band slice away from the radial holes (holes at mid-width, r = 1.6)
    z_band = W / 2 + (W / 2 - tf) * 0.6
    band = max(slice_polys(tm, z_band), key=lambda p: p.area)
    prof = PROFILES[spec.profile]
    # groove corners sit on the OD, so the root is at d - depth - c with d = sqrt(R^2 - (w/2)^2)
    r_root_nom = math.sqrt((spec.od / 2) ** 2 - (prof["width"] / 2) ** 2) - prof["depth"] - GROOVE_CLEARANCE
    n, rmin, rmax = count_grooves(band, spec.od / 2, r_root_nom)
    res["tooth_count_slice_z"] = round(z_band, 3)
    res["tooth_count"] = n
    res["od_measured"] = round(2 * rmax, 3)
    res["root_d_measured"] = round(2 * rmin, 3)
    fl = max(slice_polys(tm, tf / 2), key=lambda p: p.area)
    c = np.array(fl.exterior.coords)
    res["flange_od_measured"] = round(float(2 * np.hypot(c[:, 0], c[:, 1]).max()), 3)
    af, cd = bore_af(fl)
    res["bore_af_measured"] = round(af, 3)
    res["bore_corner_d_measured"] = round(cd, 3)
    res["width_measured"] = round(float(ext[2]), 3)
    res["fits_256_plate"] = bool(ext[0] <= PLATE and ext[1] <= PLATE and ext[2] <= 256)
    # qty copies on one plate, 5 mm gap, square grid
    pitch = ext[0] + 5
    per_row = int((PLATE - 10 + 5) // pitch)
    res["qty_fit_one_plate"] = bool(per_row >= 1 and math.ceil(spec.qty / per_row) * pitch - 5 <= PLATE - 10)
    res["nominal"] = {"pd": round(spec.pd, 3), "od": round(spec.od, 3), "flange_d": round(spec.flange_d, 3),
                      "root_d": round(2 * r_root_nom, 3), "bore_af": round(2 * (REX_FLAT_R + BORE_CLEARANCE), 3),
                      "bore_corner_d": round(2 * (REX_CORNER_R + BORE_CLEARANCE), 3), "width": W}
    # mid-width check: set screw hole present on both sides (bore connects to outside)
    mid = slice_polys(tm, W / 2)
    res["midwidth_slice_regions"] = len(mid)
    return res


def main():
    OUT_3MF.mkdir(parents=True, exist_ok=True)
    OUT_STL.mkdir(parents=True, exist_ok=True)
    report = {"params": {k: v for k, v in globals().items() if k.isupper() and isinstance(v, (int, float, bool, tuple))},
              "pulleys": []}
    report["params"].pop("PROFILES", None)
    for spec in PULLEYS:
        man = build_pulley(spec)
        assert man.status() == m3d.Error.NoError, man.status()
        tm = to_trimesh(man)
        p3 = OUT_3MF / f"{spec.name}.3mf"
        ps = OUT_STL / f"{spec.name}.stl"
        write_3mf(p3, tm, spec.name)
        tm.export(ps)
        r = verify(spec, p3, ps)
        r.update({"name": spec.name, "qty": spec.qty, "teeth": spec.teeth, "profile": spec.profile,
                  "belt_width": spec.belt_width, "replaces": list(spec.replaces)})
        ok = (r["watertight"] and r["bodies"] == 1 and r["tooth_count"] == spec.teeth
              and abs(r["od_measured"] - spec.od) < 0.05 and abs(r["flange_od_measured"] - spec.flange_d) < 0.05
              and abs(r["width_measured"] - spec.total_width) < 0.01
              and abs(r["root_d_measured"] - r["nominal"]["root_d"]) < 0.05
              and abs(r["bore_af_measured"] - 2 * (REX_FLAT_R + BORE_CLEARANCE)) < 0.02 and r["fits_256_plate"])
        r["PASS"] = bool(ok)
        report["pulleys"].append(r)
        print(json.dumps({k: r[k] for k in ("name", "PASS", "watertight", "bodies", "volume_mm3", "bbox_mm",
                                            "od_measured", "root_d_measured", "flange_od_measured",
                                            "width_measured", "tooth_count", "bore_af_measured",
                                            "bore_corner_d_measured", "qty_fit_one_plate")}))
    OUT_JSON.write_text(json.dumps(report, indent=2, default=list))
    if not all(p["PASS"] for p in report["pulleys"]):
        sys.exit("verification FAILED")
    print("all pulleys PASS ->", OUT_JSON)


if __name__ == "__main__":
    main()
