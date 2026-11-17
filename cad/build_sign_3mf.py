"""
Merge the v12 sign plate + numeral STLs (exported by cad/export_v12.py) into multi-colour Bambu AMS .3mf files:
  cad/3MF/v12/robot_sign_plate_149_red.3mf  (red plate, white numerals)
  cad/3MF/v12/robot_sign_plate_149_blue.3mf (blue plate, white numerals)
Run locally:  python cad/build_sign_3mf.py
"""
import glob
import os
import struct
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
PARTS = os.path.join(HERE, "3MF", "v12", "_sign_parts")
OUT = os.path.join(HERE, "3MF", "v12")


def read_stl(path):
    data = open(path, "rb").read()
    tris = []
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = []
        for line in data.decode("ascii", "ignore").splitlines():
            t = line.strip().split()
            if t and t[0] == "vertex":
                v.append(tuple(float(x) for x in t[1:4]))
                if len(v) == 3:
                    tris.append(tuple(v))
                    v = []
    else:
        n = struct.unpack("<I", data[80:84])[0]
        for i in range(n):
            o = 84 + i * 50 + 12
            f = struct.unpack("<9f", data[o:o + 36])
            tris.append((f[0:3], f[3:6], f[6:9]))
    return tris


def mesh_xml(tris, oid, pid, pindex):
    verts, idx, out_t = [], {}, []
    for t in tris:
        ids = []
        for p in t:
            k = (round(p[0], 4), round(p[1], 4), round(p[2], 4))
            if k not in idx:
                idx[k] = len(verts)
                verts.append(k)
            ids.append(idx[k])
        if len(set(ids)) == 3:
            out_t.append(ids)
    vx = "".join(f'<vertex x="{a}" y="{b}" z="{c}"/>' for a, b, c in verts)
    tx = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in out_t)
    return f'<object id="{oid}" type="model" pid="{pid}" pindex="{pindex}"><mesh><vertices>{vx}</vertices><triangles>{tx}</triangles></mesh></object>', len(out_t)


def build(color_name, rgb):
    files = sorted(glob.glob(os.path.join(PARTS, "*.stl")))
    if not files:
        raise SystemExit("no sign STLs - run cad/export_v12.py in Fusion first")
    objs, comps, ntri = [], [], 0
    oid = 2
    for f in files:
        plate = "_plate_" in os.path.basename(f)
        x, n = mesh_xml(read_stl(f), oid, 1, 0 if plate else 1)
        objs.append(x)
        comps.append(f'<component objectid="{oid}"/>')
        ntri += n
        oid += 1
    model = ('<?xml version="1.0" encoding="UTF-8"?>'
             '<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             f'<metadata name="Title">FTC 149 robot sign ({color_name}) - R401-R403 compliant 174x78 mm (7.25 mm visible border), 57.5 mm numerals</metadata>'
             '<resources><basematerials id="1">'
             f'<base name="{color_name} PLA" displaycolor="{rgb}"/><base name="White PLA" displaycolor="#FAFAFAFF"/>'
             '</basematerials>' + "".join(objs) +
             f'<object id="{oid}" type="model"><components>{"".join(comps)}</components></object>'
             f'</resources><build><item objectid="{oid}"/></build></model>')
    ct = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    path = os.path.join(OUT, f"robot_sign_plate_149_{color_name.lower()}.3mf")
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", ct)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)
    return path, len(files), ntri


if __name__ == "__main__":
    for nm, rgb in (("Red", "#DC2626FF"), ("Blue", "#2563EBFF")):
        print(build(nm, rgb))
