#!/usr/bin/env python3
"""Re-arrange a single-object PrusaSlicer 3MF into a regular grid of instances.

The input is expected to be a 3MF such as ``Bandolibre - Key - slice.3mf`` that
contains exactly one real mesh (object id 1) plus a number of component-wrapper
objects, one build <item> per printed instance.

This tool rebuilds the instance list: it keeps the mesh untouched and lays out N
instances on a regular grid, filling each platter left-to-right (columns, +X) then
front-to-back (rows, +Y).

Several platters are placed so they land on PrusaSlicer 2.9's multi-bed grid.
PrusaSlicer arranges beds in a square spiral around bed 0 and offsets bed k by
(grid_x*(bed_w+gap), grid_y*(bed_d+gap)), where gap = min(100, build-volume
diagonal * 0.3) == 100 mm for this printer. Matching that grid is what makes each
platter land cleanly on its own bed instead of being re-binned by PrusaSlicer.

Note: the buttons are tiny (~11 mm), so all 72 actually fit on one 250x220 bed if
you pass --plates 1 with enough rows/cols.
"""

import argparse
import re
import shutil
import zipfile
from pathlib import Path

MODEL_PATH = "3D/3dmodel.model"
CONFIG_PATH = "Metadata/Slic3r_PE_model.config"

# Rotation part of the original item transforms (180 deg about X) and the Z lift
# that drops the flipped mesh onto the bed. Kept identical to the source file.
ROT = "1 0 0 0 -1 0 0 0 -1"
Z = 7.0


def bed_grid_coords(index):
    """Map a bed index to integer grid coords using PrusaSlicer's square spiral.

    Mirrors index2grid_coords() in PrusaSlicer's MultipleBeds.cpp (positive
    quadrant only -- we never place beds at negative coords). Order is
    0:(0,0) 1:(1,0) 2:(0,1) 3:(1,1) 4:(2,0) 5:(2,1) 6:(0,2) ...
    """
    if index == 0:
        return 0, 0
    id_ = index + 1
    a = 1
    while (a + 1) * (a + 1) < id_:
        a += 1
    id_ -= a * a
    x = y = a
    if id_ <= a:
        y = id_ - 1
    else:
        x = id_ - a - 1
    return x, y


def grid_positions(plates, cols, rows, bed_w, bed_d, bed_gap):
    """Yield (x, y) bed coordinates, filling columns then rows then platters.

    Cells are spread evenly across the print area so the grid fills the platter,
    and each platter is offset onto PrusaSlicer's corresponding bed in the grid.
    """
    for p in range(plates):
        gx, gy = bed_grid_coords(p)
        ox = gx * (bed_w + bed_gap)
        oy = gy * (bed_d + bed_gap)
        for r in range(rows):           # front (y=0) to back
            for c in range(cols):       # left (x=0) to right
                x = ox + (c + 0.5) * bed_w / cols
                y = oy + (r + 0.5) * bed_d / rows
                yield x, y


def build_resources_and_build(n, positions):
    """Return (component-object XML, build XML) for n instances of object 1."""
    objs = []
    items = [f'  <item objectid="1" transform="{ROT} '
             f'{positions[0][0]:.6f} {positions[0][1]:.6f} {Z}" printable="1"/>']
    # object 1 already exists (the mesh); instances 2..n are component wrappers.
    for i in range(2, n + 1):
        objs.append(
            f'  <object id="{i}" type="model">\n'
            f'   <components>\n'
            f'    <component objectid="1"/>\n'
            f'   </components>\n'
            f'  </object>')
        x, y = positions[i - 1]
        items.append(f'  <item objectid="{i}" transform="{ROT} '
                     f'{x:.6f} {y:.6f} {Z}" printable="1"/>')
    return "\n".join(objs), "\n".join(items)


def rewrite_model(text, n, positions):
    obj1 = re.search(r'<object id="1".*?</object>', text, re.S)
    if not obj1:
        raise ValueError("could not find <object id=\"1\"> mesh in model")

    comp_objs, build_items = build_resources_and_build(n, positions)
    resources = (f' <resources>\n{obj1.group(0)}\n{comp_objs}\n </resources>'
                 if comp_objs else f' <resources>\n{obj1.group(0)}\n </resources>')

    text = re.sub(r' *<resources>.*?</resources>', resources, text, count=1, flags=re.S)
    text = re.sub(r' *<build>.*?</build>',
                  f' <build>\n{build_items}\n </build>', text, count=1, flags=re.S)
    return text


def rewrite_config(text, n):
    return re.sub(r'(<object id="1" instances_count=")\d+(")', rf'\g<1>{n}\g<2>', text, count=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    here = Path(__file__).resolve().parent
    ap.add_argument("-i", "--input", default=here / "Bandolibre - Key - slice.3mf", type=Path)
    ap.add_argument("-o", "--output", default=here / "Bandolibre - Key - 72.3mf", type=Path)
    ap.add_argument("--plates", type=int, default=3)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--bed-w", type=float, default=250.0)
    ap.add_argument("--bed-d", type=float, default=220.0)
    ap.add_argument("--bed-gap", type=float, default=100.0,
                    help="gap between beds in PrusaSlicer's grid (default 100, "
                         "matching min(100, build-volume diagonal * 0.3))")
    args = ap.parse_args()

    n = args.plates * args.cols * args.rows
    positions = list(grid_positions(args.plates, args.cols, args.rows,
                                    args.bed_w, args.bed_d, args.bed_gap))
    assert len(positions) == n

    with zipfile.ZipFile(args.input) as zin:
        names = zin.namelist()
        model = zin.read(MODEL_PATH).decode("utf-8")
        config = zin.read(CONFIG_PATH).decode("utf-8") if CONFIG_PATH in names else None
        others = {nm: zin.read(nm) for nm in names if nm not in (MODEL_PATH, CONFIG_PATH)}

    model = rewrite_model(model, n, positions)
    if config is not None:
        config = rewrite_config(config, n)

    tmp = args.output.with_suffix(".tmp.3mf")
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as zout:
        # Preserve original file order; [Content_Types].xml should stay first.
        for nm in names:
            if nm == MODEL_PATH:
                zout.writestr(nm, model)
            elif nm == CONFIG_PATH and config is not None:
                zout.writestr(nm, config)
            else:
                zout.writestr(nm, others[nm])
    shutil.move(tmp, args.output)

    print(f"Wrote {args.output} with {n} instances "
          f"({args.plates} platters x {args.cols} cols x {args.rows} rows).")


if __name__ == "__main__":
    main()
