# The industry standard is to create footpint wiht the pin 1 at the top left.
#
# Jclpcb expect to get rotation based on their position on the tape with the
# feed socket holes on the left side.
#
# This script rewrites the *pos.csv files to adjust rotations.
#

import glob
import csv
KICAD_HEADER="Ref,Val,Package,PosX,PosY,Rot,Side".split(",")
JCLPCB_HEADER="Designator,Val,Package,Mid X,Mid Y,Rotation,Layer".split(",")

PKG_IDX = KICAD_HEADER.index("Package")
ROT_IDX = KICAD_HEADER.index("Rot")

PACKAGE_TO_ROT = {
    "SOT-23_SS39ET":180,
    "SOIC-16_MCP3008T-I_SL":-90,
    "SOT-23-6":-90,
}

def fix_rotation(row):
    row = list(row)
    row[ROT_IDX] = (180. + 360. + float(row[ROT_IDX]) + PACKAGE_TO_ROT.get(row[PKG_IDX], 0)) % 360. - 180.
    return row
    

for path in glob.glob("boards/bandoneon/fabrication/*-pos.csv"):
    print(f"Read {path}")
    with open(path) as csvfile:
        all_rows = list(csv.reader(csvfile))
        header = all_rows[0]
        rows = all_rows[1:]
    
    if header != KICAD_HEADER:
        print("  Header is different from kicad format.")
        print(f"  Value: {header}")
        continue
    
    rows = [fix_rotation(r) for r in rows]
    with open(path, "w", newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(JCLPCB_HEADER)
        csvwriter.writerows(rows)
        
    print("  File rewritten.")