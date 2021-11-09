
import json
with open("waypoints.json", "r") as f:
     wps = json.load(f)
 
import csv
with open ("waypoints.csv", "w") as out:
     writer=csv.DictWriter(out, ['x', 'y', 'z', 'speed', 'index'])
     writer.writeheader()
     for wp in wps:
             writer.writerow(wp)

import numpy as np

y = np.linspace(-25.2, -17.5, 30)

x = (y  +66.7188)/(-2.524)

pts = []
for i in range(len(y)):
     pts.append({'x': x[i], 'y': y[i], 'speed': 11.4, 'index': 0})

import json
print(json.dumps(pts, indent=4))