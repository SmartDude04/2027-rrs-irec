import pandas as pd

data = pd.read_csv("noise-flight.csv")

t = data["Time (s)"]
h = data["Altitude (m)"]
v = data["Vertical velocity (m/s)"]
a = data["Vertical acceleration (m/s²)"]
rho = data["Air pressure (Pa)"]

condensed = [(t[i], h[i], v[i], a[i], rho[i]) for i in range(len(t))]

for row in condensed:
    print(f"{{{row[0]}f, {row[1]}f, {row[2]}f, {row[3]}f, {row[4]}f}},")
