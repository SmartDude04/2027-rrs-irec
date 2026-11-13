import csv

import numpy as np

SIZE = 50000

header = ["position_m", "velocity_mps", "acceleration_mps2", "pressure_pa"]

rng = np.random.default_rng()
pos = rng.integers(low=0, high=1000, size=SIZE)
velo = rng.integers(low=0, high=1000, size=SIZE)
accel = rng.integers(low=0, high=1000, size=SIZE)
rho = rng.integers(low=0, high=1000, size=SIZE)

rows = [[]] * SIZE

for i in range(SIZE):
    rows[i] = [pos[i], velo[i], accel[i], rho[i]]

with open("flights.csv", "w", newline="", encoding="utf-8") as file:
    writer = csv.writer(file)
    writer.writerow(header)
    writer.writerows(rows)
