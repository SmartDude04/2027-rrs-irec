import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

nominal_data = pd.read_csv("nominal-flight.csv")

rng = np.random.default_rng(seed=42)

num_cols = nominal_data.select_dtypes(include=[np.number]).columns

mean = 0.0
std_dev = 5
noise = rng.normal(loc=mean, scale=std_dev, size=nominal_data[num_cols].shape)

noisy = nominal_data.copy()
noisy[num_cols] = noisy[num_cols] + noise

noisy["Time (s)"] = nominal_data["Time (s)"]

noisy.to_csv("noise-flight.csv", index=False)

plt.figure()
plt.plot(nominal_data["Time (s)"], nominal_data["Vertical acceleration (m/s²)"], label="Nominal")
plt.plot(noisy["Time (s)"], noisy["Vertical acceleration (m/s²)"], label="Noisy")
plt.xlabel("Time (s)")
plt.ylabel("Vertical acceleration (m/s²)")
plt.legend()
plt.show()
