import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

nominal_data = pd.read_csv("nominal-flight.csv")

rng = np.random.default_rng(seed=42)

noise_columns = [
    "Vertical acceleration (m/s²)",
    "Air pressure (Pa)",
]

mean = 0.0
std_dev = 5.0

noisy = nominal_data.copy()

noise = rng.normal(
    loc=mean,
    scale=std_dev,
    size=nominal_data[noise_columns].shape,
)

noisy[noise_columns] = nominal_data[noise_columns] + noise

noisy.to_csv("noise-flight.csv", index=False)

plt.figure()
plt.plot(nominal_data["Time (s)"], nominal_data["Vertical acceleration (m/s²)"], label="Nominal")
plt.plot(noisy["Time (s)"], noisy["Vertical acceleration (m/s²)"], label="Noisy")
plt.xlabel("Time (s)")
plt.ylabel("Vertical acceleration (m/s²)")
plt.legend()
plt.show()
