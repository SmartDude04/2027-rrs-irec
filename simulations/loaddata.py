import pandas as pd
import numpy as np

data = pd.read_csv("simulation_linear.csv")

t = data["Time (s)"]
h = data["Altitude (m)"]
v = data["Vertical velocity (m/s)"]
a = data["Vertical acceleration (m/s²)"]
rho = data["Air pressure (Pa)"]

condensed = [(t[i], h[i], v[i], a[i], rho[i]) for i in range(len(t))]

for row in condensed:
    print(f"{{{row[0]}f, {row[1]}f, {row[2]}f, {row[3]}f, {row[4]}f}},")

def resample_openrocket(filepath, dt=0.01):
    """
    Reads an OpenRocket CSV export and linearly interpolates all numeric
    columns onto a uniform time grid.
    """
    # 1. Extract the header row (OpenRocket prefixes headers with '# ')
    header = None
    with open(filepath, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('# Time (s)'):
                header = [col.strip() for col in line.lstrip('#').split(',')]
                break

    # 2. Load data, ignoring '#' comment lines
    df = pd.read_csv(filepath, comment='#', names=header)

    # 3. Clean data: keep numeric columns and drop duplicate timestamps
    # (OpenRocket logs two identical timestamps when flight events occur)
    time_col = df.columns[0]
    df = df.select_dtypes(include=[np.number])
    df = df.drop_duplicates(subset=[time_col], keep='last').sort_values(by=time_col)

    # 4. Create the uniform time grid
    t_start = df[time_col].min()
    t_end = df[time_col].max()
    uniform_time = np.arange(t_start, t_end, dt)

    # 5. Linearly interpolate every column onto the new time array
    resampled_data = {time_col: uniform_time}
    for col in df.columns:
        if col != time_col:
            resampled_data[col] = np.interp(uniform_time, df[time_col], df[col])

    return pd.DataFrame(resampled_data)

# df_linear = resample_openrocket('nominal-flight-2.csv', dt=0.1)
# df_linear.to_csv('simulation_linear.csv', index=False)
