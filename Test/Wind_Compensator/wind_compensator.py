import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 1. Load the datasets
dmi_df = pd.read_csv('DMI - Flyvestation Ålborg.csv')
sensors_df = pd.read_csv('sensors.csv')

# 2. Clean column names and handle time formats
sensors_df.columns = sensors_df.columns.str.strip()
dmi_df.columns = dmi_df.columns.str.strip()

# Remove timezone info to allow merging
sensors_df['time'] = pd.to_datetime(sensors_df['time']).dt.tz_localize(None)
dmi_df['DateTime'] = pd.to_datetime(dmi_df['DateTime']).dt.tz_localize(None)

# 3. Create a common 'hour' key for merging
sensors_df['hour'] = sensors_df['time'].dt.floor('h')
dmi_df['hour'] = dmi_df['DateTime'].dt.floor('h')

# 4. Merge the sensor data with DMI reference
merged = pd.merge(sensors_df, dmi_df, on='hour', how='inner')

# 5. Correction Logic
# Since the bug causes the value to loop at 2.539454061, we find the 
# multiple (n * limit) that brings the sensor value closest to DMI.
LIMIT = 2.539454061

def reconstruct_wind(row):
    raw_val = row['wind']
    reference = row['Højeste 10 min. middelvind']
    # Check multiples 0 through 4 (covering wind up to ~12.7 m/s)
    potential_values = [raw_val + (i * LIMIT) for i in range(5)]
    return min(potential_values, key=lambda x: abs(x - reference))

merged['wind_corrected'] = merged.apply(reconstruct_wind, axis=1)

# 6. Aggregate for Visualization
hourly_avg = merged.groupby('hour').agg({
    'wind': 'mean',
    'wind_corrected': 'mean',
    'Højeste 10 min. middelvind': 'first'
}).reset_index()

# 7. Plotting
plt.figure(figsize=(14, 7))
plt.plot(hourly_avg['hour'], hourly_avg['Højeste 10 min. middelvind'], 
         label='DMI Reference', color='black', linewidth=2, linestyle='--')
plt.plot(hourly_avg['hour'], hourly_avg['wind'], 
         label='Raw Sensor Data (Buggy)', color='red', alpha=0.6)
plt.plot(hourly_avg['hour'], hourly_avg['wind_corrected'], 
         label='Corrected Sensor Data', color='green', linewidth=2)

plt.title('Wind Data Reconstruction: Correcting Sensor Wrap-around Bug')
plt.xlabel('Time')
plt.ylabel('Wind Speed (m/s)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.show()