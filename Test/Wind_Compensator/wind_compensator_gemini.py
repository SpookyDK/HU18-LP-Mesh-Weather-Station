import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from scipy.integrate import trapezoid
from scipy.optimize import minimize
import matplotlib.font_manager as fm

pagella_path = '/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyrepagella-regular.otf'
try:
    fm.fontManager.addfont(pagella_path)
    plt.rcParams['font.family'] = 'TeX Gyre Pagella'
except:
    print("Font not found, using default.")
# 1. Load the datasets
dmi_df = pd.read_csv('DMI - Flyvestation Ålborg.csv')
sensors_df = pd.read_csv('sensors.csv')

# 2. Clean column names and handle time formats
sensors_df.columns = sensors_df.columns.str.strip()
dmi_df.columns = dmi_df.columns.str.strip()

sensors_df['time'] = pd.to_datetime(sensors_df['time']).dt.tz_localize(None)
dmi_df['DateTime'] = pd.to_datetime(dmi_df['DateTime']).dt.tz_localize(None)

# 3. Create a common 'hour' key for merging
sensors_df['hour'] = sensors_df['time'].dt.floor('h')
dmi_df['hour'] = dmi_df['DateTime'].dt.floor('h')

# 4. Merge the sensor data with DMI reference
merged = pd.merge(sensors_df, dmi_df, on='hour', how='inner')

# 5. Optimization Logic (Must run before reconstruction)
def objective_function(test_limit):
    def reconstruct(row):
        raw_val = row['wind']
        reference = row['Højeste 10 min. middelvind']
        potential_values = [raw_val + (i * test_limit) for i in range(10)]
        return min(potential_values, key=lambda x: abs(x - reference))
    temp_corrected = merged.apply(reconstruct, axis=1)
    mse = ((temp_corrected - merged['Højeste 10 min. middelvind'])**2).mean()
    return mse

print("Optimizing LIMIT value... please wait...")
initial_guess = 2.539454061
result = minimize(objective_function, initial_guess, method='Nelder-Mead', tol=1e-6)
LIMIT_Cor = result.x[0]

# 6. Correction Logic using Optimized Limit
LIMIT = 2.539454061

def reconstruct_wind(row):
    raw_val = row['wind']
    reference = row['Højeste 10 min. middelvind']
    potential_values = [raw_val + (i * LIMIT) for i in range(10)]
    return min(potential_values, key=lambda x: abs(x - reference))

def reconstruct_wind_cor(row):
    raw_val = row['wind']
    reference = row['Højeste 10 min. middelvind']
    potential_values = [raw_val + (i * LIMIT_Cor) for i in range(10)]
    return min(potential_values, key=lambda x: abs(x - reference))

merged['wind_corrected'] = merged.apply(reconstruct_wind, axis=1)
merged['wind_corrected_cor'] = merged.apply(reconstruct_wind_cor, axis=1)

# 7. Aggregate for Visualization
hourly_avg = merged.groupby('hour').agg({
    'wind': 'mean',
    'wind_corrected': 'mean',
    'Højeste 10 min. middelvind': 'first',
    'wind_corrected_cor': 'mean'
}).reset_index()

# 8. Accuracy Metrics
mean_reference = merged['Højeste 10 min. middelvind'].mean()
accuracy_point = (1 - (abs(merged['wind_corrected'] - merged['Højeste 10 min. middelvind']).mean() / mean_reference)) * 100
accuracy_point_raw = (1 - (abs(merged['wind'] - merged['Højeste 10 min. middelvind']).mean() / mean_reference)) * 100
accuracy_point_cor = (1 - (abs(merged['wind_corrected_cor'] - merged['Højeste 10 min. middelvind']).mean() / mean_reference)) * 100

# 9. Integral Analysis
time_delta_hours = (hourly_avg['hour'] - hourly_avg['hour'].min()).dt.total_seconds() / 3600
area_dmi = trapezoid(hourly_avg['Højeste 10 min. middelvind'], time_delta_hours)
area_raw = trapezoid(hourly_avg['wind'], time_delta_hours)
area_cor = trapezoid(hourly_avg['wind_corrected_cor'], time_delta_hours)
diff_cor = hourly_avg['wind_corrected_cor'] - hourly_avg['Højeste 10 min. middelvind']
over_dmi_area = trapezoid(np.maximum(diff_cor, 0), time_delta_hours)
under_dmi_area = trapezoid(np.minimum(diff_cor, 0), time_delta_hours)

# 10. Print Reports
print("="*50)
print(f"OPTIMIZATION COMPLETE")
print(f"Optimal Limit: {LIMIT_Cor:.9f}")
print(f"Improvement: {((objective_function(initial_guess) - result.fun) / objective_function(initial_guess))*100:.2f}%")
print("-" * 50)
print(f"DMI Area: {area_dmi:.2f} | Opt Area: {area_cor:.2f}")
print(f"Overshoot: {over_dmi_area:.2f} | Undershoot: {abs(under_dmi_area):.2f}")
print("="*50)

# 11. Plotting (Your original style)
plt.figure(figsize=(20, 10))
plt.xlabel('Time', fontsize=30)
plt.ylabel('Wind Speed [m/s]', fontsize=30)
plt.tick_params(axis='y', labelsize=28)
plt.tick_params(axis='x', labelsize=28, rotation=45)
plt.title("Gathered Wind Speed Data with Correction Applied Compared to DMI",fontsize=35)
ax = plt.gca()
ax.xaxis.set_major_locator(mdates.HourLocator(interval=24))
ax.xaxis.set_major_formatter(mdates.DateFormatter('%a:%H:%M'))

DmiAvg = hourly_avg['Højeste 10 min. middelvind'].mean()
RawAvg = hourly_avg['wind'].mean()
CorAvg = hourly_avg['wind_corrected'].mean()
CorCorAvg = hourly_avg['wind_corrected_cor'].mean()

plt.plot(hourly_avg['hour'], hourly_avg['Højeste 10 min. middelvind'], label=f'DMI Reported Wind Speed: Avg +{DmiAvg:.1f} [m/s]', color='green', linewidth=2, alpha=0.8)
plt.plot(hourly_avg['hour'], hourly_avg['wind'], label=f'Raw Wind Data (Buggy): Avg +{RawAvg:.1f} [m/s]: Acc {accuracy_point_raw:.1f}%', color='orange', alpha=0.8)
plt.plot(hourly_avg['hour'], hourly_avg['wind_corrected'], label=f'Corrected Wind Data: Avg +{CorAvg:.1f} [m/s]: Acc {accuracy_point:.1f}%', color='red', linewidth=2, alpha=0.8)
plt.plot(hourly_avg['hour'], hourly_avg['wind_corrected_cor'], label=f'Calibrated Corrected Wind Data: Avg +{CorCorAvg:.1f} [m/s]: Acc {accuracy_point_cor:.1f}%', color='pink', linewidth=2, alpha=0.8)

plt.legend(fontsize = 24, loc="upper right", frameon=True, shadow=True, fancybox=True)
plt.subplots_adjust(left=0.08, right=0.92, top=0.92, bottom=0.2)
plt.grid(True, alpha=0.3)
plt.show()
