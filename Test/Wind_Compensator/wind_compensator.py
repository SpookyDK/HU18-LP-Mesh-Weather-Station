import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from scipy.integrate import trapezoid
from scipy.optimize import minimize
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
LIMIT_Cor = 2.116745

def reconstruct_wind(row):
    raw_val = row['wind']
    reference = row['Højeste 10 min. middelvind']
    # Check multiples 0 through 4 (covering wind up to ~12.7 m/s)
    potential_values = [raw_val + (i * LIMIT) for i in range(5)]
    return min(potential_values, key=lambda x: abs(x - reference))

def reconstruct_wind_cor(row):
    raw_val = row['wind']
    reference = row['Højeste 10 min. middelvind']
    # Check multiples 0 through 4 (covering wind up to ~12.7 m/s)
    potential_values = [raw_val + (i * LIMIT_Cor) for i in range(5)]
    return min(potential_values, key=lambda x: abs(x - reference))

def objective_function(test_limit):
    def reconstruct(row):
        raw_val = row['wind']
        reference = row['Højeste 10 min. middelvind']
        # Check multiples 0-4
        potential_values = [raw_val + (i * test_limit) for i in range(5)]
        return min(potential_values, key=lambda x: abs(x - reference))
    
    # Apply the reconstruction with the test_limit
    temp_corrected = merged.apply(reconstruct, axis=1)
    
    # Calculate Mean Squared Error (MSE)
    mse = ((temp_corrected - merged['Højeste 10 min. middelvind'])**2).mean()
    return mse

merged['wind_corrected'] = merged.apply(reconstruct_wind, axis=1)

merged['wind_corrected_cor'] = merged.apply(reconstruct_wind_cor, axis=1)

# 6. Aggregate for Visualization
hourly_avg = merged.groupby('hour').agg({
    'wind': 'mean',
    'wind_corrected': 'mean',
    'Højeste 10 min. middelvind': 'first',
    'wind_corrected_cor': 'mean'
}).reset_index()

# 7. Plotting

plt.xlabel('Time', fontsize=30)
plt.ylabel('Wind Speed [m/s]', fontsize=30)
plt.tick_params(axis='y', labelsize=28)
plt.tick_params(axis='x', labelsize=28, rotation=45)
plt.title("Gathered Wind Speed Data with Correction Applied Compared to DMI",fontsize=35)
ax = plt.gca()
ax.xaxis.set_major_locator(mdates.HourLocator(interval=24))
ax.xaxis.set_major_formatter(mdates.DateFormatter('%a:%H:%M'))
plt.legend(fontsize = 24, loc="upper right", frameon=True, shadow=True, fancybox=True)
plt.subplots_adjust(left=0.08, right=0.92, top=0.92, bottom=0.2)
plt.grid(True, alpha=0.3)
# 1. Prepare data for integration (Time in hours)
# We convert time to a numeric value (hours from start) to get an area in [m/s * h]
time_delta_hours = (hourly_avg['hour'] - hourly_avg['hour'].min()).dt.total_seconds() / 3600

# 2. Calculate Integrals (Total Area under the curve)
area_dmi = trapezoid(hourly_avg['Højeste 10 min. middelvind'], time_delta_hours)
area_raw = trapezoid(hourly_avg['wind'], time_delta_hours)
area_cor = trapezoid(hourly_avg['wind_corrected'], time_delta_hours)

# 3. Calculate Sectional Differences (Over/Under DMI)
diff_cor = hourly_avg['wind_corrected'] - hourly_avg['Højeste 10 min. middelvind']
over_dmi_area = trapezoid(np.maximum(diff_cor, 0), time_delta_hours)
under_dmi_area = trapezoid(np.minimum(diff_cor, 0), time_delta_hours)

# 4. Print Comparison Report
print("="*50)
print("WIND DATA INTEGRAL ANALYSIS")
print("="*50)
print(f"{'Source':<20} | {'Total Area (m/s*h)':<20}")
print("-"*50)
print(f"{'DMI (Reference)':<20} | {area_dmi:>15.2f}")
print(f"{'Raw (Buggy)':<20} | {area_raw:>15.2f}")
print(f"{'Corrected':<20} | {area_cor:>15.2f}")
print("-"*50)

print(f"\nCorrection Accuracy Metrics:")
print(f"Total Recovery:      {area_cor - area_raw:>15.2f} units")
print(f"Residual Deviation:  {area_cor - area_dmi:>15.2f} units (Net)")

print(f"\nDirectional Error (Corrected vs DMI):")
print(f"Overshoot Area:      {over_dmi_area:>15.2f} units (Points above DMI)")
print(f"Undershoot Area:     {abs(under_dmi_area):>15.2f} units (Points below DMI)")
print("="*50)
# Calculate absolute error for every row
merged['abs_error'] = abs(merged['wind_corrected'] - merged['Højeste 10 min. middelvind'])
merged['abs_error_raw'] = abs(merged['wind'] - merged['Højeste 10 min. middelvind'])
merged['abs_error_cor'] = abs(merged['wind_corrected_cor'] - merged['Højeste 10 min. middelvind'])

# Calculate the mean error relative to the mean wind speed
# We use the mean of DMI as the denominator to avoid division by zero
mean_reference = merged['Højeste 10 min. middelvind'].mean()
mean_abs_error = merged['abs_error'].mean()

mean_reference_raw = merged['Højeste 10 min. middelvind'].mean()
mean_abs_error_raw = merged['abs_error_raw'].mean()

mean_reference_cor = merged['Højeste 10 min. middelvind'].mean()
mean_abs_error_cor = merged['abs_error_cor'].mean()

accuracy_point = (1 - (mean_abs_error / mean_reference)) * 100
accuracy_point_raw = (1 - (mean_abs_error_raw / mean_reference_raw)) * 100
accuracy_point_cor = (1 - (mean_abs_error_cor / mean_reference_cor)) * 100

print(f"Point-to-Point Accuracy:    {accuracy_point:.2f}%")
initial_guess = 2.539454061
print("Optimizing LIMIT value... please wait...")

# We use Nelder-Mead because our function is non-differentiable (due to the 'min' selection)
result = minimize(objective_function, initial_guess, method='Nelder-Mead', tol=1e-6)

optimal_limit = result.x[0]
min_mse = result.fun

print("-" * 30)
print(f"Optimization Complete!")
print(f"Original Limit: {initial_guess:.9f}")
print(f"Optimal Limit:  {optimal_limit:.9f}")
print(f"Improvement:    {((objective_function(initial_guess) - min_mse) / objective_function(initial_guess))*100:.2f}% reduction in error")
print("-" * 30)

# 3. Use the NEW limit for the final data
LIMIT = optimal_limit

DmiAvg = hourly_avg['Højeste 10 min. middelvind'].mean()
RawAvg = hourly_avg['wind'].mean()
CorAvg = hourly_avg['wind_corrected'].mean()
CorCorAvg = hourly_avg['wind_corrected_cor'].mean()

DmiLabel = f'DMI Reported Wind Speed: Avg +{DmiAvg:.1f} [m/s]'
plt.plot(hourly_avg['hour'], hourly_avg['Højeste 10 min. middelvind'], label=DmiLabel, color='green', linewidth=2, alpha=0.8)


RawLabel = f'Raw Wind Data (Buggy): Avg +{RawAvg:.1f} [m/s]: Acc {accuracy_point_raw:.1f}%'
plt.plot(hourly_avg['hour'], hourly_avg['wind'], label=RawLabel, color='orange', alpha=0.8)

CorLabel = f'Corrected Wind Data: Avg +{CorAvg:.1f} [m/s]: Acc {accuracy_point:.1f}%'
plt.plot(hourly_avg['hour'], hourly_avg['wind_corrected'], label=CorLabel, color='red', linewidth=2, alpha=0.8)

CorCorLabel = f'Calibrated Corrected Wind Data: Avg +{CorCorAvg:.1f} [m/s]: Acc {accuracy_point_cor:.1f}%'
plt.plot(hourly_avg['hour'], hourly_avg['wind_corrected_cor'], label=CorCorLabel, color='pink', linewidth=2, alpha=0.8)

plt.legend(fontsize = 24, loc="upper right", frameon=True, shadow=True, fancybox=True)
plt.show()
