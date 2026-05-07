import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.dates as mdates
import matplotlib as mpl
import matplotlib.font_manager as fm
import numpy as np
pagella_path = '/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyrepagella-regular.otf'

# Register it with Matplotlib
fm.fontManager.addfont(pagella_path)
plt.rcParams['font.family'] = 'TeX Gyre Pagella'
# Read CSV correctly
our = pd.read_csv('./sensors.csv', sep=',', decimal=',')  # comma as separator, comma as decimal

dmi = pd.read_csv('./dmi_only_aalborg_sensor_names.csv', sep=',', decimal='.')  # comma as separator, comma as decimal
our.columns = our.columns.str.strip()  # clean extra spaces

# Convert TIME column to datetime
our['TIME'] = pd.to_datetime(our['TIME'])
dmi['DateTime'] = pd.to_datetime(dmi['DateTime'])
print(our.dtypes)
print(dmi.dtypes)
temp_cols = ['SOIL TEMP1', 'SOIL TEMP2', 'SOIL TEMP3', 'SOIL TEMP4']
cols_to_float = ['SOIL TEMP1', 'SOIL TEMP2', 'SOIL TEMP3', 'SOIL TEMP4', 'AIR TEMP', 'LIGHT']
# Convert each column to float
for col in cols_to_float:
    our[col] = pd.to_numeric(our[col], errors='coerce')  # invalid values become NaN
# Calculate row-wise min and max
temp_min = our[temp_cols].min(axis=1) * 0.9094 + 0.1375
temp_max = our[temp_cols].max(axis=1)* 0.9094 + 0.1375
our['LIGHT_SCALED'] = our['LIGHT'] # Keeping your requested factor
our['AIR TEMP'] =  our['AIR TEMP'] * 0.8723 - 0.0601# Keeping your requested factor

# Create Figure and the first Axis (Temperature)
fig, ax1 = plt.subplots(figsize=(16, 10))

# --- PLOT TEMPERATURE DATA (Left Axis) ---
ax1.set_xlabel('Time', fontsize=30)
ax1.set_ylabel('Temperature (°C)', fontsize=30)
ax1.tick_params(axis='y', labelsize=28)
ax1.tick_params(axis='x', labelsize=28, rotation=45)

line_air = ax1.plot(our['TIME'], our['AIR TEMP'], label='Air Temp', color='orange', linewidth=4)
fill_soil = ax1.fill_between(our['TIME'], temp_min, temp_max, color='lightblue', alpha=0.7, label='Soil Temp Range')
line_avg_soil = ax1.plot(our['TIME'], our[temp_cols].mean(axis=1) * 0.9094 + 0.1375, color='blue', label='Average Soil Temp', linewidth=1.5, alpha=0.3)
# line_avg_soil = ax1.plot(our['TIME'], our[temp_cols].mean(axis=1), color='blue', label='Average Soil Temp', linewidth=1.5, alpha=0.3)
line_dmi = ax1.plot(dmi['DateTime'], dmi['air_temp'], label='DMI Avg', color='green', linewidth=4)

# --- PLOT LIGHT DATA (Right Axis) ---
ax2 = ax1.twinx()  # Instantiate a second axes that shares the same x-axis
ax2.set_ylim(0, 200000)
ax2.set_ylabel('Relative LIGHT', color='black', fontsize=30)
ax2.tick_params(axis='y', labelcolor='black', labelsize=28)

line_light = ax2.plot(our['TIME'], our['LIGHT_SCALED'], label='Relative LIGHT', color='gold', linewidth=2)
ax2.fill_between(our['TIME'], our['LIGHT_SCALED'], color='yellow', alpha=0.2)

# --- FORMATTING ---
plt.title('Compensated Air and Soil Temperatures Over Time For Long Deployment Compared to DMI', fontsize=32)

# X-axis date formatting
ax1.xaxis.set_major_locator(mdates.HourLocator(interval=24))
ax1.xaxis.set_major_formatter(mdates.DateFormatter('%a:%H:%M'))

# Combine legends from both axes
lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_2, labels_2 = ax2.get_legend_handles_labels()
ax1.legend(lines_1 + lines_2, labels_1 + labels_2, 
           fontsize=24, loc='upper right', frameon=True, shadow=True, fancybox=True)

plt.subplots_adjust(left=0.08, right=0.92, top=0.92, bottom=0.12)
time_in_seconds = (our['TIME'] - our['TIME'].min()).dt.total_seconds()
time_in_hours = time_in_seconds / 3600

# 2. Calculate the Integrals (Area Under the Curve)
# Using the Trapezoidal Rule: np.trapz(y_values, x_values)
air_temp_integral = np.trapezoid(our['AIR TEMP'], x=time_in_hours)
light_integral = np.trapezoid(our['LIGHT'], x=time_in_hours)
dmi_time_in_hours = (dmi['DateTime'] - dmi['DateTime'].min()).dt.total_seconds() / 3600
dmi_integral = np.trapezoid(dmi['air_temp'], x=dmi_time_in_hours)

print(f"Total Heat Exposure (Our Sensor): {air_temp_integral:.2f} °C-hours")
print(f"Total Heat Exposure (DMI):        {dmi_integral:.2f} °C-hours")
print(f"Total Light Exposure:             {light_integral:.2e} units-hours")
diff_percent = ((air_temp_integral - dmi_integral) / dmi_integral) * 100
print(f"Our sensor measured {diff_percent:.1f}% more total heat energy than DMI.")
# Calculate the percentage difference
percentage_diff = ((air_temp_integral - dmi_integral) / dmi_integral) * 100

print(f"Our sensor recorded {percentage_diff:.1f}% more total thermal accumulation than the DMI station.")
# 1. Force conversion to datetime and strip timezone info
our['TIME'] = pd.to_datetime(our['TIME']).dt.tz_localize(None)
dmi['DateTime'] = pd.to_datetime(dmi['DateTime']).dt.tz_localize(None)

# 2. Prepare for alignment
our_temp = our[['TIME', 'AIR TEMP']].copy().set_index('TIME')
dmi_temp = dmi[['DateTime', 'air_temp']].copy().rename(columns={'DateTime': 'TIME'}).set_index('TIME')

# 3. Align
# We use 'nearest' or 'time' to ensure we get a match even if seconds differ
dmi_aligned = dmi_temp.reindex(our_temp.index, method='nearest', tolerance=pd.Timedelta('1 hour'))
dmi_aligned = dmi_aligned.interpolate(method='time')

# 4. Create the sync_df and check it
sync_df = pd.concat([our_temp, dmi_aligned], axis=1).dropna()

if sync_df.empty:
    print("!!! Error: No overlapping time found !!!")
    print(f"Our starts: {our['TIME'].min()} | Ends: {our['TIME'].max()}")
    print(f"DMI starts: {dmi['DateTime'].min()} | Ends: {dmi['DateTime'].max()}")
else:
    # --- STEP B: Regression ---
    m, c = np.polyfit(sync_df['AIR TEMP'], sync_df['air_temp'], 1)
    our['AIR_TEMP_CALIBRATED'] = our['AIR TEMP'] * m + c
    print(f"Success! Formula: {m:.4f}x + {c:.4f}")
sync_df['Abs_Error'] = (sync_df['AIR TEMP'] - sync_df['air_temp']).abs()

# 2. Integrate the Error over time (in hours)
# This gives you "Total Error-Hours"
total_accumulated_error = np.trapezoid(sync_df['Abs_Error'], x=sync_df.index.astype(int) / 3.6e12) 
# Note: if using the 'time_in_hours' logic from before, use that for x instead.

# 3. Calculate the Mean Absolute Error (MAE)
mae = sync_df['Abs_Error'].mean()

print(f"Total Accumulated Error: {total_accumulated_error:.2f} °C-hours")
print(f"Average Moment-to-Moment Error: {mae:.2f} °C")
our['SOIL_AVG'] = our[temp_cols].mean(axis=1) * 0.9094 + 0.1375

# 2. Prepare for alignment (Soil vs DMI Air Temp)
# Note: Usually we compare our soil to our air, but here we'll keep DMI as the reference
our_soil = our[['TIME', 'SOIL_AVG']].copy().set_index('TIME')
# dmi_temp is already defined from your previous air temp code

# 3. Align datasets
dmi_aligned_soil = dmi_temp.reindex(our_soil.index, method='nearest', tolerance=pd.Timedelta('1 hour'))
dmi_aligned_soil = dmi_aligned_soil.interpolate(method='time')

# 4. Create sync_df for Soil
sync_soil_df = pd.concat([our_soil, dmi_aligned_soil], axis=1).dropna()

if not sync_soil_df.empty:
    # --- Regression for Soil ---
    m_soil, c_soil = np.polyfit(sync_soil_df['SOIL_AVG'], sync_soil_df['air_temp'], 1)
    our['SOIL_CALIBRATED'] = our['SOIL_AVG'] * m_soil + c_soil
    
    # --- Error Metrics for Soil ---
    sync_soil_df['Abs_Error'] = (sync_soil_df['SOIL_AVG'] - sync_soil_df['air_temp']).abs()
    
    # Use your time_in_hours logic for the integral
    time_in_hours_soil = (sync_soil_df.index - sync_soil_df.index.min()).total_seconds() / 3600
    total_acc_error_soil = np.trapezoid(sync_soil_df['Abs_Error'], x=time_in_hours_soil)
    mae_soil = sync_soil_df['Abs_Error'].mean()
    
    # --- Integrals ---
    soil_integral = np.trapezoid(our['SOIL_AVG'], x=time_in_hours) # assuming time_in_hours exists from air code
    
    print("--- SOIL TEMPERATURE ANALYSIS ---")
    print(f"Total Soil Heat Exposure: {soil_integral:.2f} °C-hours")
    print(f"Regression Formula: {m_soil:.4f}x + {c_soil:.4f}")
    print(f"Total Accumulated Error (vs DMI): {total_acc_error_soil:.2f} °C-hours")
    print(f"Average Moment-to-Moment Error: {mae_soil:.2f} °C")
else:
    print("No overlapping data found for Soil Temp analysis.")
plt.show()
