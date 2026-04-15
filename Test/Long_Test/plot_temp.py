import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.dates as mdates
import matplotlib.font_manager as fm
from datetime import datetime

# 1. Font Setup
pagella_path = '/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyrepagella-regular.otf'
try:
    fm.fontManager.addfont(pagella_path)
    plt.rcParams['font.family'] = 'TeX Gyre Pagella'
except:
    pass

# 2. Define the full list of names (21 to match the "widest" row in your CSV)
# If your CSV has a header row, we skip it with skiprows=1

# 3. Read CSV safely
our = pd.read_csv(
    './sensors(1).csv', 
    sep=',', 
    index_col=False,
    on_bad_lines='warn'    # Skips lines that are even longer than 21 columns
)

print("--- Column Data Types ---")
print(our.dtypes)
print("\n--- Data Summary ---")
print(f"Rows loaded: {len(our)}")
print(f"Date Range: {our['time'].min()} to {our['time'].max()}")
# 4. Convert and print dtypes as you requested
our['time'] = pd.to_datetime(our['time'], format='ISO8601', errors='coerce').dt.tz_localize(None)

# 5. Convert numeric columns
temp_cols = ['soil_t1', 'soil_t2', 'soil_t3', 'soil_t4']
cols_to_float = temp_cols + ['air_temp', 'lux']

for col in cols_to_float:
    our[col] = pd.to_numeric(our[col].astype(str).str.replace(',', '.'), errors='coerce')

# --- PRINT TYPES BACK ---
# ------------------------

# 6. Plotting
plt.figure(figsize=(16, 10))

# Limits
start = datetime(2026, 4, 10, 0, 0)
end = datetime(2026, 4, 16, 0, 0)

# Filter for plotting
mask = (our['time'] >= start) & (our['time'] <= end)
plot_df = our.loc[mask].dropna(subset=['time', 'air_temp'])

if plot_df.empty:
    print(f"\nNo data found between {start} and {end}. Check the 'Date Range' above.")
else:
    plt.plot(plot_df['time'], plot_df['air_temp'], label='Air Temp', color='orange', linewidth=4)
    plt.plot(plot_df['time'], plot_df[temp_cols].mean(axis=1), color='blue', label='Avg Soil Temp', alpha=0.4)
    plt.plot(plot_df['time'], plot_df['lux']/10000, color='gold', label='LUX')
    plt.plot(plot_df['time'], plot_df['precip'], color='blue', label='Rain 15min')
    plt.plot(plot_df['time'], plot_df['wind'], color='green', label='Wind Broken')
    plt.plot(plot_df['time'], plot_df['vol'], color='red', label='BatVolt')

# 7. Axis Formatting
plt.axhline(0, color='black', linewidth=1, alpha=0.3)
plt.xlim(start, end)
plt.gca().xaxis.set_major_locator(mdates.DayLocator(interval=1))
plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%a %H:%M'))

plt.xlabel('Time', fontsize=30)
plt.ylabel('Temperature (°C)', fontsize=30)
plt.title('Air and Soil Temperatures', fontsize=32)
plt.xticks(rotation=45, fontsize=28)
plt.yticks(fontsize=28)

plt.legend(fontsize=24, shadow=True)
plt.grid(True, alpha=0.3)
plt.tight_layout()

plt.show()
