import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.dates as mdates
import matplotlib as mpl
import matplotlib.font_manager as fm
import numpy as np

# --- FONT SETUP ---
pagella_path = '/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyrepagella-regular.otf'
try:
    fm.fontManager.addfont(pagella_path)
    plt.rcParams['font.family'] = 'TeX Gyre Pagella'
except:
    print("Font not found, using default.")

# --- DATA PROCESSING ---
our = pd.read_csv('./sensors.csv', sep=',', decimal=',') 
our.columns = our.columns.str.strip() 
our['TIME'] = pd.to_datetime(our['TIME'])

cols_to_float = ['LIGHT', 'bat_voltage']
for col in cols_to_float:
    our[col] = pd.to_numeric(our[col], errors='coerce')

our['LIGHT_SCALED'] = our['LIGHT']

# --- REGRESSION CALCULATIONS ---
x_numeric = mdates.date2num(our['TIME'])
y_values = our['bat_voltage']
idx = np.isfinite(x_numeric) & np.isfinite(y_values)
slope, intercept = np.polyfit(x_numeric[idx], y_values[idx], 1)
trend_values = slope * x_numeric + intercept
# Slope is in V/day because date2num units are days
equation_label = f'Trend: +{slope:.4f} V/day'

# --- PLOTTING ---
fig, ax1 = plt.subplots(figsize=(16, 10))

# 1. Plot Battery Voltage (Left Axis)
ax1.set_ylim(3.5, 4.25)
ax1.set_xlabel('Time', fontsize=30)
ax1.set_ylabel('Battery Voltage', fontsize=30)
ax1.tick_params(axis='y', labelsize=28)
ax1.tick_params(axis='x', labelsize=28, rotation=45)

ax1.plot(our['TIME'], our['bat_voltage'], label='Battery Voltage [V]', color='red', linewidth=4)
ax1.plot(our['TIME'], trend_values, label=equation_label, color='darkred', linestyle='--', linewidth=3)

# 2. Plot Light Data (Right Axis)
ax2 = ax1.twinx() 
ax2.set_ylim(0, 100000)
ax2.set_ylabel('Relative LIGHT', color='black', fontsize=30)
ax2.tick_params(axis='y', labelcolor='black', labelsize=28)

ax2.plot(our['TIME'], our['LIGHT_SCALED'], label='Relative LIGHT', color='gold', linewidth=2)
ax2.fill_between(our['TIME'], our['LIGHT_SCALED'], color='yellow', alpha=0.2)

# --- THE FIX: VERTICAL LINES AT MIDNIGHT ---
# Define the locator for Midnight (00:00)
midnight_locator = mdates.HourLocator(byhour=0)
ax1.xaxis.set_major_locator(midnight_locator)
ax1.xaxis.set_major_formatter(mdates.DateFormatter('%a:%H:%M'))

# Get the specific timestamps for midnight within your data range
# This is much more reliable than get_xticks()
start_date = our['TIME'].min()
end_date = our['TIME'].max()
midnight_ticks = midnight_locator.tick_values(start_date, end_date)

for t in midnight_ticks:
    ax1.axvline(x=t, color='gray', linestyle='--', alpha=0.4, linewidth=1.5, zorder=1)

# --- LEGEND & FINAL TOUCHES ---
# Update legend to include all lines
lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_2, labels_2 = ax2.get_legend_handles_labels()

ax1.legend(lines_1 + lines_2, labels_1 + labels_2, 
           fontsize=24, loc='upper right', frameon=True, shadow=True, fancybox=True)

plt.title('Battery Level vs Measured Sunlight Intensity', fontsize=32)
plt.subplots_adjust(left=0.08, right=0.92, top=0.92, bottom=0.15)
plt.show()
