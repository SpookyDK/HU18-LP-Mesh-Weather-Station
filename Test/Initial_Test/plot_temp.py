import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.dates as mdates
import matplotlib as mpl
import matplotlib.font_manager as fm
pagella_path = '/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyrepagella-regular.otf'

# Register it with Matplotlib
fm.fontManager.addfont(pagella_path)
plt.rcParams['font.family'] = 'TeX Gyre Pagella'
# Read CSV correctly
our = pd.read_csv('Collected.csv', sep=',', decimal=',')  # comma as separator, comma as decimal

dmi = pd.read_csv('DMI_TEMP.csv', sep=',', decimal='.')  # comma as separator, comma as decimal
our.columns = our.columns.str.strip()  # clean extra spaces

# Convert TIME column to datetime
our['TIME'] = pd.to_datetime(our['TIME'])
our['LIGHT'] = our['LIGHT']/10000
dmi['DateTime'] = pd.to_datetime(dmi['DateTime'])
print(our.all)
print(dmi.all)
temp_cols = ['SOIL TEMP1', 'SOIL TEMP2', 'SOIL TEMP3', 'SOIL TEMP4']

# Calculate row-wise min and max
temp_min = our[temp_cols].min(axis=1)
temp_max = our[temp_cols].max(axis=1)
plt.figure(figsize=(16,10))
plt.xlabel('Time', fontsize = 24)
plt.ylabel('Temperature (°C)', fontsize = 24)
plt.title('Air and Soil Temperatures Over Time For Initial Test Compared to DMI', fontsize = 25)
plt.plot(our['TIME'], our['LIGHT'], label=' Relative LIGHT', color='gold', linewidth=2)
plt.fill_between(our['TIME'], our['LIGHT'], color='yellow', alpha=0.2)  # alpha = transparency
plt.plot(our['TIME'], our['AIR TEMP'], label='Air Temp', color='orange', linewidth=4)
plt.fill_between(our['TIME'], temp_min, temp_max, color='lightblue', alpha=0.7, label='Soil Temp Range')

# Optional: add a line for average temperature
plt.plot(our['TIME'], our[temp_cols].mean(axis=1), color='blue', label='Average Soil Temp', linewidth=1.5, alpha=0.3)
# plt.plot(our['TIME'], our['SOIL TEMP1'], label='Soil Temp 1', color='blue')
# plt.plot(our['TIME'], our['SOIL TEMP2'], label='Soil Temp 2', color='green')
# plt.plot(our['TIME'], our['SOIL TEMP3'], label='Soil Temp 3', color='orange')
# plt.plot(our['TIME'], our['SOIL TEMP4'], label='Soil Temp 4', color='purple')

plt.plot(dmi['DateTime'], dmi['Low'], label='DMI Low', color='lime', linewidth = 2)
plt.plot(dmi['DateTime'], dmi['High'], label='DMI High', color='lightgreen', linewidth = 2)
plt.plot(dmi['DateTime'], dmi['Avg'], label='DMI Avg', color='green', linewidth = 4)
plt.axhline()
plt.gca().xaxis.set_major_locator(mdates.HourLocator(interval=2))  # every 2 hours
plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%a:%H:%M')) # show HH:MM
plt.xticks(rotation=45, fontsize=20)  # x-axis values
plt.yticks(fontsize=20)               # y-axis values
plt.subplots_adjust(left=0.05, right=0.98, top=0.95, bottom=0.07, hspace=0.3)
plt.legend(
    fontsize=20,           # font size
    loc='upper right',     # legend position
    frameon=True,          # show box around legend
    shadow=True,           # shadow effect
    fancybox=True          # rounded corners
)
plt.show()

