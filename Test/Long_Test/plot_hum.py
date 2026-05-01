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
our = pd.read_csv('CollectedLargeFormatted.csv', sep=',', decimal=',')  # comma as separator, comma as decimal

dmi = pd.read_csv('DMI_HUM.csv', sep=',', decimal='.')  # comma as separator, comma as decimal
our.columns = our.columns.str.strip()  # clean extra spaces

# Convert TIME column to datetime
our['TIME'] = pd.to_datetime(our['TIME'])
dmi['DateTime'] = pd.to_datetime(dmi['DateTime'])
print(our.dtypes)
print(dmi.all)

cols_to_float = ['SOIL TEMP1', 'SOIL TEMP2', 'SOIL TEMP3', 'SOIL TEMP4', 'AIR TEMP', 'LIGHT', 'AIR HUM']
# Convert each column to float
for col in cols_to_float:
    our[col] = pd.to_numeric(our[col], errors='coerce')  # invalid values become NaN
# Set TIME as index
# Calculate row-wise min and max
plt.figure(figsize=(16,10))
plt.xlabel('Time', fontsize = 30)
plt.ylabel('Humidity (RH%)', fontsize = 30)
plt.title('Air Humidity Measured From Initial Test Compared To DMI', fontsize = 32)
plt.plot(our['TIME'], our['AIR HUM'], label='Measured Humidity', color='orange', linewidth=4)
plt.plot(dmi['DateTime'], dmi['Humidity'], label='DMI Humidity', color='green', linewidth = 4)
plt.axhline()
plt.gca().xaxis.set_major_locator(mdates.HourLocator(interval=2))  # every 2 hours
plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%a:%H:%M')) # show HH:MM
plt.xticks(rotation=45, fontsize=28)  # x-axis values
plt.yticks(fontsize=28)               # y-axis values
plt.subplots_adjust(left=0.05, right=0.98, top=0.95, bottom=0.09, hspace=0.3)
plt.legend(
    fontsize=28,           # font size
    loc='upper right',     # legend position
    frameon=True,          # show box around legend
    shadow=True,           # shadow effect
    fancybox=True          # rounded corners
)
plt.show()

