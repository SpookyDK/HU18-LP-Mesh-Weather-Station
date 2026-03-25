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

dmi = pd.read_csv('DMI_WIND.csv', sep=',', decimal='.')  # comma as separator, comma as decimal
our.columns = our.columns.str.strip()  # clean extra spaces

# Convert TIME column to datetime
our['TIME'] = pd.to_datetime(our['TIME'])
dmi['DateTime'] = pd.to_datetime(dmi['DateTime'])

cols_to_float = ['WIND', 'SOIL TEMP1', 'SOIL TEMP2', 'SOIL TEMP3', 'SOIL TEMP4', 'AIR TEMP', 'LIGHT']
# Convert each column to float
for col in cols_to_float:
    our[col] = pd.to_numeric(our[col], errors='coerce')  # invalid values become NaN
print(our.dtypes)
print(dmi.all)
print(our['TIME'].dtype)
numeric_cols = our.select_dtypes(include='number').columns

cols_to_float = ['WIND', 'SOIL TEMP1', 'SOIL TEMP2', 'SOIL TEMP3', 'SOIL TEMP4', 'AIR TEMP', 'LIGHT']
# Convert each column to float
for col in cols_to_float:
    our[col] = pd.to_numeric(our[col], errors='coerce')  # invalid values become NaN
# Set TIME as index
our_hourly = our.set_index('TIME')[numeric_cols].resample('60min').mean()
# Calculate row-wise min and max
plt.figure(figsize=(16,10))
plt.xlabel('Time', fontsize = 30)
plt.ylabel('WindSpeed (M/S)', fontsize = 30)
plt.title('Wind Speed Measured From Initial Test Compared To DMI', fontsize = 32)
plt.plot(our['TIME'], our['WIND'], label='Wind Speed', color='orange', linewidth=4)

plt.plot(our_hourly.index, our_hourly['WIND'], label='Wind Speed Avg', color='red', linewidth=4)
plt.plot(dmi['DateTime'], dmi['High'], label='DMI 10 Min High', color='lime', linewidth = 2)
plt.plot(dmi['DateTime'], dmi['Gust'], label='DMI Gust', color='lightgreen', linewidth = 2)
plt.plot(dmi['DateTime'], dmi['Avg'], label='DMI Avg', color='green', linewidth = 4)
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

