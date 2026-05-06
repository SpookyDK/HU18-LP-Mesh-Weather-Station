import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.dates as mdates
import matplotlib.font_manager as fm

# Font setup
pagella_path = '/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyrepagella-regular.otf'
fm.fontManager.addfont(pagella_path)
plt.rcParams['font.family'] = 'TeX Gyre Pagella'

# 1. Load Data
our = pd.read_csv('sensors.csv', sep=',', decimal=',')  
dmiAal = pd.read_csv('./dmi_rain.csv', sep=',', decimal='.')  
dmiAir = pd.read_csv('./dmi_only_aalborg_sensor_names.csv', sep=',', decimal='.')  
our.columns = our.columns.str.strip()  

# 2. Convert Time
our['TIME'] = pd.to_datetime(our['TIME'])
dmiAal['DateTime'] = pd.to_datetime(dmiAal['DateTime'])
dmiAir['DateTime'] = pd.to_datetime(dmiAir['DateTime'])

# 3. Numeric Conversion
cols_to_float = ['WIND', 'SOIL TEMP1', 'SOIL TEMP2', 'SOIL TEMP3', 'SOIL TEMP4', 'AIR TEMP', 'LIGHT', 'RAIN']
for col in cols_to_float:
    our[col] = pd.to_numeric(our[col], errors='coerce')

# 4. THE RESAMPLING FIX
# Set index to TIME first
our.set_index('TIME', inplace=True)

# Resample to hourly using SUM for rainfall
# 'label=right' or 'closed=right' can be used if your sensor logs at the end of the hour
our_hourly = our.resample('1h').sum() 

# 5. Plotting
plt.figure(figsize=(16,10))
# Plotting 'our' data as bars
plt.ylim(0,2.5)
plt.bar(our_hourly.index, our_hourly['RAIN'], 
        label='Measured Total Hourly Rain', 
        color='blue', alpha=0.5, width=0.03)

# Plotting 'dmi' data as bars (shifted slightly or with transparency to compare)
plt.bar(dmiAal['DateTime'], dmiAal['Rain'], 
        label='DMI Hourly Rain Aalborg Area', 
        color='red', alpha=0.5, width=0.03)

plt.bar(dmiAir['DateTime'], dmiAir['precip'], 
        label='DMI Hourly Rain Aalborg Airport', 
        color='orange', alpha=0.5, width=0.03)
# Formatting
plt.xlabel('Time', fontsize=30)
plt.ylabel('Precipitation (mm)', fontsize=30)
plt.title('Total Hourly Precipitation Comparison Aalborg General Area and Airport', fontsize=32)

plt.gca().xaxis.set_major_locator(mdates.DayLocator(interval=1))  # Tick every day
plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%a %d'))
plt.xticks(rotation=45, fontsize=24)
plt.yticks(fontsize=24)

plt.legend(fontsize=24, loc='upper right', frameon=True, shadow=True)
plt.tight_layout()
plt.show()
