from os import wait
import matplotlib.pyplot as plt 
import numpy as np 
import pandas as pd 
import matplotlib.font_manager as fm

pagella_path = '/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyrepagella-regular.otf'

# Register it with Matplotlib
fm.fontManager.addfont(pagella_path)
plt.rcParams['font.family'] = 'TeX Gyre Pagella'

csvFuller = pd.read_csv("./power_Long.csv") 
csvFull = pd.read_csv("./power_Long.csv") 
csvGps = pd.read_csv("./power_Long.csv") 
segment1 = csvGps[csvGps['TIME'].between(1792126, 1949262)]
segment2 = csvGps[csvGps['TIME'].between(3592167, 3648253)]
print(csvFull.dtypes)
mask = (csvFull['TIME'].between(1792126, 1949262)) | \
       (csvFull['TIME'].between(3592167, 3648253))
csvFull.loc[mask, 'AMP'] = np.nan
csvGps.loc[~mask, 'AMP'] = np.nan
FullPowerAvg = csvFull['AMP'].mean()
FullPowerMax = csvFull['AMP'].max()
FullPowerMin = csvFull['AMP'].min()



print("FullPowerAvg = ", FullPowerAvg, "mA")
print("GPS Avg = ", csvGps['AMP'].mean())



plt.xlabel('Time[m]', fontsize=30)
plt.ylabel('mA', fontsize=30)
plt.tick_params(axis='y', labelsize=28)
plt.tick_params(axis='x', labelsize=28)
plt.title("Power Draw of Weather Station Over 1 Hour at 15 Minute Cycle",fontsize=35)



tomin = 1 / 1000 / 60

AvgLabel = f'Avg: +{FullPowerAvg:.1f} mA'
MaxLabel = f'Max: +{FullPowerMax:.1f} mA'
MinLabel = f'Min: +{FullPowerMin:.1f} mA'
plt.plot(csvFull['TIME'] * tomin, csvFull['AMP'], color='blue', alpha = 0.5)
plt.fill_between(csvFull['TIME'] *tomin, 0,0, color='blue', alpha=0.4, label=MaxLabel)
plt.fill_between(csvFull['TIME']* tomin, 0,0, color='blue', alpha=0.4, label=MinLabel)
plt.plot(csvGps['TIME']* tomin, csvGps['AMP'],color='lightblue', alpha = 0.4)

GpsAvg = csvGps['AMP'].mean() - FullPowerAvg
GpsMax = csvGps['AMP'].max() - FullPowerAvg
GpsAvgLabel = f'GPS Avg: +{GpsAvg:.1f} mA'
GpsMaxLabel = f'GPS Max: +{GpsMax:.1f} mA'
plt.fill_between(csvGps['TIME']*tomin, csvGps['AMP'],FullPowerAvg, color='lightblue', alpha=0.3, label=GpsAvgLabel)
plt.fill_between(csvGps['TIME']*tomin, 0,0, color='lightblue', alpha=0.3, label=GpsMaxLabel)
AvgExclGpsLabel = f'Avg Excl GPS: +{FullPowerAvg:.1f} mA'
plt.axhline(y=FullPowerAvg, color='red', alpha=0.6, label=AvgExclGpsLabel, linewidth = 4)
Avg = csvFuller['AMP'].mean()
AvgInclGpsLabel = f'Avg Incl GPS: +{Avg:.1f} mA'
plt.axhline(y=csvFuller['AMP'].mean(), color='orange', alpha=0.6, label=AvgInclGpsLabel, linewidth = 4)
plt.fill_between(csvGps['TIME']*tomin, FullPowerAvg,0, color='blue', alpha=0.2)

plt.legend(fontsize = 24, loc="upper right", frameon=True, shadow=True, fancybox=True)
plt.subplots_adjust(left=0.08, right=0.92, top=0.92, bottom=0.12)
plt.show()
