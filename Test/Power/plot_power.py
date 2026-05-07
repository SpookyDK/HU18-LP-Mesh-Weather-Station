from os import wait
import matplotlib.pyplot as plt 
import numpy as np 
import pandas as pd 
import matplotlib.font_manager as fm

pagella_path = '/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyrepagella-regular.otf'

# Register it with Matplotlib
fm.fontManager.addfont(pagella_path)
plt.rcParams['font.family'] = 'TeX Gyre Pagella'

csvBare = pd.read_csv("./power_bare.csv")
csvBare1 = pd.read_csv("./power_bare.csv")
csvBare2 = pd.read_csv("./power_bare.csv")
csvFull = pd.read_csv("./power_full.csv") 
csvHall = pd.read_csv("./power_hall.csv")
csvTemp = pd.read_csv("./power_tempds.csv")
csvLight = pd.read_csv("./power_light.csv")
csvSoil = pd.read_csv("./power_soil.csv")




print(csvBare.dtypes)
print(csvFull.dtypes)
print(csvHall.dtypes)
print(csvTemp.dtypes)
print(csvLight.dtypes)
print(csvSoil.dtypes)

csvBare = csvBare[csvBare['TIME'].between(5000, 45000)]
csvBare1 = csvBare1[csvBare1['TIME'].between(5000, 45000)]
csvBare2 = csvBare2[csvBare2['TIME'].between(5000, 45000)]
csvBare3 = csvBare2[csvBare2['TIME'].between(5000, 45000)]

csvFull['TIME'] = csvFull['TIME'] + 4605
csvFull = csvFull[csvFull['TIME'].between(5000, 45000)]

csvHall['TIME'] = csvHall['TIME'] + 2000
csvHall = csvHall[csvHall['TIME'].between(5000, 45000)]

csvTemp['TIME'] = csvTemp['TIME'] + 0
csvTemp = csvTemp[csvTemp['TIME'].between(5000, 45000)]
csvLight['TIME'] = csvLight['TIME'] - 6100 - 20
csvLight = csvLight[csvLight['TIME'].between(5000, 45000)]

csvSoil['TIME'] = csvSoil['TIME'] + 1100 - 20
csvSoil = csvSoil[csvSoil['TIME'].between(5000, 45000)]
mask = (csvBare['TIME'].between(6634, 7745)) | \
       (csvBare['TIME'].between(16639, 17547)) | \
       (csvBare['TIME'].between(26635, 27540)) | \
       (csvBare['TIME'].between(36645, 37552))

mask2 = (csvBare['TIME'].between(6533, 7646)) | \
       (csvBare['TIME'].between(16538, 17648)) | \
       (csvBare['TIME'].between(26534, 27641)) | \
       (csvBare['TIME'].between(36544, 37651))
csvBare1.loc[mask, 'AMP'] = 36
csvBare2.loc[~mask2, 'AMP'] = np.nan
csvBare3.loc[~mask2, 'AMP'] = 0

# plt.plot(csvBare['TIME'], csvBare['AMP'], label=('BARE BOARD: Avg = ', 1, 'mA'))
# plt.plot(csvFull['TIME'], csvFull['AMP'], label='FULL')
# plt.plot(csvHall['TIME'], csvHall['AMP'], label='Hall')
# plt.plot(csvTemp['TIME'], csvTemp['AMP'], label='DS18B20')
# plt.plot(csvLight['TIME'], csvLight['AMP'], label='LIGHT')
# plt.plot(csvSoil['TIME'], csvSoil['AMP'], label='Soil Moist')

time_in_seconds = (csvBare['TIME'] - csvBare['TIME'].min())
time_in_hours = time_in_seconds / 36000

# 2. Calculate the Integrals (Area Under the Curve)
# Using the Trapezoidal Rule: np.trapz(y_values, x_values)
BarePower = np.trapezoid(csvBare['AMP'], x=time_in_hours)
FullPower = np.trapezoid(csvFull['AMP'], x=time_in_hours)
HallPower = np.trapezoid(csvHall['AMP'], x=time_in_hours)
TempPower = np.trapezoid(csvTemp['AMP'], x=time_in_hours)
LightPower = np.trapezoid(csvLight['AMP'], x=time_in_hours)
SoilPower = np.trapezoid(csvSoil['AMP'], x=time_in_hours)

BarePowerAvg = csvBare['AMP'].mean()
FullPowerAvg = csvFull['AMP'].mean()
HallPowerAvg = csvHall['AMP'].mean()
TempPowerAvg = csvTemp['AMP'].mean()
LightPowerAvg = csvLight['AMP'].mean()
SoilPowerAvg = csvSoil['AMP'].mean()


FullPowerDif = FullPowerAvg - BarePowerAvg 
HallPowerDif = HallPowerAvg - BarePowerAvg
TempPowerDif = TempPowerAvg - BarePowerAvg
LightPowerDif = LightPowerAvg - BarePowerAvg
SoilPowerDif = SoilPowerAvg - BarePowerAvg

print("BarePowerAvg = ", BarePowerAvg, "mA")
print("FullPowerAvg = ", FullPowerAvg, "mA")
print("HallPowerAvg = ", HallPowerAvg, "mA")
print("TempPowerAvg = ", TempPowerAvg, "mA")
print("LightPowerAvg = ", LightPowerAvg, "mA")
print("SoilPowerAvg = ", SoilPowerAvg, "mA")



print("BarePowerInt = ", BarePower)
print("FullPowerInt = ", FullPower)
print("HallPowerInt = ", HallPower)
print("TempPowerInt = ", TempPower)
print("LightPowerInt = ", LightPower)
print("SoilPowerInt = ", SoilPower)

print("FullPowerDif = ", FullPowerDif)
print("HallPowerDif = ", HallPowerDif)
print("TempPowerDif = ", TempPowerDif)
print("LightPowerDif = ", LightPowerDif)
print("SoilPowerDif = ", SoilPowerDif)

TotalAvg = (BarePowerAvg + HallPowerDif + TempPowerDif + LightPowerDif + SoilPowerDif)
print("Bare + All = ", TotalAvg)


plt.xlabel('Time[ms]', fontsize=30)
plt.ylabel('mA', fontsize=30)
plt.tick_params(axis='y', labelsize=28)
plt.tick_params(axis='x', labelsize=28)
plt.title("Avg Relative Current Draw of Different Parts on The Weather Station in Relation to The MainBoard",fontsize=35)


BareLabel = f'MainBoard Avg Current: {BarePowerAvg:.1f} mA'
plt.plot(csvBare1['TIME'], csvBare1['AMP'])
plt.fill_between(csvBare['TIME'], csvBare1['AMP'], 0, color = 'gray', alpha = 0.2, label=BareLabel)
LoraAvg = csvBare2['AMP'].mean()
LoraAvgAll = csvBare3['AMP'].mean()
# LoraLabel = f'LoRa Avg Current: +{LoraAvg:.1f} mA / Avg:+{LoraAvgAll:.1f} mA'
# plt.plot(csvBare2['TIME'], csvBare2['AMP'])
# plt.fill_between(csvBare2['TIME'], csvBare2['AMP'], 36, color = 'orange', alpha = 0.2, label=LoraLabel)


LightLabel = f'Light Sensor Avg Current: +{LightPowerDif:.1f} mA'
plt.plot(csvBare['TIME'], csvBare['AMP']+LightPowerDif, color='yellow')
plt.fill_between(csvBare['TIME'], csvBare['AMP']+LightPowerDif,csvBare['AMP'], color='yellow', alpha=0.2, label=LightLabel)

TempLabel = f'Ds18b20 Temp Avg Current: +{TempPowerDif:.1f} mA'
plt.plot(csvBare['TIME'], csvBare['AMP'] + LightPowerDif + TempPowerDif, color='blue')
plt.fill_between(csvBare['TIME'], csvBare['AMP']+LightPowerDif,csvBare['AMP']+LightPowerDif+TempPowerDif, color='blue', alpha=0.2, label=TempLabel)


SoilLabel = f'Soil Moisture Avg Current: +{SoilPowerDif:.1f} mA'
plt.plot(csvBare['TIME'], csvBare['AMP'] + SoilPowerDif + LightPowerDif + TempPowerDif, color='red')
plt.fill_between(csvBare['TIME'], csvBare['AMP']+LightPowerDif+TempPowerDif,csvBare['AMP']+LightPowerDif+TempPowerDif+SoilPowerDif, color='red', alpha=0.2, label=SoilLabel)

HallLabel = f'Hall Effect Avg Current: +{HallPowerDif:.1f} mA'
plt.plot(csvBare['TIME'], csvBare['AMP'] + SoilPowerDif + LightPowerDif + TempPowerDif + HallPowerDif, color='green')
plt.fill_between(csvBare['TIME'], csvBare['AMP']+LightPowerDif+TempPowerDif+SoilPowerDif,csvBare['AMP']+LightPowerDif+TempPowerDif+SoilPowerDif+HallPowerDif, color='green', alpha=0.2, label=HallLabel)

FullLabel = f'Full System Avg Current: +{TotalAvg:.1f} mA'

# plt.axhline(y=TotalAvg, color='purple', alpha=0.6,linestyle='--', label=FullLabel, linewidth = 2)
plt.xlim(5000,45000)
plt.ylim(0,150)
plt.legend(fontsize = 24, loc="upper right", frameon=True, shadow=True, fancybox=True)
plt.subplots_adjust(left=0.08, right=0.92, top=0.92, bottom=0.12)
plt.show()
