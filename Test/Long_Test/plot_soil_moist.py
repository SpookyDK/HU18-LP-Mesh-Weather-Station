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
our = pd.read_csv('./sensors.csv', sep=',', decimal=',')  # comma as separator, comma as decimal

our.columns = our.columns.str.strip()  # clean extra spaces

# Convert TIME column to datetime
our['TIME'] = pd.to_datetime(our['TIME']).dt.tz_localize(None)
our['PRESSURE'] = our['PRESSURE'].astype(float)
print(our.dtypes)

# Crop the dataframes
# Set TIME as index
# Calculate row-wise min and max
plt.figure(figsize=(16,10))
plt.xlabel('Time', fontsize = 24)
plt.ylabel('Humidity (RH%)', fontsize = 24)
plt.title('Pressure Measured From 2 Week Deployment Compared To DMI', fontsize = 30)
plt.plot(our['TIME'], our['SOIL MOIST'], label='Measured Soil Moisture', color='orange', linewidth=2)
plt.axhline()
# plt.ylim(1000, 1050)
plt.gca().xaxis.set_major_locator(mdates.HourLocator(interval=24))  # every 2 hours
plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%a:%H:%M')) # show HH:MM
plt.xticks(rotation=45, fontsize=20)  # x-axis values
plt.yticks(fontsize=24)               # y-axis values
plt.subplots_adjust(left=0.05, right=0.98, top=0.95, bottom=0.07, hspace=0.3)
plt.legend(
    fontsize=30,           # font size
    loc='upper right',     # legend position
    frameon=True,          # show box around legend
    shadow=True,           # shadow effect
    fancybox=True          # rounded corners
)
# our_cropped.set_index('TIME', inplace=True)
# dmi_cropped.set_index('DateTime', inplace=True)
#
# # 4. Resample to a common frequency (e.g., 10 minutes)
# # '10T' = 10 minutes. Use 'H' for hourly if data is sparse.
# # Change '10T' to '10min'
# our_res = our_cropped['PRESSURE'].resample('10min').mean()
# dmi_res = dmi_cropped['Lufttryk'].resample('10min').mean()
#
# # 5. Combine and calculate accuracy
# aligned = pd.concat([our_res, dmi_res], axis=1).dropna()
# aligned.columns = ['Measured', 'DMI']
#
# # Mean Absolute Error (lower is better)
# mae = (aligned['Measured'] - aligned['DMI']).abs().mean()
# # Accuracy as a percentage based on the mean value
# accuracy = 100 - (mae / aligned['DMI'].mean() * 100)
#
# print(f"Mean Absolute Error: {mae:.4f} hPa")
# print(f"Average Accuracy: {accuracy:.2f}%")
#
# # 1. Bias: Is your sensor consistently reading too high or too low?
# bias = (aligned['Measured'] - aligned['DMI']).mean()
#
# # 2. MAE: Total magnitude of inaccuracy (regardless of direction)
# mae = (aligned['Measured'] - aligned['DMI']).abs().mean()
#
# print(f"Bias: {bias:.2f} hPa") 
# # If Bias is +2.0, your sensor is likely just not calibrated (consistently high).
# # If Bias is 0.0 but MAE is 5.0, your sensor is "noisy" (unstable).
plt.show()

