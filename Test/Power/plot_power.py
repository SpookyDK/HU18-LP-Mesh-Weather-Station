import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
csvBare = pd.read_csv("./power_bare.csv")
print(csvBare.dtypes)

csvFull = pd.read_csv("./power_full.csv")
csvFull = csvFull[(csvFull['TIME'] <= 53000) & (csvFull['TIME'] >= 13000)]
print(csvFull.dtypes)

plt.plot(csvBare['TIME'], csvBare['AMP'], label="Only MainBoard")
plt.plot(csvFull['TIME']-800, csvFull['AMP'], label="With all Ext Sensors")
plt.legend()
plt.show()

bare_time_in_seconds = (csvBare['TIME'] - csvBare['TIME'].min())
bare_time_in_hours = bare_time_in_seconds / 3600
powerBareArea = np.trapezoid(csvBare['AMP'], x=bare_time_in_hours)

full_time_in_seconds = (csvFull['TIME'] - csvFull['TIME'].min())
full_time_in_hours = full_time_in_seconds / 3600
powerFullArea = np.trapezoid(csvFull['AMP'], x=full_time_in_hours)
print("BARE = ", powerBareArea,"mA")
print("FULL = ", powerFullArea, "mA")



