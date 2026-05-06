from os import wait
import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.dates as mdates
import matplotlib.font_manager as fm

# Font setup
pagella_path = '/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyrepagella-regular.otf'
fm.fontManager.addfont(pagella_path)
plt.rcParams['font.family'] = 'TeX Gyre Pagella'

# 1. Load Data
our = pd.read_csv('./rain_pulse.csv', sep=',', decimal=',')  
our.columns = our.columns.str.strip()  

# 2. Calculations
actual_value = 400
our['measured'] = our['PULSE'] * 0.2 * 41.77
our['diff'] = our['measured'] - actual_value

# 3. Plotting
plt.figure(figsize=(16,10))

# Create a color list based on the difference
colors = ['red' if x < 0 else 'green' for x in our['diff']]

# 1. Plot the main measured bars
our_avg = our['measured'].mean()
label_our = f"Measured Water Amount {our_avg:+.1f}ml"
plt.bar(our['ID'], our['measured'], 
        label=label_our, 
        color='blue', 
        alpha=0.4,
        edgecolor='black', 
        linewidth=1.5)

# 2. Plot the "Difference" bars starting from the 400 line
dif_avg = our['diff'].mean()
label_dif = f"Difference {dif_avg:+.1f}ml"
plt.bar(our['ID'], our['diff'], 
        bottom=actual_value, 
        color=colors, 
        edgecolor='black',
        linewidth=1.5,
        label=label_dif)

# 3. Add text labels for the difference
for i in range(len(our)):
    diff_val = our['diff'].iloc[i]
    measured_val = our['measured'].iloc[i]
    
    # Format text (e.g., +2.4ml or -5.1ml)
    label_text = f"{diff_val:+.1f}ml"
    
    # Determine vertical position: 
    # If overshooting, put text above the bar. If undershooting, put it below the 400 line.
    y_pos = measured_val + 5 if diff_val >= 0 else measured_val - 25
    
    plt.text(our['ID'].iloc[i], 
             y_pos, 
             label_text, 
             ha='center', 
             fontsize=36, 
             fontweight='bold',
             color='darkgreen' if diff_val >= 0 else 'darkred')

plt.axhline(y=actual_value, color='black', linestyle='-', linewidth=3, label='Actual Amount: 400ml')

# Formatting
plt.xlabel('Test ID', fontsize=30)
plt.ylabel('ml water', fontsize=36)
plt.title('Measured Water Amount Through Bucket Gauge', fontsize=32)
plt.xticks(our['ID'])
plt.xticks(rotation=45, fontsize=24)
plt.yticks(fontsize=24)

# Set y-limit slightly higher than the max value to make room for labels
plt.ylim(0, max(our['measured'].max(), actual_value) + 100)

plt.legend(fontsize=28, loc='upper right', frameon=True, shadow=True)
plt.tight_layout()
plt.show()
