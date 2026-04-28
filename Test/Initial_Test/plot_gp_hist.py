import matplotlib.pyplot as plt
import pandas as pd
import matplotlib as mpl
import matplotlib.font_manager as fm

# Load the custom font
pagella_path = '/usr/share/texmf/fonts/opentype/public/tex-gyre/texgyrepagella-regular.otf'
try:
    fm.fontManager.addfont(pagella_path)
    plt.rcParams['font.family'] = 'TeX Gyre Pagella'
except:
    print("Font not found, using default.")

# Read CSV correctly
our = pd.read_csv('Collected.csv', sep=',', decimal=',') 
our.columns = our.columns.str.strip() 

# Ensure 'GPS DIF' is numeric (important for histogram)
our['GPS DIF'] = pd.to_numeric(our['GPS DIF'], errors='coerce')
our = our.dropna(subset=['GPS DIF'])

# Create the Histogram
plt.figure(figsize=(16, 10))

# Change: Use plt.hist instead of plt.plot
# bins=20 controls the number of bars; edgecolor makes them distinct
plt.hist(our['GPS DIF'], bins=20, color='orange', edgecolor='black', alpha=0.7, label='GPS Distance Distribution')

# Update Labels and Title
plt.xlabel('Distance (M)', fontsize=24)
plt.ylabel('Frequency (Count)', fontsize=24)
plt.title('Distribution of Distance from Reported to Actual Positions', fontsize=25)

# Formatting
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
plt.grid(axis='y', linestyle='--', alpha=0.7)

plt.legend(
    fontsize=20,
    loc='upper right',
    frameon=True,
    shadow=True,
    fancybox=True
)

plt.tight_layout()
plt.savefig('GPS_Histogram.png')
plt.show()
