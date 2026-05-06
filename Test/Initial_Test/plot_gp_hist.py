import matplotlib.pyplot as plt
import pandas as pd
import matplotlib as mpl
import matplotlib.font_manager as fm
from matplotlib.lines import Line2D

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

# Ensure 'GPS DIF' is numeric
our['GPS DIF'] = pd.to_numeric(our['GPS DIF'], errors='coerce')
our = our.dropna(subset=['GPS DIF'])

# Calculate the statistics for labeling
min_val = our['GPS DIF'].min()
max_val = our['GPS DIF'].max()
median = our['GPS DIF'].median()
q1 = our['GPS DIF'].quantile(0.25)
q3 = our['GPS DIF'].quantile(0.75)

# Create the Boxplot
plt.figure(figsize=(16, 10))

bp = plt.boxplot(our['GPS DIF'], 
            vert=False, 
            patch_artist=True, 
            widths=0.5,
            boxprops=dict(facecolor='orange', color='black'),
            medianprops=dict(color='black', linewidth=2),
            flierprops=dict(marker='o', markerfacecolor='orange', alpha=0.5))

# --- Add Labels ---
# text_offset moves the text slightly above or below the plot line (1 is the y-center)
text_offset = 0.35 

plt.text(min_val, 3 + text_offset, f'Min: {min_val:.2f}', ha='center', va='bottom', fontsize=24)
plt.text(max_val, 1 + text_offset, f'Max: {max_val:.2f}', ha='center', va='bottom', fontsize=24)
plt.text(median, 1 + text_offset, f'Median: {median:.2f}', ha='center', va='bottom', fontsize=28, fontweight='bold')
plt.text(q1, 1 - text_offset, f'Q1: {q1:.2f}', ha='center', va='top', fontsize=24)
plt.text(q3, 1 - text_offset, f'Q3: {q3:.2f}', ha='center', va='top', fontsize=24)

# Formatting
plt.xlabel('Distance (M)', fontsize=24)
plt.ylabel('', fontsize=24)
plt.title('Distribution of Distance from Reported to Actual Positions', fontsize=30)
plt.xticks(fontsize=24)
plt.yticks([]) 
plt.grid(axis='x', linestyle='--', alpha=0.7)

# Legend
legend_elements = [Line2D([0], [0], color='orange', lw=4, label='GPS Distance Distribution')]
plt.legend(handles=legend_elements, fontsize=28, loc='upper right', frameon=True, shadow=True, fancybox=True)

plt.tight_layout()
plt.savefig('GPS_Boxplot_Labeled.png')
plt.show()
