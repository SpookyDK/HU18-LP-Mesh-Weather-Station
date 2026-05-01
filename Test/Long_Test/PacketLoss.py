import pandas as pd
import numpy as np

# 1. Load and Sort
df = pd.read_csv('./CollectedLargeFormatted.csv')
df['TIME'] = pd.to_datetime(df['TIME'])
df = df.sort_values('TIME')

# 2. Parameters
INTERVAL_SEC = 300  # Exactly 5 minutes

gaps = []
total_received = len(df)

# 3. Group by Node (Important if you have multiple nodes in one file)
for node_id, node_group in df.groupby('NODE ID'):
    node_group = node_group.sort_values('TIME')
    
    # Calculate difference between this packet and the previous one
    node_group['diff'] = node_group['TIME'].diff().dt.total_seconds()
    
    # We look for any gap significantly larger than 5 minutes (e.g., > 7 mins)
    # This accounts for small network delays (jitter)
    missing_mask = node_group['diff'] > (INTERVAL_SEC + 60)
    missing_data = node_group[missing_mask]

    for idx, row in missing_data.iterrows():
        # Calculate loss: (Total gap / 5 mins) rounded to nearest whole number - 1
        # Example: 610s / 300s = 2.03 -> 2 intervals occurred, 1 was lost.
        missed_count = int(round(row['diff'] / INTERVAL_SEC)) - 1
        
        if missed_count > 0:
            gaps.append({
                'Node': node_id,
                'Last_Seen': node_group.loc[node_group.index[node_group.index.get_loc(idx)-1], 'TIME'],
                'Resume_Time': row['TIME'],
                'Gap_Minutes': round(row['diff'] / 60, 2),
                'Packets_Lost': missed_count
            })

# 4. Final Calculations
gaps_df = pd.DataFrame(gaps)
total_lost = gaps_df['Packets_Lost'].sum() if not gaps_df.empty else 0
total_expected = total_received + total_lost
loss_percent = (total_lost / total_expected) * 100

print(f"Analysis complete for {total_received} received packets.")
print(f"Total Packets Lost: {total_lost}")
print(f"Packet Loss Rate:   {loss_percent:.2f}%")

# Optional: export the gaps to a CSV to see exactly when the network died
# gaps_df.to_csv('detected_gaps.csv', index=False)
