import os
import pandas as pd
import matplotlib.pyplot as plt

# Path to CSV
file_path = os.path.join('.', 'data', 'data.csv')

# Read data
df = pd.read_csv(file_path)

# Ensure 'time' exists
if 'time' not in df.columns:
    raise ValueError("CSV must contain a 'time' column.")

# Loop through all columns except 'time'
for col in df.columns:
    if col == 'time':
        continue

    # Skip non-numeric columns automatically
    if not pd.api.types.is_numeric_dtype(df[col]):
        continue

    plt.figure()
    plt.plot(df['time'], df[col])

    # Title = header name (future-proof)
    plt.title(col)

    plt.xlabel('time')
    plt.ylabel(col)

    plt.grid()
    plt.tight_layout()
    plt.show()