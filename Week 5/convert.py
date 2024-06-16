import pandas as pd
import io
import re

# Open your file and read it as a single string.
with open('highway_map.csv', 'r') as file:
    data = file.read()

# Replace all characters that are not digits, '.', '-' or '\n' with a space.
clean_data = re.sub(r'[^\d.\n-]+', ' ', data)

# Use io.StringIO to create a file-like object that pandas can read from.
df = pd.read_csv(io.StringIO(clean_data), sep=' ', header=None)

# Drop the NaN columns
df = df.dropna(axis=1)

# Rename the columns
df.columns = ['x', 'y', 'z', 'theta', 'phi']

# Print the first few rows of the DataFrame
print(df.head())

df.to_csv('highway_map.csv', index=False)