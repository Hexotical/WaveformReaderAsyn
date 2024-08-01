import pandas as pd
import matplotlib.pyplot as plt
import re

screenlog_path = '/nfs/slac/g/lcls/epics/ioc/data/sioc-b34-mp12/screenlog.0'
csv_output_path = 'output.csv'

# Read the screenlog file
with open(screenlog_path, 'r') as file:
    lines = file.readlines()

# Find the most recent 'fourierTransform' command
fourier_command = 'fourierTransform'
start_index = None
for i in range(len(lines) - 1, -1, -1):
    if fourier_command in lines[i]:
        start_index = i
        break

if start_index is None:
    raise ValueError(f"'{fourier_command}' command not found in the screenlog file.")


# Extract CSV data from the screenlog
csv_data = []
csv_pattern = re.compile(r'^\s*\d+(\.\d+)?([eE][+-]?\d+)?\s*,\s*\d+(\.\d+)?([eE][+-]?\d+)?\s*$') 

for line in lines[start_index + 1:]:
    if csv_pattern.match(line):
        csv_data.append(line.strip())

if not csv_data:
    raise ValueError("No CSV data found after the most recent 'fourierTransform' command.")

print("Plotting csv data now")

# Write the extracted CSV data to a file with header
with open(csv_output_path, 'w') as file:
    file.write('Frequency,Magnitude\n')  # Add header
    file.write('\n'.join(csv_data))

# Load the CSV data into a DataFrame
df = pd.read_csv(csv_output_path)

# Plot the data
plt.figure(figsize=(10, 8))
plt.plot(df['Frequency'], df['Magnitude'], marker='o', linestyle='-', markersize=2, linewidth=0.5)
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude')
plt.title('FFT Output')
plt.grid(True)
plt.show()
