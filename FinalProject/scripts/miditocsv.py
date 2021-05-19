import csv
import py_midicsv as pm
import sys

# Load the MIDI file and parse it into CSV format
csv_string = pm.midi_to_csv(sys.argv[1] + ".mid")

# Write new CSV file
with open(sys.argv[1] + ".csv", "w") as f:
    f.writelines(csv_string)