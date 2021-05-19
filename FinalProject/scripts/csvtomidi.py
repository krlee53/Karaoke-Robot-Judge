import py_midicsv as pm
import sys

# Parse the CSV output of the previous command back into a MIDI file
midi_object = pm.csv_to_midi(sys.argv[1] + ".csv")

# Save the parsed MIDI file to disk
with open(sys.argv[1] + "_raw.mid", "wb") as output_file:
    midi_writer = pm.FileWriter(output_file)
    midi_writer.write(midi_object)