import csv
import py_midicsv as pm
from itertools import chain
import sys

data=[]
active_notes = []
# Open CSV file and read it
csvfile = open (sys.argv[1] + '_sort.csv','r')
reader = csv.reader(csvfile)
# Calculate duration of each note
for row in reader:
    if int(row[2]) > 0:
        active_notes.append((int(row[0]),int(row[1]), int(row[2])))
    elif int(row[1]) > 127:
        new_list=[]
        new_list.append(int(row[0]))
        new_list.append(int(row[1]))
        new_list.append(int(row[2]))
        new_list.append(0)
        new_list.append('')
        data.append(new_list)
    else:
        for t in active_notes:
            if t[1] == int(row[1]):
                new_list=[]
                new_list.append(t[0])
                new_list.append(t[1])
                new_list.append(t[2])
                dur = int(row[0]) - t[0]
                new_list.append(dur)
                new_list.append('')
                data.append(new_list)
                active_notes.remove(t)
                break

# Sort notes by timing
sorted = sorted(data, key = lambda x: int(x[0]))

# Check if is background music
if sys.argv[2] == 'bg':
    # Place brackets around rows
    for row in sorted:
        row[0] = '{' + str(row[0])
        row[1] = str(row[1])
        row[2] = str(row[3]) + '}'
        del row[3]

# Write new CSV file
with open(sys.argv[1] + '_sort_dur.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(sorted)