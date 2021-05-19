import csv
import py_midicsv as pm
from itertools import chain
import sys

data = []
# Open CSV file and read it
csvfile = open (sys.argv[1] + '_sort_dur.csv','r')
reader = list(csv.reader(csvfile))

# Create rows that represent gaps in vocal melody
for i in range(len(reader)-1):
    dur = int(reader[i][0]) + int(reader[i][3])
    new_list=[]
    new_list.append(int(reader[i][0]))
    new_list.append(int(reader[i][1]))
    new_list.append(int(reader[i][3]))
    new_list.append('')
    data.append(new_list)
    if int(reader[i][1]) < 127 and dur < int(reader[i+1][0]):
        new_list=[]
        new_list.append(dur)
        new_list.append(0)
        new_list.append(int(reader[i+1][0]) - dur)
        new_list.append('')
        data.append(new_list)

# Add last row
new_list=[]
new_list.append(int(reader[len(reader)-1][0]))
new_list.append(int(reader[len(reader)-1][1]))
new_list.append(int(reader[len(reader)-1][3]))
new_list.append('')
data.append(new_list)

# Sort notes by timing
sorted = sorted(data, key = lambda x: int(x[0]))

# Place brackets around rows
for row in sorted:
        row[0] = '{' + str(row[1])
        row[1] = str(row[2]) + '}'
        del row[2]

# Write new CSV file
with open(sys.argv[1] + '_sort_dur_vocal.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(sorted)