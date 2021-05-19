import csv
import py_midicsv as pm
from itertools import chain
import sys

data=[]
# Open CSV file and read it
csvfile = open (sys.argv[1] + '.csv','r')
reader = csv.reader(csvfile)
# Only add note and tempo lines
for row in reader:
    if row[2] != " Control_c":
        if row[2] == " Note_on_c" or row[2] == " Note_off_c":
            new_list = list(chain(row[1:2], row[4:6]))
            data.append(new_list)
        elif row[2] == " Tempo":
            new_list = list(chain(row[1:2], row[3:4]))
            new_list.append(0)
            data.append(new_list)

# Sort file based on timing
sorted = sorted(data, key = lambda x: int(x[0]))

# Write new CSV file
with open(sys.argv[1] + '_sort.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(sorted)