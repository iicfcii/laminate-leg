import csv
import itertools
import numpy as np

def write(filename, keys, values):
    assert len(keys) == len(values)

    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(keys)

        for vs in itertools.zip_longest(*values):
            writer.writerow(vs)

def read(filename):
    keys = []
    values = []
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for i, row in enumerate(reader):
            if i == 0:
                for key in row:
                    if key != '':
                        keys.append(key)
                        values.append([])
            else:
                for j, v in enumerate(row):
                    if j < len(keys):
                        if v == '':
                            values[j].append(None)
                        else:
                            values[j].append(float(v))

    data = {}
    for k,v in zip(keys,values):
        data[k] = v

    return data

def detect_change(d,window=10,threshold=0.05):
    d_init = np.average(d[0:window])
    for i in range(len(d)):
        d_init_c = np.average(d[i:i+window])
        if np.abs(d_init_c-d_init)/np.abs(d_init) > threshold:
            break

    return i
