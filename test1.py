import matplotlib.pyplot as plt
from itertools import zip_longest
# import pdb
# pdb.set_trace()

STEP = 0.1
PACKET = 1024
VINDEX = 0

file1 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0.log'
file2 = '/home/whd/ndnSIM2.8/wireless-macspec/delay1.log'
file3 = '/home/whd/ndnSIM2.8/wireless-macspec/delay2.log'
file4 = '/home/whd/ndnSIM2.8/wireless-macspec/delay3.log'

plt.figure(figsize=(12.0, 10.0))
plt.rcParams['font.size'] = 16
files  = [file1, file2]
colors = ['b','c','m','y']
lines = ['-.','--',':','dashed']
i=1
x, y_sum = [],[]
for file in files:
    x1,y1 = [], []
    aimds = open(file, 'r').readlines()[2:]
    datanum = 0
    index = STEP
    for line in (aimds):
        value = line.split('\t')
        if (float(value[0]) < index):
            if (int(value[7]) >= 1):
                datanum += 1
        else:
            x1.append(index)
            y1.append(float(datanum * PACKET * 4)/(STEP * 1000 * 1000))
            datanum = 0
            index += STEP
    x.append(x1)
    y_sum.append(y1)
    plt.plot(x1, y1, label='adhoc'+str(i), color = colors[i], ls=lines[i])
    plt.legend()
    i += 1
x =  x[0]  if  len(x[0])>len(x[1]) else x[1]
y = [sum(v) for v in zip_longest(*y_sum,fillvalue=0)]
# l = min(len(x),len(y))
plt.plot(x,y, label ='sum', color = 'r', ls = '-')
plt.legend()
plt.xlabel("time (s)")
plt.ylabel("Data transmission rate (Mbps)")
plt.savefig('./Adhoc.png')
plt.show()
