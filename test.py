import numpy as np
import matplotlib.pyplot as plt
# import pdb
# pdb.set_trace()

STEP = 0.1
PACKET = 1024
VINDEX = 0

# file11 = '/home/yu/ndnsim/ndnSIM-ieacc/ourcode/ieacc/log/state-64-all.txt'
# file12 = '/home/yu/ndnsim/ndnSIM-ieacc/ourcode/ieacc/log/papertest/bwvary-1.0-sinr-1.txt'
# file13 = '/home/yu/ndnsim/ndnSIM-ieacc/ourcode/ieacc/log/state-91-all.txt'
# file1 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0(Rx=-72.50).log'
# file2 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0(Rx=-72.60).log'
# file3 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0(Rx=-72.70).log'
# file4 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0(Rx=-72.80).log'
# file5 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0(Rx=-72.90).log'
# file6 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0(Rx=-73.00).log'
# file7 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0(Rx=-73.10).log'
# file8 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0(Rx=-73.20).log'
# file9 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0(Rx=-73.30).log'
# file10 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0(Rx=-73.40).log'
# file11 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0(Rx=-73.50).log'


file = '/home/whd/ndnSIM2.8/wireless-macspec/delay0.log'
# file2 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0_main.log'
# file3 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0_simple.log'

x1, y1, x, y = [], [], [],[]
# files = [file1,file2,file3,file4,file5,file6,file7,file8,file9,file10,file11]
# for file in files :
aimds = open(file, 'r').readlines()[2:]
 #   + open(file12, 'r').readlines() \
 # + open(file13, 'r').readlines()
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

datasum = 0
index = 2
for line in (aimds):
    value = line.split('\t')
    if (float(value[0]) < index):
        if (int(value[7]) >= 1):
            datasum += 1 
    else:
        # x.append(index)
        y.append(float(datasum * PACKET * 4)/(2 * 1000 * 1000))
        datasum = 0
        index += 2

x = np.arange(69.00, 73.00, 0.1)

plt.figure(figsize=(16.0, 12.0))
# plt.figure(figsize=(10,8))
plt.rcParams['font.size'] = 16
f1= plt.plot(x1, y1, color='blue', label='Continus', ls=':')
plt.xlabel("time (s)")
plt.ylabel("Data transmission rate (Mbps)")
f2, ax1 = plt.subplots()
f2.set_size_inches(16,12)
ax1.plot(x, y[0:len(x)], color='red', ls='-.')
ax1.grid(axis="y", linestyle='-.')
ax1.set_xlim(x[0],x[-1])
ax1.set_xlabel("Loss (dBm)")
ax1.set_ylabel("Data transmission rate (Mbps)")
snr = [90.96-i for i in x]
print(snr)
ax2 = ax1.twiny()
ax2.set_xlim(90.96-x[0],90.96-x[-1])
ax2.plot(snr, y)
ax2.set_xlabel('SNR(dB)')
ax2.spines['top'].set_visible(False)
ax2.spines['right'].set_visible(False)
ax2.spines['bottom'].set_visible(True)
# f2 = plt.plot(x2, y2, color='#1A6FDF', label='sta1', ls=':')
#f3, = plt.plot(x3, y3, color='#37AD6B', label='DRL-CCP', ls='dashed')  #B177DE
#f4, = plt.plot(x4, y4, color='#CC9900', label='IEACC', ls='dotted')
#f5, = plt.plot(xxx, yyy, color='slategrey', label='Bandwidth', ls='-')
#ax.plot(y1,color='black',linestyle='-')

#plt.legend(handles=[f1, f2, f3, f4],
#           labels=['ICP', 'ECP', 'DRL-CCP', 'IEACC'],
#           loc='lower right')
#plt.ylim((3,10))
#ax.legend()
#plt.grid(axis="y")



#plt.xlim((0,6000))
plt.savefig('./test.png')
plt.show()