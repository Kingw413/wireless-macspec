STEP = 0.1
PACKET = 1024
VINDEX = 0

# file11 = '/home/yu/ndnsim/ndnSIM-ieacc/ourcode/ieacc/log/state-64-all.txt'
# file12 = '/home/yu/ndnsim/ndnSIM-ieacc/ourcode/ieacc/log/papertest/bwvary-1.0-sinr-1.txt'
# file13 = '/home/yu/ndnsim/ndnSIM-ieacc/ourcode/ieacc/log/state-91-all.txt'
file1 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0.log'
file2 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0_main.log'
file3 = '/home/whd/ndnSIM2.8/wireless-macspec/delay0_simple.log'

x1, y1 = [], []
aimds = open(file1, 'r').readlines()[2:] 
      #+ open(file12, 'r').readlines() \    
      #+ open(file13, 'r').readlines()
datanum = 0
index = STEP
for line in (aimds):
    # print(line)
    value = line.split('\t')
    # print(value)
    #print("value[0]",value[0])
    # print(value[0])
    if (float(value[0])<index):
        if(int(value[7])>=1):
            datanum+=1
    else:
        x1.append(index)
        y1.append(float(datanum * PACKET * 4)/(STEP * 1000 * 1000))
        datanum =0
        
        index +=STEP


import matplotlib

matplotlib.use('cairo')
import matplotlib.pyplot as plt
#plt.plot([Y_1,Y_2,Y_3])
#fig,ax = plt.subplots()
#x = np.linespace(0,200,100)
#x = np.linspace(0, 50, 50)

#plt.figure(figsize=(9.0, 4.5))

f1, = plt.plot(x1, y1, color='red', label='ICP', ls='-.')
#f2, = plt.plot(x2, y2, color='#1A6FDF', label='ECP', ls=':')
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

plt.grid(axis="y", linestyle='-.')
plt.xlabel("time (s)")
plt.ylabel("Data transmission rate (Mbps)")

#plt.xlim((0,6000))
plt.savefig('./bwvary-1.0-sinr-1.png')
#plt.show()
