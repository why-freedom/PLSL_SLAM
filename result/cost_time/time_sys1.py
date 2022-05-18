#coding=utf-8
import matplotlib.pyplot as plt
import numpy as np
from sklearn.kernel_ridge import KernelRidge

#data1 = np.loadtxt('plsl-SolveTime.txt')
#data2 = np.loadtxt('pls-vio-SolveTime.txt')
#data3 = np.loadtxt('vins-SolveTime.txt')

# 1
data1 = np.loadtxt('plsl-FeatureTrackerTime.txt')
data2 = np.loadtxt('pls-vioFeatureTrackerTime.txt')
data3 = np.loadtxt('vins-FeatureTrackerTime.txt')

x1 = range(1,len(data1))
y1 = data1[1:len(data1)]

f1 = np.polyfit(x1, y1, 2)
p1 = np.poly1d(f1)
yvals = p1(x1)

clf = KernelRidge(alpha=0.1, kernel="rbf")
clf.fit(y1.reshape(-1, 1), x1)
y_2 = clf.predict(x1.reshape(-1,1))



y2 = data2[1:len(data2)]
x2 = range(1,len(y2)+1)
 
f2 = np.polyfit(x2, y2, 2)
p2 = np.poly1d(f2)
yvals2 = p1(x2)


y3 = data3[1:len(data3)]
x3 = range(1,len(y3)+1)
 
f3 = np.polyfit(x3, y3, 2)
p3 = np.poly1d(f3)
yvals3 = p1(x3)


#print(data)
plt.plot(data1[:], 'w')
plt.plot(x1, yvals, 'r',label='plsl-slam')

plt.plot(data2[:], 'w')
plt.plot(x2, yvals2, 'b',label='pls-vio')

plt.plot(data3[:], 'w')
plt.plot(x3, yvals3, 'g',label='vins-fusion')

plt.xlabel("Frame")
plt.ylabel("time/ms")

#plt.title("FeatureTracker Time Cost Conparison")
plt.title("BackEnd Time Cost Conparison")

plt.legend()
plt.show()

