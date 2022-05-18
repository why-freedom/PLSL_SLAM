#coding=utf-8
import matplotlib.pyplot as plt
import numpy as np

# 1
data1 = np.loadtxt('plsl-FeatureTrackerTime.txt')


x1 = range(1,len(data1))
y1 = data1[1:len(data1)]

f1 = np.polyfit(x1, y1, 0)
p1 = np.poly1d(f1)
yvals = p1(x1)




#print(data)
plt.plot(x1, yvals, 'r',label='plsl-slam')


plt.xlabel("Frame")
plt.ylabel("time/ms")

#plt.title("FeatureTracker Time Cost Conparison")
plt.title("BackEnd Time Cost Conparison")

plt.legend()
plt.show()

