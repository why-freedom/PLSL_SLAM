#coding=utf-8
import matplotlib.pyplot as plt
import numpy as np
 
dataS = np.loadtxt('depth.txt')
dataL = np.loadtxt('lidar-depth.txt')

#print(data)
plt.plot(dataS[:], label='depth of visual trangulate')
plt.plot(dataL[:], label='depth of liadr and visual data association')

plt.xlabel("Number of Features Points")
plt.ylabel("detph/m")

plt.title("Depth Value Conparison")

plt.legend()

plt.show()

