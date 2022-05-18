#coding=utf-8
import matplotlib.pyplot as plt
import numpy as np
 

def show(data, yaix, lstr):
	Ix = data[0:len(data)]
	Iy = np.empty(len(data))
	Iy.fill(yaix)
	plt.scatter(Ix, Iy, s=0.01)
        plt.plot(Ix, Iy, 'o-', label=lstr)
	plt.legend()



data0 = np.loadtxt('time_imgL.txt')
data1 = np.loadtxt('time_imgR.txt')
dataI = np.loadtxt('time_imu.txt')
dataL = np.loadtxt('time_lidar.txt')

show(data0, 1, "Left Image")
show(data1, 1.2, "Right Image")
show(dataL, 1.4, "Lidar")
show(dataI, 1.6, "IMU")



#print(data)
#plt.plot(data0[:], label='imgL')
#plt.plot(Ix, Iy, '.', label='imgR')
#plt.plot(dataI[:], label='imu')
#plt.plot(dataL[:], label='lidar')

plt.xlabel("time/s")

plt.title("time stamp")

plt.legend()

plt.show()

