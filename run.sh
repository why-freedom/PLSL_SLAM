source devel/setup.bash

xterm -e " roslaunch plsl vins_rviz.launch " &

sleep 2
xterm -e " rosrun plsl plsl_slam src/plsl-slam/config/mynteye-d/mynt_stereo_imu_lidar_config.yaml " &

#sleep 2
#xterm -e " rosrun plsl_loop plsl_loop src/plsl-slam/config/mynteye-d/mynt_stereo_imu_lidar_config.yaml "
#xterm -e " rosrun vins scanRegistration " &

#sleep 2
#xterm -e " rosrun vins laserMapping " 


