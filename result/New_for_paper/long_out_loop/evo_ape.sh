# evo_ape tum groundtruth_outlong.txt vio.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/VIO.zip

# evo_ape tum groundtruth_outlong.txt pls-vio.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/PLS-VIO.zip

# evo_ape tum groundtruth_outlong.txt plsl-no-lidar.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/PLSL-No-Lidar.zip

# evo_ape tum groundtruth_outlong.txt aloam.txt -va --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/ALOAM.zip

# evo_ape tum groundtruth_outlong.txt plsl-slam.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/PLSL-SLAM.zip

# evo_ape tum groundtruth_outlong.txt plsl-loop.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/PLSL-Loop.zip

# evo_ape tum groundtruth_outlong.txt vins-fusion.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/VINS-Fusion.zip

evo_ape tum groundtruth_outlong.txt orbslam3.txt -va -s --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/ORBSLAM3.zip
