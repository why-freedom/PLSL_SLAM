evo_ape tum groundtruth_long.txt plsl-slam.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/1PLSL-SLAM.zip

evo_ape tum groundtruth_long.txt pls-vio.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/2PLS-VIO.zip

evo_ape tum groundtruth_long.txt vins-fusion.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/3VINS-Fusion.zip

evo_ape tum groundtruth_long.txt aloam.txt -va --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/4ALOAM.zip


