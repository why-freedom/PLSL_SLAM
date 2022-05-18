evo_ape tum groundtruth.csv pls-vio.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/PLS-VIO.zip

#evo_ape tum groundtruth.csv pl-vio.txt -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/PL-VIO.zip

evo_ape tum groundtruth.csv vins-mono.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/VINS-MONO.zip

evo_ape tum groundtruth.csv vins-fusion.csv -va  --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/VINS-Fusion.zip

