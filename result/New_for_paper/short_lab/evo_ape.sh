evo_ape tum groundtruth.txt pls-vio.csv -va --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/1PLS-VIO.zip
evo_ape tum groundtruth.txt vins-fusion.csv -va --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/2VINS-Fusion.zip
evo_ape tum groundtruth.txt vins-mono.csv -va --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/3VINS-Mono.zip
evo_ape tum groundtruth.txt pl-vio.txt -va --t_offset=0.3 --t_max_diff=0.3 --plot --plot_mode xy --save_results result/4PL-VIO.zip

