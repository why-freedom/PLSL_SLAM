import pandas as pd
f = pd.read_csv("data.csv")
keep_col  =['#timestamp', ' p_RS_R_x [m]',' p_RS_R_y [m]',' p_RS_R_z [m]',' q_RS_w []',' q_RS_x []',' q_RS_y []',' q_RS_z []']
new_f = f[keep_col]
new_f.to_csv("MH_03_tum.csv",index=False)

