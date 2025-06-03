import pandas as pd 
import sys
#dir=sys.argv[1]
#df_read = pd.read_csv("/home/andres/proy_ws/src/GenMov/genmov_NAO/src/motions/{}.csv".format(dir))
df_read = pd.read_csv("/home/invitado8/proy_ws/src/GenMov/genmov_NAO/src/inversion_matriz/move_arms.csv")

def reconstruct():
    reconstructed_data = {}
    for body_part in df_read["body_part"].unique():
        body_part_data = df_read[df_read["body_part"] == body_part]
        body_part_array = body_part_data[["x", "y", "z"]].values
        reconstructed_data[body_part] = body_part_array
    return reconstructed_data