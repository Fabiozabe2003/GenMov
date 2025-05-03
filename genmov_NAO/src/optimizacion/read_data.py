import pandas as pd 
import os

#print("Read data:");print(df_read.head())

def reconstruct(filename):
    base_path = "/home/invitado8/proy_ws/src/GenMov/genmov_NAO/src/optimizacion/motions"
    full_path = os.path.join(base_path, filename)
    df_read = pd.read_csv(full_path)
    reconstructed_data = {}
    for body_part in df_read["body_part"].unique():
        body_part_data = df_read[df_read["body_part"] == body_part]
        body_part_array = body_part_data[["x", "y", "z"]].values
        reconstructed_data[body_part] = body_part_array
    return reconstructed_data