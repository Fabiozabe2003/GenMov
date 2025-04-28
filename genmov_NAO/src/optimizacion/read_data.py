import pandas as pd 

df_read = pd.read_csv("/home/fabiozabe/proy_ws/src/GenMov/genmov_NAO/src/optimizacion/motions/move_arms.csv")
#print("Read data:");print(df_read.head())

def reconstruct():
    reconstructed_data = {}
    for body_part in df_read["body_part"].unique():
        body_part_data = df_read[df_read["body_part"] == body_part]
        body_part_array = body_part_data[["x", "y", "z"]].values
        reconstructed_data[body_part] = body_part_array
    return reconstructed_data