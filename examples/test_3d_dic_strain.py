import os
import sys
import numpy as np
opencorrpy_rel_path = '../../bin'
module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), opencorrpy_rel_path))
sys.path.append(module_dir)
import opencorrpy as ocpy

def update_progress(msg):
    print(msg)

folder_path = '3d_dic'
cam0_tar_image_path = os.path.abspath(os.path.join(folder_path,"GT4-0273_0.tif"))
dic_result_file_path = os.path.abspath(os.path.join(folder_path,"GT4-0273_0_epipolar_sift_py.csv"))
strain_out_time_file_path = os.path.abspath(os.path.join(folder_path,"GT4-0273_0_strain_time_py.csv"))

config_data = {}
config_data["strain_radius"] = 20.0
config_data["min_neighbors"] = 5

if os.path.exists(dic_result_file_path):
    ocpy.dic_3d_strain(update_progress,cam0_tar_image_path,dic_result_file_path,strain_out_time_file_path,config_data)
else:
    print("Result file doesn't exixt!")

