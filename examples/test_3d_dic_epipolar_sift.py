import os
import sys
import numpy as np
opencorrpy_rel_path = '../../bin'
module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), opencorrpy_rel_path))
sys.path.append(module_dir)
import opencorrpy as ocpy

def update_progress(msg):
    print(msg)

calibration_data = {
    'Cam0_Fx [pixels]': 6673.315918,
    'Cam0_Fy [pixels]': 6669.302734,
    'Cam0_Fs [pixels]': 0.0,
    'Cam0_Kappa 1': 0.032258954,
    'Cam0_Kappa 2': -1.01141417,
    'Cam0_Kappa 3': 29.78838921,
    'Cam0_P1': 0.0,
    'Cam0_P2': 0.0,
    'Cam0_Cx [pixels]': 872.1577759,
    'Cam0_Cy [pixels]': 579.9553223,
    'Cam1_Fx [pixels]': 6607.618164,
    'Cam1_Fy [pixels]': 6602.857422,
    'Cam1_Fs [pixels]': 0.0,
    'Cam1_Kappa 1': 0.064598486,
    'Cam1_Kappa 2': -4.531373978,
    'Cam1_Kappa 3': 29.78838921,
    'Cam1_P1': 0.0,
    'Cam1_P2': 0.0,
    'Cam1_Cx [pixels]': 917.9733887,
    'Cam1_Cy [pixels]': 531.6352539,
    'Tx [mm]': 122.2488624,
    'Ty [mm]': 1.848889206,
    'Tz [mm]': 17.62463774,
    'Theta [deg]': 0.130689541,
    'Phi [deg]': -19.06769369,
    'Psi [deg]': 0.281404039
}
# convert calibration data to desired units
calibration_data['Tx']=calibration_data['Tx [mm]']
calibration_data['Ty']=calibration_data['Ty [mm]']
calibration_data['Tz']=calibration_data['Tz [mm]']
# convert calibration rotations to radians
calibration_data['Theta [rad]']=np.deg2rad(calibration_data['Theta [deg]'])
calibration_data['Phi [rad]']=np.deg2rad(calibration_data['Phi [deg]'])
calibration_data['Psi [rad]']=np.deg2rad(calibration_data['Psi [deg]'])

folder_path = '3d_dic'
cam0_ref_image_path = os.path.abspath(os.path.join(folder_path,"GT4-0000_0.tif"))
cam1_ref_image_path = os.path.abspath(os.path.join(folder_path,"GT4-0000_1.tif"))
cam0_tar_image_path = os.path.abspath(os.path.join(folder_path,"GT4-0273_0.tif"))
cam1_tar_image_path = os.path.abspath(os.path.join(folder_path,"GT4-0273_1.tif"))
poi_file_path = os.path.abspath(os.path.join(folder_path,"GT4-POIs.csv"))

config_data = {}
config_data["ss_radius_x"] = 16
config_data["ss_radius_y"] = 16
# ICGN config data
config_data["icgn_conv_crit"] = 0.001
config_data["icgn_stop_cond"] = 10
#epipolar search config data
config_data["epipolar_paralax_width"] = -30
config_data["epipolar_paralax_height"] = -40
config_data["epipolar_search_radius"] = 30
config_data["epipolar_search_step"] = 5
config_data["epipolar_ss_radius_x"] = 20
config_data["epipolar_ss_radius_y"] = 20
config_data["epipolar_icgn_conv_crit"] = 0.05
config_data["epipolar_icgn_stop_cond"] = 5
#sift config data
config_data["sift_n_features"] = 0
config_data["sift_n_octave_layers"] = 3
config_data["sift_contrast_threshold"] = 0.04
config_data["sift_edge_threshold"] = 10.0
config_data["sift_sigma"] = 1.6
config_data["sift_matching_ratio"] = 0.8
#faeture affine
config_data["faeture_affine_ransac_error_threshold"] = 1.5
config_data["faeture_affine_ransac_sample_mumber"] = 3
config_data["faeture_affine_ransac_trial_number"] = 20
config_data["faeture_affine_min_neighbor_num"] = 7
config_data["faeture_affine_neighbor_search_radius"] = np.sqrt((config_data["ss_radius_x"]**2 + config_data["ss_radius_y"]**2))


ocpy.dic_3d_epipolar_sift(update_progress,calibration_data,
                         cam0_ref_image_path,cam1_ref_image_path,cam0_tar_image_path,cam1_tar_image_path,
                         poi_file_path,config_data)
