/*
 This example demonstrates how to use OpenCorr to realize 3D/stereo DIC.
 In the implementation, the stereo matching between the two views uses
 epipolar constraint aided method and the ICGN algorithm with the 2nd order
 shape function, while the matching between the same view before and after
 deformation uses the SIFT feature guided method and the ICGN algorithm with
 the 1st order shape function.
*/

#include <fstream>

#include "opencorr.h"

using namespace opencorr;
using namespace std;
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

namespace py = pybind11;


void correlation_1(
	const std::function<void(int, const std::string&)>& update_progress,
	const std::vector<std::string>& cam0_img_path_list,
	const std::vector<std::string>& cam1_img_path_list,
	const std::string& poi_file_path,
	py::array_t<double, py::array::c_style> np_array,
	const std::vector<double>& real_list,
	const std::vector<int>& int_list,
	double real_scalar,
	int int_scalar)
{
	int progress = 0;
	std::string message = "Pozvana funkcija correlation_1";
	update_progress(progress, message);

	// Pristupite podacima polja
	auto ptr = static_cast<double*>(np_array.request().ptr);

	// Ispis svakog èlana polja
	for (size_t i = 0; i < np_array.shape(0); ++i) {
		for (size_t j = 0; j < 3; ++j) {
			std::cout << "Element (" << i << ", " << j << "): " << ptr[i * 3 + j] << std::endl;
		}
	}
}
void dic_3d_epipolar_sift(
	const std::function<void(const std::string&)>& update_progress,
	const py::dict& cal_data,
	const std::string& cam0_ref_file_path,
	const std::string& cam1_ref_file_path,
	const std::string& cam0_tar_file_path,
	const std::string& cam1_tar_file_path,
	const std::string& poi_file_path,
	const std::string& dic_result_file_path,
	const std::string& dic_out_time_file_path,
	const py::dict& config_data)
{
	//create the instances of camera parameters
	CameraIntrinsics view1_cam_intrinsics, view2_cam_intrinsics;
	CameraExtrinsics view1_cam_extrinsics, view2_cam_extrinsics;
	update_progress("dic_3d_epipolar_sift function started");
	view1_cam_intrinsics.fx = cal_data["Cam0_Fx [pixels]"].cast<float>();
	view1_cam_intrinsics.fy = cal_data["Cam0_Fy [pixels]"].cast<float>();
	view1_cam_intrinsics.fs = cal_data["Cam0_Fs [pixels]"].cast<float>();
	view1_cam_intrinsics.cx = cal_data["Cam0_Cx [pixels]"].cast<float>();
	view1_cam_intrinsics.cy = cal_data["Cam0_Cy [pixels]"].cast<float>();
	view1_cam_intrinsics.k1 = cal_data["Cam0_Kappa 1"].cast<float>();
	view1_cam_intrinsics.k2 = cal_data["Cam0_Kappa 2"].cast<float>();
	view1_cam_intrinsics.k3 = cal_data["Cam0_Kappa 3"].cast<float>();
	view1_cam_intrinsics.k4 = 0;
	view1_cam_intrinsics.k5 = 0;
	view1_cam_intrinsics.k6 = 0;
	view1_cam_intrinsics.p1 = cal_data["Cam0_P1"].cast<float>();
	view1_cam_intrinsics.p2 = cal_data["Cam0_P2"].cast<float>();

	view1_cam_extrinsics.tx = 0;
	view1_cam_extrinsics.ty = 0;
	view1_cam_extrinsics.tz = 0;
	view1_cam_extrinsics.rx = 0;
	view1_cam_extrinsics.ry = 0;
	view1_cam_extrinsics.rz = 0;

	view2_cam_intrinsics.fx = cal_data["Cam1_Fx [pixels]"].cast<float>();
	view2_cam_intrinsics.fy = cal_data["Cam1_Fy [pixels]"].cast<float>();
	view2_cam_intrinsics.fs = cal_data["Cam1_Fs [pixels]"].cast<float>();
	view2_cam_intrinsics.cx = cal_data["Cam1_Cx [pixels]"].cast<float>();
	view2_cam_intrinsics.cy = cal_data["Cam1_Cy [pixels]"].cast<float>();
	view2_cam_intrinsics.k1 = cal_data["Cam1_Kappa 1"].cast<float>();
	view2_cam_intrinsics.k2 = cal_data["Cam1_Kappa 2"].cast<float>();
	view2_cam_intrinsics.k3 = cal_data["Cam1_Kappa 3"].cast<float>();
	view2_cam_intrinsics.k4 = 0;
	view2_cam_intrinsics.k5 = 0;
	view2_cam_intrinsics.k6 = 0;
	view2_cam_intrinsics.p1 = cal_data["Cam1_P1"].cast<float>();
	view2_cam_intrinsics.p2 = cal_data["Cam1_P2"].cast<float>();

	view2_cam_extrinsics.tx = cal_data["Tx"].cast<float>();
	view2_cam_extrinsics.ty = cal_data["Ty"].cast<float>();
	view2_cam_extrinsics.tz = cal_data["Tz"].cast<float>();
	view2_cam_extrinsics.rx = cal_data["Theta [rad]"].cast<float>();
	view2_cam_extrinsics.ry = cal_data["Phi [rad]"].cast<float>();
	view2_cam_extrinsics.rz = cal_data["Psi [rad]"].cast<float>();


	//set paths of images.
	string ref_view1_image_path = cam0_ref_file_path;
	string ref_view2_image_path = cam1_ref_file_path; 
	string tar_view1_image_path = cam0_tar_file_path; 
	string tar_view2_image_path = cam1_tar_file_path; 

	//create the instances of images
	Image2D ref_view1_img(ref_view1_image_path);
	Image2D ref_view2_img(ref_view2_image_path);
	Image2D tar_view1_img(tar_view1_image_path);
	Image2D tar_view2_img(tar_view2_image_path);

	//create instances to read and write csv files
	string delimiter = ",";
	ofstream csv_out; //instance for output calculation time
	IO2D in_out; //instance for input and output DIC data
	in_out.setDelimiter(delimiter);
	in_out.setHeight(ref_view1_img.height);
	in_out.setWidth(ref_view1_img.width);

	//load the coordinates of POIs in principal view
	string file_path = poi_file_path; 
	vector<Point2D> ref_view1_pt_queue = in_out.loadPoint2D(file_path);
	int queue_length = (int)ref_view1_pt_queue.size();


	//initialize papameters for timing
	double timer_tic, timer_toc, consumed_time;
	vector<double> computation_time;

	//get the time of start
	timer_tic = omp_get_wtime();

	//set OpenMP parameters
	int cpu_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(cpu_thread_number);



	//create the instances for stereovision
	Calibration cam_view1_calib(view1_cam_intrinsics, view1_cam_extrinsics);
	Calibration cam_view2_calib(view2_cam_intrinsics, view2_cam_extrinsics);
	cam_view1_calib.prepare(ref_view1_img.height, ref_view1_img.width);
	cam_view2_calib.prepare(ref_view2_img.height, ref_view2_img.width);
	Stereovision stereo_reconstruction(&cam_view1_calib, &cam_view2_calib, cpu_thread_number);

	//create the the queues of 2D points for stereo matching and reconstruction
	Point2D point_2d;
	vector<Point2D> ref_view2_pt_queue(queue_length, point_2d);
	vector<Point2D> tar_view1_pt_queue(queue_length, point_2d);
	vector<Point2D> tar_view2_pt_queue(queue_length, point_2d);

	//create the the queues of 3D points for stereo reconstruction
	Point3D point_3d;
	vector<Point3D> ref_pt_3d_queue(queue_length, point_3d);
	vector<Point3D> tar_pt_3d_queue(queue_length, point_3d);

	//create the queues of POIs for matching
	POI2D poi_2d(point_2d);
	vector<POI2D> poi_queue(queue_length, poi_2d);
	vector<POI2D> poi_round_queue(queue_length, poi_2d);
	vector<POI2DS> poi_result_queue(queue_length, poi_2d);

	//assign the coordinates to POI queues
#pragma omp parallel for
	for (int i = 0; i < (int)queue_length; i++)
	{
		poi_queue[i].x = ref_view1_pt_queue[i].x;
		poi_queue[i].y = ref_view1_pt_queue[i].y;

		poi_result_queue[i].x = ref_view1_pt_queue[i].x;
		poi_result_queue[i].y = ref_view1_pt_queue[i].y;
	}

	//set DIC parameters
	string key;
	int subset_radius_x = 16;
	int subset_radius_y = 16;
	float conv_criterion = 0.001f;
	int stop_condition = 10;

	key = "ss_radius_x";
	if (config_data.contains(key)) {
		subset_radius_x = config_data[key.c_str()].cast<int>();
	}
	key = "ss_radius_y";
	if (config_data.contains(key)) {
		subset_radius_y = config_data[key.c_str()].cast<int>();
	}

	key = "icgn_conv_crit";
	if (config_data.contains(key)) {
		conv_criterion = config_data[key.c_str()].cast<float>();
	}

	key = "icgn_stop_cond";
	if (config_data.contains(key)) {
		stop_condition = config_data[key.c_str()].cast<float>();
	}




	//initialize ICGN with the 1st order shape function
	ICGN2D1* icgn1 = new ICGN2D1(subset_radius_x, subset_radius_y, conv_criterion, stop_condition, cpu_thread_number);
	//initialize ICGN with the 2nd order shape function
	ICGN2D2* icgn2 = new ICGN2D2(subset_radius_x, subset_radius_y, conv_criterion, stop_condition, cpu_thread_number);

	//create the instances for SIFT feature aided matching
	//set SIFT defaults
	Sift2dConfig sift_config = Sift2dConfig();
	sift_config.n_features = 0;
	sift_config.n_octave_layers = 3;
	sift_config.contrast_threshold = 0.04f; //default configuration in OpenCV
	sift_config.edge_threshold = 10.f;
	sift_config.sigma = 1.6f;
	float sift_matching_ratio = 0.8f;
	key = "sift_n_features";
	if (config_data.contains(key)) {
		sift_config.n_features = config_data[key.c_str()].cast<int>();
	}
	key = "sift_n_octave_layers";
	if (config_data.contains(key)) {
		sift_config.n_octave_layers = config_data[key.c_str()].cast<int>();
	}
	key = "sift_contrast_threshold";
	if (config_data.contains(key)) {
		sift_config.contrast_threshold = config_data[key.c_str()].cast<float>();
	}
	key = "sift_edge_threshold";
	if (config_data.contains(key)) {
		sift_config.edge_threshold = config_data[key.c_str()].cast<float>();
	}
	key = "sift_sigma";
	if (config_data.contains(key)) {
		sift_config.sigma = config_data[key.c_str()].cast<float>();
	}
	key = "sift_matching_ratio";
	if (config_data.contains(key)) {
		sift_matching_ratio = config_data[key.c_str()].cast<float>();
	}
	SIFT2D* sift = new SIFT2D();
	sift->setSiftConfig(sift_config);
	sift->setMatching(sift_matching_ratio);

	FeatureAffine2D* feature_affine = new FeatureAffine2D(subset_radius_x, subset_radius_y, cpu_thread_number);
	RansacConfig ransac_config = RansacConfig();
	
	ransac_config.error_threshold = 1.5f;
	ransac_config.sample_mumber = 3;
	ransac_config.trial_number = 20;
	int faeture_affine_min_neighbor_num = 7;
	float faeture_affine_neighbor_search_radius = feature_affine->getSearchRadius();
	key = "faeture_affine_ransac_error_threshold";
	if (config_data.contains(key)) {
		ransac_config.error_threshold = config_data[key.c_str()].cast<float>();
	}
	key = "faeture_affine_ransac_sample_mumber";
	if (config_data.contains(key)) {
		ransac_config.sample_mumber = config_data[key.c_str()].cast<int>();
	}
	key = "faeture_affine_ransac_trial_number";
	if (config_data.contains(key)) {
		ransac_config.trial_number = config_data[key.c_str()].cast<int>();
	}
	key = "faeture_affine_min_neighbor_num";
	if (config_data.contains(key)) {
		faeture_affine_min_neighbor_num = config_data[key.c_str()].cast<int>();
	}
	key = "faeture_affine_neighbor_search_radius";
	if (config_data.contains(key)) {
		faeture_affine_neighbor_search_radius = config_data[key.c_str()].cast<float>();
	}
	feature_affine->setRansacConfig(ransac_config);
	feature_affine->setSearchParameters(faeture_affine_neighbor_search_radius, faeture_affine_min_neighbor_num);

	//create an instance for epipolar constraint aided matching
	EpipolarSearch* epipolar_search = new EpipolarSearch(cam_view1_calib, cam_view2_calib, cpu_thread_number);

	//set search parameters in epipolar constraint aided matching
	Point2D parallax_guess(-30, -40);
	key = "epipolar_paralax_width";
	if (config_data.contains(key)) {
		parallax_guess.x = config_data[key.c_str()].cast<float>();
	}
	key = "epipolar_paralax_height";
	if (config_data.contains(key)) {
		parallax_guess.y = config_data[key.c_str()].cast<float>();
	}

	epipolar_search->setParallax(parallax_guess);
	int search_radius = 30;
	int search_step = 5;
	key = "epipolar_search_radius";
	if (config_data.contains(key)) {
		search_radius = config_data[key.c_str()].cast<int>();
	}
	key = "epipolar_search_step";
	if (config_data.contains(key)) {
		search_step = config_data[key.c_str()].cast<int>();
	}

	epipolar_search->setSearch(search_radius, search_step);

	//initialize an ICGN2D1 instance in epipolar constraint aided matching
	subset_radius_x = 20;
	subset_radius_y = 20;
	conv_criterion = 0.05f;
	stop_condition = 5;

	key = "epipolar_ss_radius_x";
	if (config_data.contains(key)) {
		subset_radius_x = config_data[key.c_str()].cast<int>();
	}
	key = "epipolar_ss_radius_y";
	if (config_data.contains(key)) {
		subset_radius_y = config_data[key.c_str()].cast<int>();
	}

	key = "epipolar_icgn_conv_crit";
	if (config_data.contains(key)) {
		conv_criterion = config_data[key.c_str()].cast<float>();
	}

	key = "epipolar_icgn_stop_cond";
	if (config_data.contains(key)) {
		stop_condition = config_data[key.c_str()].cast<float>();
	}
	epipolar_search->createICGN(subset_radius_x, subset_radius_y, conv_criterion, stop_condition);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	update_progress("Initialization with " + to_string(queue_length) + " POIs takes " + to_string(consumed_time) + " sec, " + to_string(cpu_thread_number) + " CPU threads launched.");
	//get the time of start
	timer_tic = omp_get_wtime();

	//stereo matching between reference view1 and reference view2
	epipolar_search->setImages(ref_view1_img, ref_view2_img);
	epipolar_search->prepare();

	//coarse registration
	epipolar_search->compute(poi_queue);

	//refined registration
	icgn2->setImages(ref_view1_img, ref_view2_img);
	icgn2->prepare();
	icgn2->compute(poi_queue);

	//store the results of stereo matching between the two reference views
#pragma omp parallel for
	for (int i = 0; i < queue_length; i++)
	{
		Point2D current_location(poi_queue[i].x, poi_queue[i].y);
		Point2D current_offset(poi_queue[i].deformation.u, poi_queue[i].deformation.v);
		ref_view2_pt_queue[i] = current_location + current_offset;
		poi_result_queue[i].result.r2_x = ref_view2_pt_queue[i].x;
		poi_result_queue[i].result.r2_y = ref_view2_pt_queue[i].y;
		poi_result_queue[i].result.r1r2_zncc = poi_queue[i].result.zncc;
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //1

	//display the time of processing on the screen
	update_progress("Stereo matching between reference view1 and view2 takes " + to_string(consumed_time) +" sec.");

	//get the time of start
	timer_tic = omp_get_wtime();

	//temporal matching between reference view1 and target view 1
	//extract and match the SIFT features
	sift->setImages(ref_view1_img, tar_view1_img);
	sift->prepare();
	sift->compute();

	//estimate the deformation according to the SIFT features around each POIs
	feature_affine->setImages(ref_view1_img, tar_view1_img);
	feature_affine->setKeypointPair(sift->ref_matched_kp, sift->tar_matched_kp);
	feature_affine->prepare();
	feature_affine->compute(poi_queue);

	//high accuracy registration
	icgn1->setImages(ref_view1_img, tar_view1_img);
	icgn1->prepare();
	icgn1->compute(poi_queue);

	//store the results of temporal matching
#pragma omp parallel for
	for (int i = 0; i < queue_length; i++)
	{
		Point2D current_location(poi_queue[i].x, poi_queue[i].y);
		Point2D current_offset(poi_queue[i].deformation.u, poi_queue[i].deformation.v);
		tar_view1_pt_queue[i] = current_location + current_offset;
		poi_result_queue[i].result.t1_x = tar_view1_pt_queue[i].x;
		poi_result_queue[i].result.t1_y = tar_view1_pt_queue[i].y;
		poi_result_queue[i].result.r1t1_zncc = poi_queue[i].result.zncc;

		poi_round_queue[i].x = round(tar_view1_pt_queue[i].x);
		poi_round_queue[i].y = round(tar_view1_pt_queue[i].y);
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //2

	//display the time of processing on the screen
	update_progress("Temporal matching between reference view1 and target view1 takes " + to_string(consumed_time) + " sec.");

	//get the time of start
	timer_tic = omp_get_wtime();

	//stereo matching between reference view1 and target view2
	//coase stereo matching between target view1 and taget view2
	parallax_guess.x = -30.f;
	parallax_guess.y = -40.f;
	key = "epipolar_paralax_width";
	if (config_data.contains(key)) {
		parallax_guess.x = config_data[key.c_str()].cast<float>();
	}
	key = "epipolar_paralax_height";
	if (config_data.contains(key)) {
		parallax_guess.y = config_data[key.c_str()].cast<float>();
	}
	epipolar_search->setParallax(parallax_guess);
	epipolar_search->setImages(tar_view1_img, tar_view2_img);
	epipolar_search->prepare();
	epipolar_search->compute(poi_round_queue);

	//combine the dispalcements obtained by stereo matching and temperoal matching
#pragma omp parallel for
	for (int i = 0; i < queue_length; i++)
	{
		poi_queue[i].deformation.u += poi_round_queue[i].deformation.u;
		poi_queue[i].deformation.v += poi_round_queue[i].deformation.v;
	}

	//stereo matching between reference view1 and target view2
	icgn2->setImages(ref_view1_img, tar_view2_img);
	icgn2->prepare();
	icgn2->compute(poi_queue);

	//store the results of matching
#pragma omp parallel for
	for (int i = 0; i < queue_length; i++)
	{
		Point2D current_location(poi_queue[i].x, poi_queue[i].y);
		Point2D current_offset(poi_queue[i].deformation.u, poi_queue[i].deformation.v);
		tar_view2_pt_queue[i] = current_location + current_offset;
		poi_result_queue[i].result.t2_x = tar_view2_pt_queue[i].x;
		poi_result_queue[i].result.t2_y = tar_view2_pt_queue[i].y;
		poi_result_queue[i].result.r1t2_zncc = poi_queue[i].result.zncc;
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //3

	//display the time of processing on the screen
	update_progress("Stereo matching between reference view1 and target view2 takes " + to_string(consumed_time) + " sec.");

	//get the time of start
	timer_tic = omp_get_wtime();

	//reconstruct the 3D coordinates in world coordinate system
	stereo_reconstruction.prepare();
	stereo_reconstruction.reconstruct(ref_view1_pt_queue, ref_view2_pt_queue, ref_pt_3d_queue);
	stereo_reconstruction.reconstruct(tar_view1_pt_queue, tar_view2_pt_queue, tar_pt_3d_queue);

	//calculate the 3D displacements of POIs
#pragma omp parallel for
	for (int i = 0; i < queue_length; i++)
	{
		poi_result_queue[i].ref_coor = ref_pt_3d_queue[i];
		poi_result_queue[i].tar_coor = tar_pt_3d_queue[i];
		poi_result_queue[i].deformation.u = tar_pt_3d_queue[i].x - ref_pt_3d_queue[i].x;
		poi_result_queue[i].deformation.v = tar_pt_3d_queue[i].y - ref_pt_3d_queue[i].y;
		poi_result_queue[i].deformation.w = tar_pt_3d_queue[i].z - ref_pt_3d_queue[i].z;
	}

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //4

	//display the time of processing on the screen
	update_progress("Stereo reconstruction and calculation of displacements takes " + to_string(consumed_time) + " sec.");

	//save the results
	file_path = dic_result_file_path;
	in_out.setPath(file_path);
	in_out.saveTable2DS(poi_result_queue);

	//save the computation time
	file_path = dic_out_time_file_path;
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "POI number" << delimiter << "Initialization" << delimiter << "r1_to_r2" << delimiter << "r1_to_t1" << delimiter << "r1_to_t2" << delimiter << "reconstruction" << endl;
		csv_out << poi_queue.size() << delimiter << computation_time[0] << delimiter << computation_time[1] << delimiter << computation_time[2] << delimiter << computation_time[3] << delimiter << computation_time[4] << endl;
	}
	csv_out.close();

	delete icgn1;
	delete icgn2;
	delete sift;
	delete feature_affine;
	delete epipolar_search;
	update_progress("dic_3d_epipolar_sift function finished!");
}
void dic_3d_strain(
	const std::function<void(const std::string&)>& update_progress,
	const std::string& tar_file_path,
	const std::string& dic_result_file_path,
	const std::string& strain_out_time_file_path,
	const py::dict& config_data
	)
{
	update_progress("dic_3d_strain function started!");
	//select the image file to get the file name to process
	string tar_image_path = tar_file_path;
	Image2D tar_img(tar_image_path);

	//initialize papameters for timing
	double timer_tic, timer_toc, consumed_time;
	vector<double> computation_time;

	//get the time of start
	timer_tic = omp_get_wtime();

	//get the DIC results from csv file
	string file_path = dic_result_file_path;
	string delimiter = ",";
	ofstream csv_out; //instance for output calculation time
	IO2D in_out; //instance for input and output DIC data
	in_out.setPath(file_path);
	in_out.setHeight(tar_img.height);
	in_out.setWidth(tar_img.width);
	in_out.setDelimiter(delimiter);

	//load a queue of POIs
	vector<POI2DS> poi_queue = in_out.loadTable2DS();

	//set OpenMP parameters
	int cpu_thread_number = omp_get_num_procs() - 1;
	omp_set_num_threads(cpu_thread_number);

	//set the radius of subregion for polynomial fit of displacement field
	float strain_radius = 20.f;
	string key;
	key = "strain_radius";
	if (config_data.contains(key)) {
		strain_radius = config_data[key.c_str()].cast<float>();
	}

	//set the miminum number of neighbor POIs to perform fitting
	int min_neighbors = 5;
	key = "min_neighbors";
	if (config_data.contains(key)) {
		min_neighbors = config_data[key.c_str()].cast<int>();
	}

	//initialize a object of stain calculation
	Strain* strain = new Strain(strain_radius, min_neighbors, cpu_thread_number);

	//get the time of end 
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //0

	//display the time of initialization on screen
	update_progress("Initialization takes " + to_string(consumed_time) + " sec, " + to_string(cpu_thread_number) + " CPU threads launched.");

	//get the time of start
	timer_tic = omp_get_wtime();

	//calculate the strain exx, eyy, ezz, exy, eyz, ezx
	strain->prepare(poi_queue);
	strain->compute(poi_queue);

	//get time of end
	timer_toc = omp_get_wtime();
	consumed_time = timer_toc - timer_tic;
	computation_time.push_back(consumed_time); //1

	update_progress("Strain calculation of " + to_string(poi_queue.size()) + " POIs takes " + to_string(consumed_time) + " sec.");

	//update the table of DIC results with calculated strains
	in_out.saveTable2DS(poi_queue);

	//save the computation time
	file_path = strain_out_time_file_path;
	csv_out.open(file_path);
	if (csv_out.is_open())
	{
		csv_out << "POI number" << delimiter << "Initialization" << delimiter << "Strain calculation" << endl;
		csv_out << poi_queue.size() << delimiter << computation_time[0] << delimiter << computation_time[1] << endl;
	}
	csv_out.close();

	//destroy the instance
	delete strain;
	update_progress("dic_3d_strain function finished!");


}

PYBIND11_MODULE(opencorrpy, m) {
	m.def("dic_3d_epipolar_sift", &dic_3d_epipolar_sift);
	m.def("dic_3d_strain", &dic_3d_strain);
}