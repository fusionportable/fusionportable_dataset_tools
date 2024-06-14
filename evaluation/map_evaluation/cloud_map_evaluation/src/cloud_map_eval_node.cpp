#include "cloud_map_eval.h"
#include <filesystem>

/* author: xchu
 * time: 2021.01.21
 * email: xhubd@connnect.ust.hk
 * 
 * This is a simple example to show how to use the cloud map evaluation class.
 * The evaluation class is used to evaluate the accuracy of the estimated map.
 * The evaluation metrics include: RMSE, MME, etc.
 */



int main(int argc, char **argv) {
    bool save_immediate_result = true;
    std::cout << "Current working directory: " << std::filesystem::current_path() << std::endl;

    // evaluation method, 0: point-to-point icp 1: point-to-plane icp 2: GICP
    int method = 2;
    // max correspondence pairs distance for icp knn search correspondence
    // iteration (the search radius for kdtree)
    // double icp_max_distance = 1.0;  for large scale outdoor map
    double icp_max_distance = 0.5;

    // set evaluation accuracy level, do not recommend to change, eg. 20cm/10cm/5cm/2cm/1cm
    Vector5d accuracy_level = Vector5d::Zero();
    // for outdoor map evaluation, we can set accuracy_level as  0.5, 0.3, 0.1, 0.05, 0.03;
    accuracy_level << 0.2, 0.1, 0.05, 0.02, 0.01;

    // for map produced by LIO, we need a initial pose
    // we can use cloud compare to align your maps to get the initial pose
    Eigen::Matrix4d initial_matrix = Eigen::Matrix4d::Identity();

    // the path dir must end with '/'
    std::string est_path, gt_path, results_path, sequence_name;
    // set your map folder
    std::string est_folder = "/home/xchu/my_git/Cloud_Map_Evaluation/cloud_map_evaluation/dataset/";
    sequence_name = "MCR_slow";
    // the estimated map file path
    est_path = est_folder + sequence_name + "/";
    // the ground truth map file path
    gt_path = est_folder + sequence_name + "/" + sequence_name + "_gt.pcd";
    // the result will be saved in this folder
    results_path = est_folder + sequence_name + "/";

    // if you evaluate mme (cost too much time, do not recommend to use it)
    bool evaluate_mme = false;
    // but you do not have ground truth map, you can set evaluate_mme = true
    bool evaluate_gt_mme = false;
    Param my_param(est_path, gt_path, results_path, initial_matrix, sequence_name,
                   method, icp_max_distance, accuracy_level,
                   save_immediate_result, evaluate_mme, evaluate_gt_mme);
    CloudMapEva my_evaluation(my_param);
    my_evaluation.process();

    return EXIT_SUCCESS;
}
