# FusionPortable_dataset_tools

## News
* (20240713) Provide usage example of running SLAM and calibration algorithms with our dataset.
* (20240508) Groundtruth poses of all vehicle-related sequences are postprocessed: eliminate poses characterized by high uncertainty.
* (20240422) Data can be downloaded from <a href="https://pan.baidu.com/s/1lZwK-TNrCyoyC9oWEs8jUg?pwd=byj8">Baidu Wang Pan</a> with the code <b>byj8</b>.
* (20240414) All sequences, ground-truth trajectories, and ground-truth maps have been publicly released. If you find issues of GT trajectories and maps, please contact us or report <a href="https://github.com/fusionportable/fusionportable_dataset_tools/issues">here</a>.
* (20240413) A small simulated navigation environment is provied.
* (20240408) The development tool has been initially released.
* (20240407) Data can be downloaded from <a href="https://drive.google.com/drive/folders/1PYhnf3PlY5r0hbyzWDGTUTPxRMl6SYa-?usp=sharing">Google Drive</a>. 

## Download Dataset
1. Please visit [FusionPortable dataset](https://fusionportable.github.io/dataset/fusionportable) and [FusionPortableV2_dataset](https://fusionportable.github.io/dataset/fusionportable_v2) to check and download data.
2. Download the compressed rosbag.
3. When finished, use the similar command ```7z l 20220216_garden_day.7z``` to extract data.

<!-- ### Notice 20230928
Please add this line in ```/etc/hosts```: ```143.89.6.5 www.ram-lab.com filebrowser.ram-lab.com``` to visit the dataset page. -->

## Installation
Clone the Repo only
```
git clone https://github.com/fusionportable/fusionportable_dataset_tools.git
```
Clone the Repo with submodules (including calibration_files and algorithms for experiments)
```
git clone https://github.com/fusionportable/fusionportable_dataset_tools.git --recursive
```
Setup the Python environment (tested on Python-3.9) or using Anaconda directly and run
```
cd fusionportable_dataset_tools
conda create -n fp_dataset python=3.9
pip install -r requirements
```

## Usage
#### Usage of Data-Loader-Related Functions
1. Convert the raw rosbag into individual files: ```write_bag_to_data.ipynb```

2. Convert the algorithms' results (i.e., R3LIVE, FAST-LIO2) stored as the rosbag into individual files: ```write_alg_bag_to_data.ipynb```

3. Convert the raw files into the KITTI-360 format (including synchronized sensor data, callibration files, odometry): ```write_data_to_kitti360.ipynb```

4. Generate depth map with respect to the frame_left camera for the depth evaluation purpose: ```write_depthmap_to_kitti360.ipynb```

#### Usage of Tool Functions
1. Project undistorted point cloud onto image to verify the error in extrinsics: ```visualize_depthmap.ipynb```

#### Usage of Evaluation Tools
1. Trajectory Evaluation

2. Mapping Evaluation: click this [link](evaluation/map_evaluation) to try. 

#### Applications
We have provided configuration files of running experiments with our dataset

1. Visual SLAM: [DROID-SLAM](application/DROID-SLAM)

#### Calibration Tools
*Intrinsic Calibration*
1. Wheel enoder calibration: [encoer_calc](calibration/encoder_calc/) 

*Extrinsic Calibration*
1. Camera-LiDAR calibration: [LCECalib](calibration/LCECalib)
2. Camera-IMU, Multi-IMU calibration: [kalibr](calibration/kalibr)


## Some Issues with Dependencies
##### 1. Something wrong with the ```ros_numpy```
```python
File ~/anaconda3/envs/fp_dataset/lib/python3.9/site-packages/ros_numpy/point_cloud2.py:224
    221             new_cloud_arr[field_name] = cloud_arr[field_name]
...
AttributeError: module 'numpy' has no attribute 'float'.
```
**Solution:** Please goto ```path_to_python/site-packages/ros_numpy/point_cloud2.py:224``` and replace the original line with
```python
def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float64):
```

## Contribution
Please refer to [Contribution Guidance](doc/contribution.md) to make contributions to this project.

## Inquiry
Please feel free to contact Dr.Jianhao Jiao (jiaojh1994 at gmail.com) or Mr.Hexiang Wei (hweiak at connect.ust.hk) if you have any questions.

## Citation
```
@misc{wei2024fusionportablev2,
  title={FusionPortableV2: A Unified Multi-Sensor Dataset for Generalized SLAM Across Diverse Platforms and Scalable Environments}, 
  author={Hexiang Wei and Jianhao Jiao and Xiangcheng Hu and Jingwen Yu and Xupeng Xie and Jin Wu and Yilong Zhu and Yuxuan Liu and Lujia Wang and Ming Liu},
  year={2024},
  eprint={2404.08563},
  archivePrefix={arXiv},
  primaryClass={cs.RO}
}
```

```
@inproceedings{jiao2022fusionportable,
  title={Fusionportable: A multi-sensor campus-scene dataset for evaluation of localization and mapping accuracy on diverse platforms},
  author={Jiao, Jianhao and Wei, Hexiang and Hu, Tianshuai and Hu, Xiangcheng and Zhu, Yilong and He, Zhijian and Wu, Jin and Yu, Jingwen and Xie, Xupeng and Huang, Huaiyang and others},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={3851--3856},
  year={2022},
  organization={IEEE}
}
```
