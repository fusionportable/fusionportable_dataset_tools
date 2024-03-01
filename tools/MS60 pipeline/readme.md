# Dataset Post-Processing Toolkit for SLAM Evaluation

This repository offers a comprehensive toolkit for processing SLAM datasets from raw measurements to evaluation-ready ground truth trajectories. It supports a sequence of operations required to transform surveying data, calibrate time offsets, and prepare data for accuracy assessment against SLAM outputs.

## Workflow Overview

The toolkit executes the following sequence of operations:

1. **XML to TUM Conversion (`xml2tum.py`)**: Converts raw surveying data from XML format into the TUM pose format, a common format for SLAM datasets.

2. **Body Frame Transformation (`ms2body.py`)**: Transforms the TUM-formatted data into the DROID's body coordinate frame, aligning the dataset with the robot's motion.

3. **Interpolation to 20Hz (`resample_patch.py`)**: Interpolates the transformed data to a higher temporal resolution of 20Hz, ensuring a dense and consistent set of pose estimates for comparison.

4. **Time Offset Calibration (`correlation.py`)**: Enumerates through different time offsets to calibrate the trajectory with the FAST-LIO2 SLAM system, optimizing the temporal alignment between the ground truth and SLAM-generated data.

5. **Offset Compensation (`add_offset.py`)**: Applies the calculated optimal time offset to the ground truth files, ensuring the ground truth and SLAM trajectories are temporally synchronized for accurate assessment.

6. **Trajectory Evaluation**: The final ground truth trajectory files, with offsets applied, are ready for evaluation against SLAM trajectory outputs, providing key metrics such as RMSE for performance analysis.

## Visualizations

- `handheld_starbucks00.png`: Illustrates the interpolation results for the `handheld_starbucks00` sequence, showcasing the data density before and after processing. Intervals exceeding one second in duration will not be subjected to interpolation.
  
  ![Interpolation Results](handheld_starbucks00.png)

- `correlation.png`: Displays the outcome of the `correlation.py` script, highlighting the correlation between different time offsets and the resulting trajectory accuracy metrics. As figure shown, optimal offset is -0.64s.

  ![Time Offset Calibration](correlation.png)

## Getting Started

To use this toolkit, clone the repository and follow the sequence of operations outlined above. Each script can be executed with the required arguments and options pertaining to your dataset:

