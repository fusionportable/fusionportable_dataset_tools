#!/usr/bin/env python
import rosbag
import os
from tqdm import tqdm

def extract_and_write_imu_data(source_dir, target_dir, topic_to_filename):
    """
    Extracts IMU data from rosbags located in the specified source directory and writes the data into text files in the target directory,
    with a tqdm progress bar for each rosbag file.

    Parameters:
    source_dir (str): The path to the directory containing the subfolders with rosbag files.
    target_dir (str): The path to the directory where the text files will be saved.
    topic_to_filename (dict): A dictionary mapping IMU topic names to specific strings for file naming.
    """
    # Ensure the target directory exists
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
    
    # Walk through the source directory
    for root, dirs, files in os.walk(source_dir):
        for subfolder in dirs:
            subfolder_path = os.path.join(root, subfolder)
            bag_filename = subfolder + '.bag'
            bag_path = os.path.join(subfolder_path, bag_filename)
            
            # Check if the rosbag file exists in the subfolder
            if os.path.isfile(bag_path):
                print(f"Processing {bag_path}")
                with rosbag.Bag(bag_path, 'r') as bag:
                    for topic, filename_suffix in topic_to_filename.items():
                        output_file_path = os.path.join(target_dir, f"{subfolder}_{filename_suffix}.txt")
                        # Initialize file outside the loop to avoid overwriting
                        file = open(output_file_path, 'w')
                        try:
                            for _, msg, _ in tqdm(bag.read_messages(topics=[topic]), total=bag.get_message_count(topic_filters=[topic]), desc=f"Processing {topic} in {subfolder}.bag"):
                                # Extract timestamp, angular velocity, and linear acceleration from the message header
                                time_stamp = msg.header.stamp.to_sec()
                                angular_velocity = msg.angular_velocity
                                linear_acceleration = msg.linear_acceleration

                                # Write data to the file
                                file.write(f"{time_stamp} {angular_velocity.x} {angular_velocity.y} {angular_velocity.z} {linear_acceleration.x} {linear_acceleration.y} {linear_acceleration.z}\n")
                        finally:
                            file.close()

# Define the source directory containing subfolders with rosbag files
source_directory = './'

# Define the target directory where text files will be saved
target_directory = './imus_data'

# Define a mapping from topic names to specific strings for file naming
topic_to_filename = {
    '/os_cloud_node/imu/data_raw': 'ouster',
    '/stim300/imu/data_raw': 'stim300',
    '/unitree/imu/data_raw': 'unitree',
}

# Call the function to extract IMU data and write it to text files
extract_and_write_imu_data(source_directory, target_directory, topic_to_filename)

