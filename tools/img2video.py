#! /usr/bin/env python3
# Usage: python3 tools/img2video.py --input_folder img_data --output_file output.mp4 --fps 20

import os
import argparse
import cv2
import numpy as np

def images_to_video(image_folder, output_file, fps, skip):
    images = sorted([img for img in os.listdir(image_folder) if img.endswith(".png")])
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    width, height = frame.shape[1], frame.shape[0]

    fourcc = "mp4v"
    out = cv2.VideoWriter(output_file, cv2.VideoWriter_fourcc(*fourcc), fps, (width, height))

    for cnt, image in enumerate(images):
        if cnt % skip != 0:
            continue
        img_path = os.path.join(image_folder, image)
        frame = cv2.imread(img_path)
        out.write(frame) # Convert PIL image to numpy array
    out.release()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert a series of images into an mp4 video.")
    parser.add_argument("--input_folder", help="Path to the folder containing input images")
    parser.add_argument("--output_file", help="Path to the output mp4 video file")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second for the output video (default: 30)")
    parser.add_argument("--skip", type=int, default=1, help="Frames to be skipped")
    args = parser.parse_args()

    images_to_video(args.input_folder, args.output_file, args.fps, args.skip)