# rosbag2PCD

This project provides tools to convert ROS bag files containing LIDAR and odometry data into point cloud data (PCD) for visualization and further processing. It is designed for robotics and mapping applications where you need to extract, synchronize, and visualize LIDAR scans with robot odometry.

## Project Overview
- **Input:** ROS bag file (e.g., `rf2o_data.bag`) containing `/scan` (LIDAR) and `/odom_rf2o` (odometry) topics.
- **Output:** Point cloud data (PCD) generated from synchronized LIDAR and odometry information, visualized using Open3D.

## File Descriptions

- **script_base.py**
  - Loads the ROS bag file and extracts LIDAR scan data.
  - Preprocesses the scan data to compute 2D point arrays (x, y) for each scan.
  - Use this script if you only want to extract and preprocess LIDAR scan data without odometry synchronization or 3D visualization.

- **script_optim.py**
  - Loads the ROS bag file and extracts both LIDAR and odometry data.
  - Synchronizes each LIDAR scan with the nearest odometry reading.
  - Computes global 3D point positions for each scan using robot pose.
  - Visualizes the resulting point cloud using Open3D.
  - Use this script for full LIDAR-odometry fusion and point cloud visualization.

## Usage

1. Place your ROS bag file (e.g., `rf2o_data.bag`) in the project directory.
2. Install dependencies:
   ```powershell
   pip install -r requirements.txt
   ```
3. Run the desired script:
   - For basic LIDAR point extraction:
     ```powershell
     python script_base.py
     ```
   - For full LIDAR-odometry fusion and visualization:
     ```powershell
     python script_optim.py
     ```

## Data
- Place your bag files or extracted CSVs in the `data/` directory (not tracked by git).
- Output point clouds are visualized directly; you can modify the scripts to save them if needed.

## Requirements
- Python 3.8+
- See `requirements.txt` for all dependencies (bagpy, pandas, numpy, open3d, etc.)

---
