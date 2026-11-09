# LiDAR Obstacle Detection Using MATLAB

A MATLAB project that processes real-world LiDAR point-cloud data from the **KITTI Vision Benchmark Suite** to visualize 3D environments and identify potential obstacle regions.

## Features

- Loads and processes real KITTI Velodyne LiDAR data (`.bin` files).
- Visualizes 3D point clouds using MATLAB.
- Filters LiDAR measurements and removes approximate ground points.
- Generates a 2D occupancy grid.
- Uses **Breadth-First Search (BFS)** to identify connected regions of LiDAR detections.
- Visualizes detected clusters with unique colors and numbered centers.

## Technologies

- MATLAB
- KITTI Vision Benchmark Suite
- Breadth-First Search (BFS)
- 3D Point-Cloud Processing
- Occupancy-Grid Mapping

## How to Run

1. Download a LiDAR recording from the [KITTI Raw Dataset](https://www.cvlibs.net/datasets/kitti/raw_data.php).
2. Extract a `.bin` file from the `velodyne_points/data` directory.
3. Place the `.bin` file in the same folder as `lidar_obstacle_detection.m`.
4. Update the `fileName` variable in the MATLAB script to match your file.
5. Run the script in MATLAB or MATLAB Online.

## Output

The program generates three visualizations:

1. Original 3D LiDAR point cloud.
   <img width="956" height="788" alt="image" src="https://github.com/user-attachments/assets/9e06dec7-e353-45d0-ac16-2712341e5e76" />
2. Color-coded spatial clusters representing connected regions based on BFS clustering algorithm.
   <img width="933" height="775" alt="image" src="https://github.com/user-attachments/assets/0d6cf83f-25e2-4b2a-a07b-ee3b626afd1b" />
3. 2D occupancy grid showing detected/not detected LiDAR measurements.
   <img width="877" height="750" alt="image" src="https://github.com/user-attachments/assets/166a2c42-c0a3-4e89-a184-0bda53cfd0b4" />


**Note:** Clusters represent connected regions of LiDAR detections and do not necessarily correspond to individual physical objects.

## Data Source

[KITTI Vision Benchmark Suite](https://www.cvlibs.net/datasets/kitti/)




