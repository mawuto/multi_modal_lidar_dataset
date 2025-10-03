# Multi-Modal LiDAR Dataset (Avia • Mid-360 • Ouster)

> **Status:** Pre-release (dataset links below).  
> **Paper:** *Understanding LiDAR Variability: A Dataset and Comparative Study of Solid-State and Spinning LiDARs* (under review).

This repository hosts **documentation, download links, and baseline code** for a multi-LiDAR dataset featuring:
- **Livox Avia** (solid-state, limited FoV)
- **Livox Mid-360** (dome-shaped solid-state)
- **Ouster OS0-128** (spinning)

---
### 1) Data Collection Platform
Robot and sensors setup:

<p align="center">
  <img src="docs/pipelines/img/data_collection_platform.png" alt="Platform" width="48%"/>
  <img src="docs/pipelines/img/data_collection_platform_calibration.png" alt="Calibration" width="48%"/>
</p>

---
## 2) Quick Links

**Dataset Download (Baidu Netdisk):**

| Dataset Name              | Description                  | Download Link                                                                 | Password | Size    |
|----------------------------|------------------------------|-------------------------------------------------------------------------------|----------|---------|
| IndoorOffice1_dataset.bag  | Indoor office environment 1  | [Link](https://pan.baidu.com/s/1sAiaXva7OY0z7ILK0nIv1w)                       | `ec2t`   | 4.47GB  |
| IndoorOffice2_dataset.bag  | Indoor office environment 2  | [Link](https://pan.baidu.com/s/1g64HCeztEmPRc-rdee_oew)                       | `5yxp`   | 6.46GB  |
| OutdoorRoad_dataset.bag    | Long structured road scene   | [Link](https://pan.baidu.com/s/1O7MBU-5u8taKWxgpxwJJwQ)                       | `uk1e`   | 44.48GB |
| OutdoorRoad_cut0.bag       | Road scene (first segment)   | [Link](https://pan.baidu.com/s/1D3BiAND8qwbKwoCiSFW0sA)                       | `eyu9`   | 4.47GB  |
| OutdoorRoad_cut1.bag       | Road scene (second segment)  | [Link](https://pan.baidu.com/s/1np_ye1Wt1Sucwv5JamCvmA)                       | `6xsa`   | 3.07GB  |
| Outdoor_Forest_dataset.bag | Forest / vegetation-rich env | [Link](https://pan.baidu.com/s/15V13tQ_k_ukbimko7NR99w)                       | `isev`   | 23.59GB |
---

## 3) Verify Dataset Integrity

1. Download bags from the links above.
2. Place each `.bag` in its corresponding folder under `dataset/`:
   - IndoorOffice1 → `dataset/indoor/IndoorOffice1/`
   - IndoorOffice2 → `dataset/indoor/IndoorOffice2/`
   - OutdoorRoad → `dataset/outdoor/OutdoorRoad/`
   - OutdoorRoad-cut0 → `dataset/outdoor/OutdoorRoad-cut0/`
   - OutdoorRoad-cut1 → `dataset/outdoor/OutdoorRoad-cut1/`
3. Run verification (example for IndoorOffice1):
   ```bash
   cd dataset/indoor/IndoorOffice1
   sha256sum -c IndoorOffice1.sha256
   ```
   Expected output:
   ```
   IndoorOffice1_dataset.bag: OK
   ```
   Repeat for the other sequences using their respective `.sha256` files.

---
## 4) Dataset Layout

Each sequence provides synchronized rosbags and metadata:

```
dataset/
  indoor/
    IndoorOffice1/
    IndoorOffice2/
  outdoor/
    OutdoorRoad/
    OutdoorRoad-cut0/
    OutdoorRoad-cut1/
```
---

## 5) ROS Topics

### IndoorOffice1 (4.47GB)

| Topic                                | #Msgs  | Type                       |
|--------------------------------------|--------|----------------------------|
| /avia/livox/imu                      | 13,461 | sensor_msgs/Imu            |
| /avia/livox/lidar                    |    662 | sensor_msgs/PointCloud2    |
| /mid360/livox/imu                    | 13,212 | sensor_msgs/Imu            |
| /mid360/livox/lidar                  |    660 | sensor_msgs/PointCloud2    |
| /ouster/imu                          |  8,257 | sensor_msgs/Imu            |
| /ouster/points                       |    661 | sensor_msgs/PointCloud2    |
| /vrpn_client_node/unitree_b1/pose    |  7,664 | geometry_msgs/PoseStamped  |

---

### IndoorOffice2 (6.46GB)

| Topic                                | #Msgs  | Type                       |
|--------------------------------------|--------|----------------------------|
| /avia/livox/imu                      | 19,449 | sensor_msgs/Imu            |
| /avia/livox/lidar                    |    957 | sensor_msgs/PointCloud2    |
| /mid360/livox/imu                    | 19,124 | sensor_msgs/Imu            |
| /mid360/livox/lidar                  |    957 | sensor_msgs/PointCloud2    |
| /ouster/imu                          | 11,939 | sensor_msgs/Imu            |
| /ouster/points                       |    955 | sensor_msgs/PointCloud2    |
| /vrpn_client_node/unitree_b1/pose    |  8,513 | geometry_msgs/PoseStamped  |

---

### OutdoorRoad (44.48GB)

| Topic               | #Msgs   | Type                      |
|---------------------|---------|---------------------------|
| /avia/livox/imu     | 133,719 | sensor_msgs/Imu           |
| /avia/livox/lidar   |   6,573 | sensor_msgs/PointCloud2   |
| /gnss_pose          |  65,732 | geometry_msgs/PoseStamped |
| /mid360/livox/imu   | 131,462 | sensor_msgs/Imu           |
| /mid360/livox/lidar |   6,573 | sensor_msgs/PointCloud2   |
| /ouster/imu         |  82,162 | sensor_msgs/Imu           |
| /ouster/points      |   6,573 | sensor_msgs/PointCloud2   |

---

### OutdoorRoad_cut0 (4.47GB)

| Topic               | #Msgs  | Type                      |
|---------------------|--------|---------------------------|
| /avia/livox/imu     | 13,429 | sensor_msgs/Imu           |
| /avia/livox/lidar   |    660 | sensor_msgs/PointCloud2   |
| /gnss_pose          |  6,602 | geometry_msgs/PoseStamped |
| /mid360/livox/imu   | 13,200 | sensor_msgs/Imu           |
| /mid360/livox/lidar |    660 | sensor_msgs/PointCloud2   |
| /ouster/imu         |  8,249 | sensor_msgs/Imu           |
| /ouster/points      |    660 | sensor_msgs/PointCloud2   |

---

### OutdoorRoad_cut1 (3.07GB)

| Topic               | #Msgs | Type                      |
|---------------------|-------|---------------------------|
| /avia/livox/imu     | 9,214 | sensor_msgs/Imu           |
| /avia/livox/lidar   |   453 | sensor_msgs/PointCloud2   |
| /gnss_pose          | 4,528 | geometry_msgs/PoseStamped |
| /mid360/livox/imu   | 9,055 | sensor_msgs/Imu           |
| /mid360/livox/lidar |   453 | sensor_msgs/PointCloud2   |
| /ouster/imu         | 5,659 | sensor_msgs/Imu           |
| /ouster/points      |   453 | sensor_msgs/PointCloud2   |

---

## Ground Truth

- **Indoor:** MoCap (`/vrpn_client_node/unitree_b1/pose`)
- **Outdoor:** GNSS-RTK (`/gnss_pose`)

---

### Sensor Frequency Notes
- `/ouster/points`: ~10 Hz  
- `/ouster/imu`: ~100 Hz  
- `/avia/livox/lidar`: 10 Hz  
- `/avia/livox/imu`: ~200 Hz  
- `/mid360/livox/lidar`: 10 Hz  
- `/mid360/livox/imu`: ~200 Hz  
---

## 6) Processing & Reproduction Pipelines

See [docs/pipelines/README.md](docs/pipelines/README.md) for detailed steps, commands, and node graphs.

**Quick overview:**
- Outdoor: GNSS→odom conversion, Livox conversion, run SLAM, record odometry + GNSS.
- Indoor: Livox conversion, run SLAM, record odometry + MoCap.
---

## 7) Trajectory Export & Evaluation

1. Export TUM files using `scripts/bag_tools/bag_tum.py`. Example:
   ```bash
   python3 scripts/bag_tools/bag_tum.py \
     --odom_bag recorded.bag --odom_topic /odometry \
     --gt_bag   recorded.bag --gt_topic /odom \
     --odom_out odom.tum --gt_out gt.tum
   ```
2. Run evaluation with `evo_ape`:
   ```bash
   evo_ape tum --align gt.tum odom.tum \
     --plot --plot_mode xyz -r trans_part --save_plot ape_trans.png
   ```

---

## 8) Baseline Methods

We tested several open-source SLAM and registration methods on this dataset:

- **FAST-LIO2** – tightly coupled LiDAR-inertial odometry ([GitHub](https://github.com/hku-mars/FAST_LIO))  
- **Faster-LIO** – optimized FAST-LIO variant ([GitHub](https://github.com/gaoxiang12/faster-lio))  
- **S-FAST-LIO** – surfel-based LiDAR-inertial odometry ([GitHub](https://github.com/zlwang7/S-FAST_LIO))  
- **FAST-LIO-SAM** – combines FAST-LIO’s front-end odometry with the loop closure and mapping backend of LIO-SAM ([GitHub](https://github.com/kahowang/FAST_LIO_SAM))  
- **GLIM** – factor graph-based LiDAR-inertial mapping ([GitHub](https://github.com/koide3/glim))  
- **KISS-ICP** – lightweight point-to-point ICP odometry, efficient and IMU-free (ROS1 compatible v0.3.0) ([GitHub](https://github.com/PRBonn/kiss-icp))  
- **GenZ-ICP** – generalized ICP variant with multi-scale feature integration for robustness ([GitHub](https://github.com/cocel-postech/genz-icp))  
- **Open3D-GICP** – Open3D’s implementation of Generalized ICP, integrated for ROS via `open3d_catkin` ([GitHub](https://github.com/leggedrobotics/open3d_slam/tree/master/open3d_catkin))  

---

## 9) Results & Visualizations

### Quantitative Results
SLAM methods (APE RMSE, mean ± std) on indoor and outdoor datasets:

![SLAM Results](docs/pipelines/img/SLAM_Experimentals_Results.png)

ICP-based methods (APE RMSE, mean ± std):

![ICP Results](docs/pipelines/img/Icps_Experimental_results.png)

---

### Trajectory Alignments
Indoor and outdoor designated ground-truth paths of all the collected data sequences:

![Ground Truth Paths](docs/pipelines/img/ground_truth_paths_collections.png)

---

### Example Outdoor SLAM Runs
<p align="center">
  <img src="docs/pipelines/img/Faster_lio.png" alt="Faster-LIO" width="45%"/>
  <img src="docs/pipelines/img/Fast_lio_sam_outdoor.png" alt="FAST-LIO-SAM" width="45%"/>
</p>

<p align="center">
  <img src="docs/pipelines/img/S_fast_lio_outdoor.png" alt="S-FAST-LIO" width="45%"/>
  <img src="docs/pipelines/img/Fast_lio2_outdoor.png" alt="FAST-LIO2" width="45%"/>
</p>

<p align="center">
  <img src="docs/pipelines/img/Glim_outdoor.png" alt="GLIM" width="45%"/>
</p>
---

## 10) Contact

Please open an issue or discussion on this repo for questions.
