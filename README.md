# Multi-Modal LiDAR Dataset (Avia • Mid-360 • Ouster)

> **Status:** Pre-release (dataset links below).  
> **Paper:** *Understanding LiDAR Variability: A Dataset and Comparative Study of Solid-State and Spinning LiDARs* (under review).

This repository hosts **documentation, download links, and baseline code** for a multi-LiDAR dataset featuring:
- **Livox Avia** (solid-state, limited FoV)
- **Livox Mid-360** (dome-shaped solid-state)
- **Ouster OS0-128** (spinning)

---

## 1) Quick Links

**Dataset Download (Baidu Netdisk):**
- IndoorOffice1_dataset.bag — https://pan.baidu.com/s/1sAiaXva7OY0z7ILK0nIv1w (pwd: `ec2t`)
- IndoorOffice2_dataset.bag — https://pan.baidu.com/s/1g64HCeztEmPRc-rdee_oew (pwd: `5yxp`)
- OutdoorRoad_dataset.bag — https://pan.baidu.com/s/1O7MBU-5u8taKWxgpxwJJwQ (pwd: `uk1e`)
- OutdoorRoad_cut0.bag — https://pan.baidu.com/s/1D3BiAND8qwbKwoCiSFW0sA (pwd: `eyu9`)
- OutdoorRoad_cut1.bag — https://pan.baidu.com/s/1np_ye1Wt1Sucwv5JamCvmA (pwd: `6xsa`)

---

## 2) Dataset Layout

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

## 3) Ground Truth

- **Indoor:** MoCap (`/vrpn_client_node/unitree_b1/pose`)
- **Outdoor:** GNSS-RTK (`/gnss_pose`)

---

## 4) Verify Dataset Integrity

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

## 5) ROS Topics

### IndoorOffice1 & IndoorOffice2

**Message types**
- `geometry_msgs/PoseStamped`  
- `sensor_msgs/Imu`   
- `sensor_msgs/PointCloud2`  

**Topics**
- `/avia/livox/imu` — 13,461 msgs : `sensor_msgs/Imu`  
- `/avia/livox/lidar` — 662 msgs : `sensor_msgs/PointCloud2`  
- `/mid360/livox/imu` — 13,212 msgs : `sensor_msgs/Imu`  
- `/mid360/livox/lidar` — 660 msgs : `sensor_msgs/PointCloud2`  
- `/ouster/imu` — 8,257 msgs : `sensor_msgs/Imu`  
- `/ouster/points` — 661 msgs : `sensor_msgs/PointCloud2`  
- `/vrpn_client_node/unitree_b1/pose` — 7,664 msgs : `geometry_msgs/PoseStamped`  

---

### OutdoorRoad (full & cut sequences)

**Message types**
- `geometry_msgs/PoseStamped`  
- `sensor_msgs/Imu`   
- `sensor_msgs/PointCloud2`  

**Topics**
- `/avia/livox/imu` — 133,719 msgs : `sensor_msgs/Imu`  
- `/avia/livox/lidar` — 6,573 msgs : `sensor_msgs/PointCloud2`  
- `/gnss_pose` — 65,732 msgs : `geometry_msgs/PoseStamped`  
- `/mid360/livox/imu` — 131,462 msgs : `sensor_msgs/Imu`  
- `/mid360/livox/lidar` — 6,573 msgs : `sensor_msgs/PointCloud2`  
- `/ouster/imu` — 82,162 msgs : `sensor_msgs/Imu`  
- `/ouster/points` — 6,573 msgs : `sensor_msgs/PointCloud2`  

---

### Sensor Frequency Notes
- `/ouster/points`: ~10 Hz  
- `/ouster/imu`: ~100 Hz  
- `/avia/livox/lidar`: 10 Hz  
- `/avia/livox/imu`: ~200 Hz  
- `/mid360/livox/lidar`: 10 Hz  
- `/mid360/livox/imu`: ~200 Hz

## 6) Contact

Please open an issue or discussion on this repo for questions.
