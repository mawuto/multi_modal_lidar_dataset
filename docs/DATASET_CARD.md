# Dataset Card: Multi-Modal LiDAR Dataset

## Summary
Multi-LiDAR dataset including Avia, Mid-360, and Ouster OS0-128.  
Ground truth: **MoCap (indoor)** and **GNSS-RTK (outdoor)**.

## Download

- IndoorOffice1_dataset.bag: https://pan.baidu.com/s/1sAiaXva7OY0z7ILK0nIv1w (pwd: `ec2t`) (Size: TBD, SHA256: TBD)
- IndoorOffice2_dataset.bag: https://pan.baidu.com/s/1g64HCeztEmPRc-rdee_oew (pwd: `5yxp`) (Size: TBD, SHA256: TBD)
- OutdoorRoad_dataset.bag: https://pan.baidu.com/s/1O7MBU-5u8taKWxgpxwJJwQ (pwd: `uk1e`) (Size: TBD, SHA256: TBD)
- OutdoorRoad_cut0.bag: https://pan.baidu.com/s/1D3BiAND8qwbKwoCiSFW0sA (pwd: `eyu9`) (Size: TBD, SHA256: TBD)
- OutdoorRoad_cut1.bag: https://pan.baidu.com/s/1np_ye1Wt1Sucwv5JamCvmA (pwd: `6xsa`) (Size: TBD, SHA256: TBD)

## Notes on Sensors

- **Avia:** frame-level timestamps only (10 Hz), IMU 200 Hz. Fields: x,y,z,intensity,tag,line.  
- **Mid-360:** per-point timestamps (`timestamp` field, float64), 10 Hz; IMU 200 Hz.  
- **Ouster:** per-point offsets (`t`), 10 Hz; IMU 100 Hz. Fields include reflectivity, ring, ambient, range.

## Ground Truth
- Indoor: MoCap (`/vrpn_client_node/unitree_b1/pose`)  
- Outdoor: GNSS‑RTK (`/gnss_pose`)

## Terms
Data: CC BY 4.0, Code: MIT.
