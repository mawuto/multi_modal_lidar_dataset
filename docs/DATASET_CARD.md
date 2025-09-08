# Dataset Card: Multi-Modal LiDAR Dataset

## Summary
Multi-LiDAR dataset including Avia, Mid-360, and Ouster OS0-128.  
Ground truth: **MoCap (indoor)** and **GNSS-RTK (outdoor)**.

## Download

- IndoorOffice1_dataset.bag: https://pan.baidu.com/s/1sAiaXva7OY0z7ILK0nIv1w (pwd: `ec2t`) (Size: 4.47GB, SHA256: cd4cd4290c10993086c1e63b2edc0a968333e686a24b8a195154749365fe2dae)
- IndoorOffice2_dataset.bag: https://pan.baidu.com/s/1g64HCeztEmPRc-rdee_oew (pwd: `5yxp`) (Size: 6.46GB, SHA256: aee8e03fc18b122c209ae9d4e272c38e26c3193fbc7a1843d92deb325bae413b)
- OutdoorRoad_dataset.bag: https://pan.baidu.com/s/1O7MBU-5u8taKWxgpxwJJwQ (pwd: `uk1e`) (Size: 44.48GB, SHA256: 71678d06a6db45cef4042d1b4124c3b16a48723720577d5af89ff32ae27d9bf1)
- OutdoorRoad_cut0.bag: https://pan.baidu.com/s/1D3BiAND8qwbKwoCiSFW0sA (pwd: `eyu9`) (Size: 4.47GB, SHA256: f12e1a37429533e158ff28c1bd0b9062a134a5ecfeeda044244b98c52862acf6)
- OutdoorRoad_cut1.bag: https://pan.baidu.com/s/1np_ye1Wt1Sucwv5JamCvmA (pwd: `6xsa`) (Size: 3.07GB, SHA256: 75086390002340c013f1ca8fa02062522f2af286c3f479377850a85cda475799)

## Notes on Sensors

- **Avia:** frame-level timestamps only (10 Hz), IMU 200 Hz. Fields: x,y,z,intensity,tag,line.  
- **Mid-360:** per-point timestamps (`timestamp` field, float64), 10 Hz; IMU 200 Hz.  
- **Ouster:** per-point offsets (`t`), 10 Hz; IMU 100 Hz. Fields include reflectivity, ring, ambient, range.

## Ground Truth
- Indoor: MoCap (`/vrpn_client_node/unitree_b1/pose`)  
- Outdoor: GNSS‑RTK (`/gnss_pose`)

## Terms
Data: CC BY 4.0, Code: MIT.
