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
- **Outdoor:** GNSS‑RTK (`/gnss_pose`)

---

## 4) Contact

Please open an issue or discussion on this repo for questions.
