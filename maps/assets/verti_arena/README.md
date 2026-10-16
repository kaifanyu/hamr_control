# Verti-Arena Extractor

A ROS-based tool for extracting synchronized, gap-filled robot-centric elevation and RGB map patches from robot-recorded bags.

---

## Features

- **Inline Static Map Loader (`StaticMap`)**: Completely self-contained parser in `Grid.py` that loads arena maps and queries local patches on CPU or CUDA without any external package dependencies.
- **In-place Multi-layer Gap-Filling**: Fills unobserved/missing cells in raw incoming ROS `GridMap` layers (such as `elevation` and `rgb`) using global static map data.
- **Robust Color-mapping**: Safely handles standard ROS GridMap color conventions (where unobserved pixels default to `0.0` or `NaN` floats) and falls back to a custom default semantic color when no static map data is available.
- **Robot-Centric Patch Rotation**: Extracts squared patches of configurable dimensions in meters around the robot's pose and rotates them by `-yaw` so that the robot's heading points forward.

---

## Setup & Environment

Ensure that your ROS workspace is sourced before running any of the scripts:
```bash
source /home/robotixx/wait_ws/devel/setup.bash
```

---

## Main Scripts

### 1. Dataset Extractor (`dataset_extractor.py`)
Extracts a synchronized dataset of robot states, velocities, control actions, and rotated map patches from a ROS bag at a specified rate (Hz) and saves it to a Python pickle file.

```bash
python3 dataset_extractor.py --bag_path bags/test2_2026-07-03-17-16-39.bag --output_path dataset.pkl --rate 10.0 --patch_size 1.0
```

#### Parameters:
- `--bag_path`: Path to input ROS bag file (defaults to `bags/test2_2026-07-03-17-16-39.bag`).
- `--output_path`: Path to output pickle file (defaults to `dataset.pkl`).
- `--rate`: Resampling rate in Hz (defaults to `10.0`).
- `--patch_size`: Dimensions of the extracted square patch in meters (defaults to `1.0`).

#### Extracted Data Structure (per sample):
- `timestamp`: Timestamp of the sample.
- `state`: `[x, y, z, roll, pitch, yaw]` (where roll/pitch are estimated using terrain footprint heights).
- `velocities`: `[vx, vy, vz, wx, wy, wz]` (linear and angular velocities).
- `control`: `[steering, throttle]` (where steering is negative angular velocity `-angular.z` and throttle is `linear.x`).
- `elevation_patch`: Rotated $(H, W)$ `float32` elevation array.
- `rgb_patch`: Rotated $(H, W, 3)$ `uint8` RGB image array.

---

### 2. Plot Example (`plot_example.py`)
Extracts and plots side-by-side robot-centric Elevation patches, RGB patches, and odometry details at sample times from the bag.

```bash
python3 plot_example.py
```
Output is saved to `example_extraction.png`.

---

### 3. Extract Example (`extract_example.py`)
Reads the first message of topics in the target ROS bag and prints metadata details showing that 100% of gaps (`NaN`s / `0.0`s) in the map layers have been successfully filled.

```bash
python3 extract_example.py
```
