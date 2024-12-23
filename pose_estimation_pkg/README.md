# pose_estimation_3dcam

Pose estimation module for object picking in R_car.

## Clone

```bash
git config --global credential.helper 'cache --timeout=3600'
git clone --recursive https://gitlab.ignitarium.in/ign-ai/customer/renesas/rcar/pose_estimation_3dcam.git --depth=1
```

## Installation

### Linux (PIP Based)

```bash
cd pose_estimation_3dcam
python3.8 -m venv venv3.8
venv3.8\Scripts\activate
python -m pip install -U pip

pip install -r requirements.txt
```

### Run
```bash
python main.py
```
