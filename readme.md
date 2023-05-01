# smarc_ds5

[![CI](https://github.com/matthew-william-lock/smarc_ds5_ros/actions/workflows/main.yaml/badge.svg)](https://github.com/matthew-william-lock/smarc_ds5_ros/actions/workflows/main.yaml) [![license](https://img.shields.io/badge/License-MIT%203--Clause-blue.svg)](https://mit-license.org/)

Control SAM AUV using a DualSense controller.

<p align="center">
  <img src="https://user-images.githubusercontent.com/53016036/235476324-ffab01e0-7e11-438f-a3e0-7eedcea22abf.png" width="100%">
</p>

## Installation

Steps for installation are taken from the [pydualsense](https://github.com/flok/pydualsense) dependency.

You first need to add a udev rule to let the user access the PS5 controller without requiring root privileges.

```bash
sudo cp 70-ps5-controller.rules /etc/udev/rules.d
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then install the ```libhidapi-dev```.

```
sudo apt install libhidapi-dev
```

After that install the package from [pypi](https://pypi.org/project/pydualsense/).

```bash
pip install pydualsense
```

## Launch

```bash
roslaunch smarc_ds5_ros sam_ds5.launch
```

## Controls 

| Button | Action |
| --- | --- |
| ```Left stick``` | Send RPM commands ranging from ```-1500``` to ```1500```. ```Up``` indicates positive and ```down``` negative |
| ```Right stick``` | Send thrust vector commands |
| ```x``` | Toggle teleop enable |

## LED States

| State | Color |
| --- | --- |
| ```Teleop enabled``` | Green |
| ```Teleop disabled``` | Red |
| ```Controller connected``` | Blue |
| ```Leak detected``` | Flashing red |
