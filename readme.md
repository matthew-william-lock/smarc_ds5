# smarc_ds5

[![CI](https://github.com/matthew-william-lock/smarc_ds5_ros/actions/workflows/main.yaml/badge.svg)](https://github.com/matthew-william-lock/smarc_ds5_ros/actions/workflows/main.yaml)[![license](https://img.shields.io/badge/License-MIT%203--Clause-blue.svg)](https://mit-license.org/)

Control SAM AUV using a DualSense controller.

## Launch

```bash
roslaunch smarc_ds5_ros sam_ds5.launch
```

## Controls 

| Button | Action |
| --- | --- |
| Left stick | Send RPM commands ranging from -1500 to 1500. Up indicates positive and down negative |
| Right stick | Send thrust vector commands |
| x | Toggle teleop enable |

## LED States

| State | Color |
| --- | --- |
| Teleop enabled | Green |
| Teleop disabled | Red |
| Controller connected | Blue |
| Leak detected | Flashing red |