# Third-party code and what existed before 10:00 Saturday

| Used | Source | How |
|---|---|---|
| Slamtec RPLIDAR SDK (`ultra_simple`) | https://github.com/Slamtec/rplidar_sdk | LiDAR driver, unmodified, piped into lidar_node.py |
| hailo-apps detection pipeline | https://github.com/hailo-ai/hailo-apps | YOLO person detection on the AI HAT; we only edit its callback |
| crazyflie-lib-python (cflib) | https://github.com/bitcraze/crazyflie-lib-python | Drone link + MotionCommander |
| scikit-learn, numpy, matplotlib, paho-mqtt, pyserial | PyPI | libraries |
| Seeed Grove DHT library (optional) | Arduino Library Manager | DHT11 reads |

Written this weekend: everything in `pi/`, `arduino/`, `tests/`.
