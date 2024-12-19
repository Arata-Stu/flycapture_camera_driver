```shell
## start
ros2 service call /set_recording std_srvs/srv/SetBool "{data: true}"

## end
ros2 service call /set_recording std_srvs/srv/SetBool "{data: false}"

## build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```