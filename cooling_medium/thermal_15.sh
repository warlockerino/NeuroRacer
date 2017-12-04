#!/bin/bash
echo 'Device 15:'
cat /sys/class/thermal/cooling_device15/type
cat /sys/class/thermal/cooling_device15/cur_state
cat /sys/class/thermal/cooling_device15/max_state
