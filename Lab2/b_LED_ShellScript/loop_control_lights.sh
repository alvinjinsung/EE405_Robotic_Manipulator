#!/bin/bash

# loop_control_lights.sh
# This code tests for the time takes for LED light control for many loops.

echo "loop_control_lights.sh"

# -----------------------------------------------

# 1) set GPIO 30/31 as export
# !! REPLACE THIS PART TO YOUR CODE !!
echo "Export: Get access permission for GPIO30 & 31"
echo 30 > /sys/class/gpio/export
echo 31 > /sys/class/gpio/export

# -----------------------------------------------

# -----------------------------------------------

# 2) set GPIO 30/31 direction as output
# !! REPLACE THIS PART TO YOUR CODE !!
echo "Set directions of GPIO 30 & 31 as output"
echo out > /sys/class/gpio/gpio30/direction
echo out > /sys/class/gpio/gpio31/direction

# -----------------------------------------------

# 3) save the start time(before the light control)
start=$(date +%s.%N)

for ((i=0;i<10;i++))
do

  # -----------------------------------------------

  # 4) LED Light control loop - Please refer to the experiment guide
  # !! REPLACE THIS PART TO YOUR CODE !!
	echo 1 > /sys/class/gpio/gpio30/value
	echo 1 > /sys/class/gpio/gpio31/value

	echo 0 > /sys/class/gpio/gpio30/value
	echo 0 > /sys/class/gpio/gpio31/value

  # -----------------------------------------------

done

# 5) save the end time(after the light control)
end=$(date +%s.%N)
echo "Start:$start*I End:$end"

# -----------------------------------------------

# 6) set GPIO 30/31 direction as input
# !! REPLACE THIS PART TO YOUR CODE !!
echo "Set directions of GPIO 30 & 31 as input"
echo in > /sys/class/gpio/gpio30/direction
echo in > /sys/class/gpio/gpio31/direction

# -----------------------------------------------

# -----------------------------------------------

# 7) set GPIO 30/31 as unexport
# !! REPLACE THIS PART TO YOUR CODE !!
echo "UnExport: Release access permission for GPIO30 & 31"
echo 30 > /sys/class/gpio/unexport
echo 31 > /sys/class/gpio/unexport

# -----------------------------------------------

