#!/bin/bash
# Execute cfg.ROOT_DIR/scripts/our_method_v6.py with the appropriate arguments for many trials

# loop variable
counter=1
while [ $counter -le "$1" ]
do
  echo Trial $counter

#  python -m scripts.move_kuka_branches_angle_based_v6 --deflection_limit "$3" --video_filename "$2"/trial"$counter".mp4 --plant_filename "$4"  >> "$2"/trial"$counter".txt
  python -m scripts.our_method_v4 --deflection_limit "$4" --video_filename "$2"/trial"$counter".mp4 --env "$3" >> "$2"/trial"$counter".txt

  counter=$(($counter + 1))
done