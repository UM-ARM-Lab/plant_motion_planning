#!/bin/bash
# Execute cfg.ROOT_DIR/scripts/ignore_all.py with the appropriate arguments for many trials

# loop variable
counter=1
while [ $counter -le "$1" ]
do
  echo Trial $counter

  python -m scripts.ignore_all --deflection_limit "$3" --video_filename "$2"/trial"$counter".mp4 --plant_filename "$4"  >> "$2"/trial"$counter".txt
#  python -m scripts.move_kuka_avoid_all_with_controls --deflection_limit "$3" --video_filename "$2"/env"$counter"/trial1.mp4

  counter=$(($counter + 1))
done