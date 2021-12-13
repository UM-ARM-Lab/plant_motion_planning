#!/bin/bash
# Execute cfg.ROOT_DIR/scripts/avoid_all.py with the appropriate arguments for many trials

# loop variable
counter=1
while [ $counter -le "$1" ]
do
  echo trial $counter

#  python -m scripts.move_kuka_avoid_all_with_controls --deflection_limit "$3" --video_filename "$2"/env"$counter"/trial1.mp4  >> "$2"/env"$counter"/trial1.txt
#  python -m scripts.move_kuka_avoid_all_with_controls --deflection_limit "$3" --video_filename "$2"/env"$counter"/trial1.mp4
  python -m scripts.avoid_all --deflection_limit "$3" --video_filename "$2"/trial"$counter".mp4 --plant_filename "$4"  >> "$2"/trial"$counter".txt

  counter=$(($counter + 1))
done