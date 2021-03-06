#!/bin/bash
# Execute cfg.ROOT_DIR/scripts/our_method_multi_branch.py with the appropriate arguments for many trials

# loop variable
counter=1
while [ $counter -le "$1" ]
do
  echo Trial $counter

  python -m scripts.our_method_v6 --deflection_limit "$4" --video_filename "$2"/trial"$counter".mp4 --plant_filename "$3" >> "$2"/trial"$counter".txt

  counter=$(($counter + 1))
done
