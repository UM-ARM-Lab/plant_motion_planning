#!/bin/bash
# Execute cfg.ROOT_DIR/scripts/our_method_single_branch.py with the appropriate arguments for many trials

# loop variable
counter=6
while [ $counter -le "$1" ]
do
  echo Trial $counter

  python -m scripts.our_method_v4 --deflection_limit "$4" --video_filename "$2"/trial"$counter".mp4 --env "$3" >> "$2"/trial"$counter".txt

  counter=$(($counter + 1))
done
