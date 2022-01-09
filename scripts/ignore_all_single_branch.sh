#!/bin/bash
# Execute cfg.ROOT_DIR/scripts/ignore_all_single_branch.py with the appropriate arguments for many trials

# loop variable
counter=1
while [ $counter -le "$1" ]
do
  echo trial $counter

  python -m scripts.ignore_all_single_branch --deflection_limit "$3" --video_filename "$2"/trial"$counter".mp4 --env "$4"  >> "$2"/trial"$counter".txt

  counter=$(($counter + 1))
done
