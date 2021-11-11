#!/bin/bash

#python -m scripts.move_kuka_branches_angle_based_v4 --env env$1 --video_filename ../testing_data/angle_constraint_with_controls/video_recordings/env"$1"/trial$x.mp4 > ../testing_data/angle_contraint_with_controls/program_text_output/env"$1"/trial$x.txt

#echo scripts.move_kuka_branches_angle_based_v4 --env "$2" --video_filename ../testing_data/angle_constraint_with_controls/video_recordings/"$2"/trial"$counter".mp4 > ../testing_data/angle_constraint_with_controls/program_text_output/"$2"//trial"$counter".txt

pwd

counter=1
while [ $counter -le "$1" ]
do
#  echo trial number $counter

  python -m scripts.move_kuka_branches_angle_based_v4 --env "$2" --video_filename ../testing_data/angle_constraint_with_controls/video_recordings/"$2"/trial"$counter".mp4 > ../testing_data/angle_constraint_with_controls/program_text_output/"$2"/trial"$counter".txt

  counter=$(($counter + 1))
done