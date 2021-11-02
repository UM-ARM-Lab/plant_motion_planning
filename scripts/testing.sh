#!/bin/bash

counter=1
while [ $counter -le 30 ]
do
  echo trial number $counter
  python -m scripts.move_kuka_branches_angle_based_v4 >> ../testing.txt

  counter=$(($counter + 1))
done