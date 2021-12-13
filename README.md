# Motion Planning Around Plants

## Introduction
This repository contains code and resources for the motion planning around plants 
project pursued at [ARM Lab](https://arm.eecs.umich.edu/), 
[University of Michigan](https://umich.edu/). The objective of this project was to
build a planning algorithm capable of moving an arm gently through plants to reach
a certain joint configuration. At a higher level, this algorithm was focused towards
applications such as produce harvesting and weed removal, where a manipulator can be
used to carry out these tasks. A report describing this project can be found 
[here]().


## Requirements
- numpy==1.21.2
- pybullet==3.2.0
- scipy==1.7.1


## Installation
- [TODO] explain about dependency installation
- Install this repository by executing:
```git clone ...```

## Usage
- To run the planning algorithm on a randomly generated environment, run the
following command:
```python -m scripts.out_method_v6 -h```
and add respective arguments as directed in the help prompt. 

## Scripts
All scripts used to execute the program can be found in the ```scripts``` folder.
The python scripts be run for testing of small scale tasks, but the bash scripts can
be used for running a large number of similar test cases. 