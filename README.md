# Motion Planning Around Plants

## Introduction
This repository contains code and resources for the motion planning around plants 
project pursued at [ARM Lab](https://arm.eecs.umich.edu/), 
[University of Michigan](https://umich.edu/). The objective of this project was to
build a planning algorithm capable of moving an arm gently through plants to reach
a certain joint configuration. At a higher level, this algorithm was focused towards
applications such as produce harvesting and weed removal, where a manipulator can be
used to carry out these tasks. Planning under uncertainty was also considered to incorporate the unpredictability of the real world.


## Requirements
- numpy==1.21.2
- pybullet==3.2.0
- scipy==1.7.1


## Installation
- Install the PyBullet simulator: [PyBullet](https://github.com/bulletphysics/bullet3).
- Download this repository by executing:
```git clone ...```
- Install dependent libraries using: ```pip3 install -e .```

## Usage
- To run the planning algorithm on a randomly generated single-world environment with a single-stemmed plant, run the
following command:
```python -m scripts.our_method_single_branch -h```
and add respective arguments as directed in the help prompt. 
- To run the planning algorithm on a randomly generated single-world environment with a multi-branched plant, run the
following command:
```python -m scripts.our_method_multi_branch -h```
and add respective arguments as directed in the help prompt. 
- To run the planning algorithm on a randomly generated multi-world environment with a multi-branched plant, run the
following command:
```python -m scripts.multi_world_our_method -h```
and add respective arguments as directed in the help prompt. To test some sample environments,
use the default seeds in the program.


## Scripts
All scripts used to execute the program can be found in the ```scripts``` folder.
The python scripts can be run for testing a few test cases, but the bash scripts can
be used for running a large number of similar test cases. 
