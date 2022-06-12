# Segmentation based on Pointnet++ (PyTorch)

**Processing pipeline designed to segment point clouds acquired in forests** 

-----------------
## Overview

This method relies on both a geometric and a deep learning approach to:
1. Identify the ground points on a complete TLS scan ([Morel et al. 2020](https://link.springer.com/chapter/10.1007/978-3-030-50433-5_20)).
2. Separate the wood points from the leaves points on scanned trees ([Morel et al. 2020](https://link.springer.com/article/10.1007/s00371-020-01966-7)).

-----------------
## Setup

* Geometric tools implemented as a suite of independent C++ programs using [PCL](https://pointclouds.org/)
* PyTorch implementation of [PointNet++](https://arxiv.org/abs/1706.02413) based on [erikwijmans/Pointnet2_PyTorch](https://github.com/erikwijmans/Pointnet2_PyTorch).
* Code updated to run with CUDA Toolkit 11.3

See the official code release "PointNet++" for model definitions and hyper-parameters.
The custom operations used by Pointnet++ are **ONLY** supported using CUDA.

### Requirements
* Linux (tested on Ubuntu 21.10)
* PCL 1.11
* Python 3.9.7
* PyTorch 1.10.2
* CUDA Toolkit 11.3
* Visdom
* CloudCompare

### Install
Install this library by running the following command:
```shell
./install.sh
```  

-----------------
## Segmentation
Several pre-trained models are provided in this package, they are stored into the `models` folder.

    .
    ├── ...
    ├── models                    	  # Pytorch models file
    │   ├── terrain_segmentation  	  #   Terrain/Vegetation segmentation models 
	│   │   ├── model_terrain_01_128  #     Fine model (2D grid resolution:10cm, N neighbors PCA:128) 
	│   │   └── model_terrain_02_64   #     Coarse model (2D grid resolution:20cm, N neighbors PCA:64) 
    │   ├── wood_segmentation    	  #   Wood/leaves segmentation models 	
	│   │   ├── model_seg_landes	  #     Model trained on vegetation from Landes (Radius PCA:5cm)	
	│   │   ├── model_seg_sologne	  #     Model trained on vegetation from Sologne (Radius PCA:5cm)	
	│   │   └── model_seg_vosges      #     Model trained on vegetation from Vosges (Radius PCA:5cm)						          
    └── ...
	
**For information purposes only:** 
while terrain and wood segmentation rely on a similar sequence of computanional steps, they only differ by the initial filtering on the input point cloud and by the computation of the geometric local descritors:

1. Segmentation ground points: filtering of the input scan through a coarse 2D XY grid (~10cm), which tends to make the point density unifrom so the local descriptors are computed with PCA amongst the K neighbors
2. Segmentation wood points: filtering of the input scan through a fine 3D grid (0.5cm). As the point density stays non uniform the local descriptors are computed with PCA considering the nighbors in a sphere of given radius.
	
###	Segmentation of the ground points

![screenshot](images/terrain.jpg?raw=true "terrain")
	
In order to segment the ground points from the vegetation points, first edit the parameters in `segment_terrain.sh` then call the script:
	
```bash
./segment_terrain.sh INPUT_FILE_1 ... INPUT_FILE_N MODEL_PATH
```
where INPUT_FILE_1 ... INPUT_FILE_N (N>=1) are the paths to the files containing the point clouds to segment and MODEL_PATH is the path to the model used for the inference.

###	Segmentation of the wood points

![screenshot](images/wood.jpg?raw=true "wood")

In order to segment the wood from the leaves points, first edit the parameters in `segment_wood.sh` then call the script:
	
```bash
./segment_wood.sh INPUT_FILE MODEL_PATH
```
where INPUT_FILE is the path to the file containing the point cloud to segment and MODEL_PATH is the path to the model used for the inference.  

-----------------
## Additional steps
Two additional computation steps have been implemented to improve the terrain segmentation results:
1. a filtering method based on Statistical Outlier Removal and Radius Outlier Removal to clean the input point cloud 
2. a clustering method which aims at cleaning the segmentation results

### Filtering
As a preliminary step before the inference, filtering of the noise can be applied by a custom filter based on Statistical Outlier Removal and Radius Outlier Removal:
```bash
.outliersFilter/outliersFilter INPUT_FILE OUTPUT_FILE meanK stddevMulThresh radiusSearch minNeighborsInRadius
```
where:
* INPUT_FILE and OUTPUT_FILE are the paths to the ascii file in input and output respectively.
* meanK and stddevMulThresh are the number of neighbors to analyze for each point and the standard deviation multiplier.
* radiusSearch is the sphere radius that is to be used for determining the k-nearest neighbors for filtering and minNeighborsInRadius is the minimum number of neighbors that a point needs to have in the given search radius in order to be considered an inlier.

We usually use:
```bash
.outliersFilter/outliersFilter INPUT_FILE OUTPUT_FILE 128 1.0 1. 50
```

### Clustering
We observed that the usual segmentation errors result in small patch of points, clearly away from the main ground points cluster. 

![screenshot](images/cluster.png?raw=true "clustering")

To cope with this issue, we propose to retrieve the main cluster of points using the following script: 
```bash
.clustering/clustering INPUT_FILE OUTPUT_FILE clusterTolerance minClusterSize maxClusterSize
```
where:
* INPUT_FILE and OUTPUT_FILE are the paths to the ascii file in input and output respectively.
* clusterTolerance is the minimum distance between 2 clusters
* minClusterSize and maxClusterSize are the lower and upper limits of the clusters size

We usually use:
```bash
.clustering/clustering INPUT_FILE OUTPUT_FILE 0.2 10 10000000
```  

-----------------
## Training of a custom model

The library provides also several scritps to train new pytorch models. The data should be labelized point clouds stored in ASCII files (4 columns: X Y Z label) and separated in 2 folders: training and validation.

### Dataset preparation
Given TRAIN_DIR and VALIDATION_DIR, both directories containing the training data and the validation data respectively, format the input data by running:
```bash
./prepare_data_terrain.sh TRAIN_DIR
./prepare_data_terrain.sh VALIDATION_DIR
```

(replace `prepare_data_terrain.sh` with `prepare_data_wood.sh` if you are training a model to segment wood from leaves points)

### Training
Edit train.sh to setup the parameters, then run:
```bash
./train.sh
```

## Acknowledgement
* [charlesq34/pointnet2](https://github.com/charlesq34/pointnet2): Paper author and official code repo.
* [erikwijmans/Pointnet2_PyTorch](https://github.com/erikwijmans/Pointnet2_PyTorch): Initial work of PyTorch implementation of PointNet++.
