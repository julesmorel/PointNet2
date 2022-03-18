# Pointnet++ (PyTorch)

* PyTorch implementation of [PointNet++](https://arxiv.org/abs/1706.02413) based on [erikwijmans/Pointnet2_PyTorch](https://github.com/erikwijmans/Pointnet2_PyTorch).
* Code updated to run with CUDA Toolkit 11.3

See the official code release "PointNet++" for model definitions and hyper-parameters.
The custom operations used by Pointnet++ are **ONLY** supported using CUDA.

## Setup
### Requirements
* Linux (tested on Ubuntu 21.10)
* PCL 1.11
* Python 3.9.7
* PyTorch 1.10.2
* CUDA Toolkit 11.3
* Visdom

### Install
Install this library by running the following command:
```shell
./install.sh
```

## Dataset preparation
Given TRAIN_DIR and VALIDATION_DIR, both directories containing the training data and the validation data respectively, format the input data by running:
```bash
./prepareData.sh TRAIN_DIR
./prepareData.sh VALIDATION_DIR
```

## Training
Edit train.sh to setup the parameters, then run:
```bash
./train.sh
```

## Filtering
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

## Inference
```bash
./predict.sh INPUT_FILE MODEL_PATH
```
where INPUT_FILE is the path to the file containing the point cloud to segment and MODEL_PATH is the path to the model used for the inference.

## Acknowledgement
* [charlesq34/pointnet2](https://github.com/charlesq34/pointnet2): Paper author and official code repo.
* [erikwijmans/Pointnet2_PyTorch](https://github.com/erikwijmans/Pointnet2_PyTorch): Initial work of PyTorch implementation of PointNet++.
