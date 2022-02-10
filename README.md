# Pointnet++ (PyTorch)

* PyTorch implementation of [PointNet++](https://arxiv.org/abs/1706.02413) based on [erikwijmans/Pointnet2_PyTorch](https://github.com/erikwijmans/Pointnet2_PyTorch).
* Code updated to run with CUDA Toolkit 11.3

See the official code release "PointNet++" for model definitions and hyper-parameters.
The custom operations used by Pointnet++ are **ONLY** supported using CUDA.

## Installation
### Requirements
* Linux (tested on Ubuntu 21.10)
* PCL 1.11
* Python 3.9.7
* PyTorch 1.10.2
* CUDA Toolkit 11.3

### Install
Install this library by running the following command:
```shell
./install.sh
```

## Training
### Data formatting
Given TRAIN_DIR and VALIDATION_DIR, both directories containing the training data and the validation data respectively, format the input data by running:
```bash
./prepareData.sh TRAIN_DIR
./prepareData.sh VALIDATION_DIR
```
Edit train.sh to setup the parameters, then run:
```
./train.sh
```

## Inference

```bash
./predict.sh INPUT_FILE MODEL_PATH
```
where INPUT_FILE is the path to the file containing the point cloud to segment and MODEL_PATH is the path to the model used for the inference.

## Acknowledgement
* [charlesq34/pointnet2](https://github.com/charlesq34/pointnet2): Paper author and official code repo.
* [erikwijmans/Pointnet2_PyTorch](https://github.com/erikwijmans/Pointnet2_PyTorch): Initial work of PyTorch implementation of PointNet++.
