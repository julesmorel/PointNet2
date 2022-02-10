# Pointnet++ (PyTorch)

* PyTorch implementation of [PointNet++](https://arxiv.org/abs/1706.02413) based on [erikwijmans/Pointnet2_PyTorch](https://github.com/erikwijmans/Pointnet2_PyTorch) and [sshaoshuai/Pointnet2.PyTorch](https://github.com/sshaoshuai/Pointnet2.PyTorch).
* Faster than the original codes by re-implementing the CUDA operations.
* Code updated to run with CUDA Toolkit 11.3

## Installation
### Requirements
* Linux (tested on Ubuntu 21.10)
* Python 3.9.7
* PyTorch 1.10.2
* CUDA Toolkit 11.3

### Install
Install this library by running the following command:

```shell
cd pointnet2
python setup.py install
cd ../
```

## Training
Work in progress

## Assertion
Work in progress

## Acknowledgement
* [charlesq34/pointnet2](https://github.com/charlesq34/pointnet2): Paper author and official code repo.
* [erikwijmans/Pointnet2_PyTorch](https://github.com/erikwijmans/Pointnet2_PyTorch): Initial work of PyTorch implementation of PointNet++.
* [sshaoshuai/Pointnet2.PyTorch](https://github.com/sshaoshuai/Pointnet2.PyTorch): Update of the PyTorch implementation.
