import os.path as osp
import os
import argparse
import sys
import numpy as np
from tqdm import tqdm

def iter_loadtxt(filename, delimiter=' ', dtype=float):
    def iter_func():
        with open(filename, 'r') as infile:
            for line in infile:
                line = line.rstrip().split(delimiter)
                for item in line:
                    yield dtype(item)
        iter_loadtxt.rowlength = len(line)

    data = np.fromiter(iter_func(), dtype=dtype)
    data = data.reshape((-1, iter_loadtxt.rowlength))
    return data

def loadPointsAndLabels(f,num_points,use_pca,use_intensity):
    path = os.path.dirname(os.path.realpath(__file__))
    print("   reading file ",f)
    txt = iter_loadtxt(os.path.join(path, f))
    if use_pca:
        if use_intensity:
            pts = txt[:, [0, 1, 2, 3, 4, 5, 6]]
        else:
            pts = txt[:, [0, 1, 2, 3, 4, 5]]
    else:
        pts = txt[:, [0, 1, 2]]

    if use_pca:
        if use_intensity:
            lab = txt[:, 7]
        else:
            lab = txt[:, 6]
    else:
        lab = txt[:, 3]

    return  pts, lab

def loadPointsAndLabelsInDirectory(p,num_points,use_pca,use_intensity):
    path = os.path.dirname(os.path.realpath(__file__))
    train_path = os.path.join(path, p)
    filenames = [d for d in os.listdir(train_path)]
    numberFiles = len(filenames)
    print("Reading "+str(numberFiles)+" files in "+train_path)
    id=1
    points_all = None
    labels_all = None

    for f in tqdm(filenames):
        id=id+1
        txt = iter_loadtxt(os.path.join(train_path, f))
        if use_pca:
            if use_intensity:
                pts = txt[:, [0, 1, 2, 3, 4, 5, 6]]
            else:
                pts = txt[:, [0, 1, 2, 3, 4, 5]]
        else:
            pts = txt[:, [0, 1, 2]]

        if use_pca:
            if use_intensity:
                lab = txt[:, 7]
            else:
                lab = txt[:, 6]
        else:
            lab = txt[:, 3]

        if points_all is None or labels_all is None:
            points_all = pts
            labels_all = lab
        else:
            points_all = np.vstack((points_all, pts))
            labels_all = np.concatenate((labels_all, lab), axis=0)

    return  points_all, labels_all

def recenterPoints(data,num_points,use_pca,use_intensity):
    sh = data.shape
    nbDiv = int(sh[0]/num_points)
    normData = data
    for i in range(nbDiv):
        xc = np.sum(data[i*num_points:(i+1)*num_points,0])
        yc = np.sum(data[i*num_points:(i+1)*num_points,1])
        zc = np.sum(data[i*num_points:(i+1)*num_points,2])
        normData[i*num_points:(i+1)*num_points,0]=data[i*num_points:(i+1)*num_points,0]-(xc/num_points)
        normData[i*num_points:(i+1)*num_points,1]=data[i*num_points:(i+1)*num_points,1]-(xc/num_points)
        normData[i*num_points:(i+1)*num_points,2]=data[i*num_points:(i+1)*num_points,2]-(xc/num_points)

        if use_pca:
            if use_intensity:
                normData[i*num_points:(i+1)*num_points,3]=data[i*num_points:(i+1)*num_points,3]
                normData[i*num_points:(i+1)*num_points,4]=data[i*num_points:(i+1)*num_points,4]
                normData[i*num_points:(i+1)*num_points,5]=data[i*num_points:(i+1)*num_points,5]
                normData[i*num_points:(i+1)*num_points,6]=data[i*num_points:(i+1)*num_points,6]
            else:
                normData[i*num_points:(i+1)*num_points,3]=data[i*num_points:(i+1)*num_points,3]
                normData[i*num_points:(i+1)*num_points,4]=data[i*num_points:(i+1)*num_points,4]
                normData[i*num_points:(i+1)*num_points,5]=data[i*num_points:(i+1)*num_points,5]

    return normData
