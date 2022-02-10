from __future__ import (
    division,
    absolute_import,
    with_statement,
    print_function,
    unicode_literals,
)
import torch
import torch.utils.data as data
import numpy as np
import os
import utils as utils

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

class Tree3DSemSeg(data.Dataset):
    def __init__(self, num_points, path, use_pca, use_intensity, train=True, download=True, data_precent=1.0):
        super().__init__()
        self.data_precent = data_precent
        self.folder = path
        self.data_dir = os.path.join(BASE_DIR, self.folder)

        self.train, self.num_points = train, num_points

        data_training, labels_training = utils.loadPointsAndLabelsInDirectory(self.folder,self.num_points,use_pca,use_intensity)
        data_training = utils.recenterPoints(data_training,self.num_points,use_pca,use_intensity)

        if use_pca:
            if use_intensity:
                self.points = data_training.reshape(-1, self.num_points, 7)
            else:
                self.points = data_training.reshape(-1, self.num_points, 6)
        else:
            self.points = data_training.reshape(-1, self.num_points, 3)
        self.labels = labels_training.reshape(-1, self.num_points)
        print(self.points.shape)

        nbChunks = self.points.shape[0]
        print(str(nbChunks)+ " samples (size : "+str(self.num_points)+" x "+ str(self.points.shape[2])+")")

    def __getitem__(self, idx):
        pt_idxs = np.arange(0, self.num_points)
        np.random.shuffle(pt_idxs)

        current_points = torch.from_numpy(self.points[idx, pt_idxs].copy()).type(
            torch.FloatTensor
        )
        current_labels = torch.from_numpy(self.labels[idx, pt_idxs].copy()).type(
            torch.LongTensor
        )

        return current_points, current_labels

    def __len__(self):
        return int(self.points.shape[0] * self.data_precent)

    def set_num_points(self, pts):
        self.num_points = pts

    def randomize(self):
        pass


if __name__ == "__main__":
    dset = Tree3DSemSeg(16, "./", train=True)
    print(dset[0])
    print(len(dset))
    dloader = torch.utils.data.DataLoader(dset, batch_size=32, shuffle=True)
    for i, data in enumerate(dloader, 0):
        inputs, labels = data
        if i == len(dloader) - 1:
            print(inputs.size())
