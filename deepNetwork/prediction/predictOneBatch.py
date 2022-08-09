from __future__ import (
    division,
    absolute_import,
    with_statement,
    print_function,
    unicode_literals,
)
from operator import truediv
import torch
import numpy as np
import utils as utils

from pointnet2_msg_sem import Pointnet2MSG as Pointnet

def predict(points,model):

    batch_size=32

    use_pca=True
    use_intensity=False
    # Size of the batch
    num_points = len(points)
    # number of categories
    k = 2

    data_pred_center = utils.recenterPoints(points,num_points,use_pca,use_intensity)
    if use_intensity:
        data_pred_center = data_pred_center.reshape(-1, num_points, 7)
    else:
        data_pred_center = data_pred_center.reshape(-1, num_points, 6)

    if use_intensity:
        model = Pointnet(num_classes=2, input_channels=4, use_xyz=True)
    else:
        model = Pointnet(num_classes=2, input_channels=3, use_xyz=True)
    model.cuda()
    model = torch.load(model)
    model.eval()

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    pred = model(torch.from_numpy(data_pred_center).float().to(device))
    prednp = pred.cpu().detach().numpy()

    arr_id=np.zeros(batch_size);
    np.where(prednp[0,:,0]<0, arr_id, 1)

    return arr_id
