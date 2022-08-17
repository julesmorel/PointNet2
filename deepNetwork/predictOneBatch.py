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

class inference:
    def __init__(self,model=None):
        self.model = Pointnet(num_classes=2, input_channels=3, use_xyz=True)
        self.model.cuda()
        self.model = torch.load(model)
        self.model.eval()
        
    def run(self,points):  
        # Size of the batch
        num_points = len(points)
        data_pred_center = utils.recenterPoints(points,num_points,True,False)
        data_pred_center = data_pred_center.reshape(-1, num_points, 6)
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        pred = self.model(torch.from_numpy(data_pred_center).float().to(device))
        prednp = pred.cpu().detach().numpy()
        arr_p0=prednp[0,:,0]
        arr_p1=prednp[0,:,1]
        return np.concatenate([arr_p0, arr_p1])
