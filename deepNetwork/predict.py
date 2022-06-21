from __future__ import (
    division,
    absolute_import,
    with_statement,
    print_function,
    unicode_literals,
)
import argparse
import torch
import sys
from tqdm import tqdm
import numpy as np
import utils as utils
import argparse

from pointnet2_msg_sem import Pointnet2MSG as Pointnet

parser = argparse.ArgumentParser(description="Arg parser")
parser.add_argument(
    "-batch_size", type=int, default=32, help="Batch size [default: 32]"
)
parser.add_argument(
    "-i",
    type=str,
    default="test",
    help="Path to the directory containing the validation data",
)
parser.add_argument(
    "-o",
    type=str,
    default="test",
    help="Path to the directory containing the validation data",
)
parser.add_argument(
    "-model",
    type=str,
    default="model",
    help="Path to the model",
)
parser.add_argument(
    "-num_points",
    type=int,
    default=1024,
    help="Number of points in the batch",
)
parser.add_argument("--use_pca", action="store_true")
parser.add_argument("--use_intensity", action="store_true")

args = parser.parse_args()
# Size of the batch
num_points = args.num_points
# number of categories
k = 2

data_pred, labels_training = utils.loadPointsAndLabels(args.i,num_points,args.use_pca,args.use_intensity)
data_pred2 = data_pred.copy()
data_pred_center = utils.recenterPoints(data_pred,num_points,args.use_pca,args.use_intensity)
data_pred_center2 = data_pred_center.copy()
if args.use_intensity:
    data_pred_center = data_pred_center.reshape(-1, num_points, 7)
else:
    data_pred_center = data_pred_center.reshape(-1, num_points, 6)

if args.use_intensity:
    model = Pointnet(num_classes=2, input_channels=4, use_xyz=True)
else:
    model = Pointnet(num_classes=2, input_channels=3, use_xyz=True)
model.cuda()
model = torch.load(args.model)
model.eval()

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

sh = data_pred_center.shape
li = sh[0]
for i in tqdm(range(li)):

    v_points = data_pred_center[i:i+1,:,:]
    pred = model(torch.from_numpy(v_points).float().to(device))
    prednp = pred.cpu().detach().numpy()
    v_points = np.squeeze(v_points)

    arr_xs=[];
    arr_ys=[];
    arr_zs=[];
    arr_p1=[];
    arr_p2=[];

    batch_size=v_points.shape[0];
    arr_xs=data_pred2[i*num_points:i*num_points+batch_size,0]
    arr_ys=data_pred2[i*num_points:i*num_points+batch_size,1]
    arr_zs=data_pred2[i*num_points:i*num_points+batch_size,2]

    arr_id=np.zeros(batch_size);
    np.where(prednp[0,:,0]<0, arr_id, 1)

    arr_p1=prednp[0,:,0]
    arr_p2=prednp[0,:,1]

    data = np.column_stack((np.array(arr_xs), np.array(arr_ys),np.array(arr_zs),arr_id,np.array(arr_p1),np.array(arr_p2)))
    with open(args.o, "ab") as f:
        np.savetxt(f,data, fmt=['%f','%f','%f','%i','%f','%f'])
