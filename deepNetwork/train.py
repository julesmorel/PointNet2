from __future__ import (
    division,
    absolute_import,
    with_statement,
    print_function,
    unicode_literals,
)
import torch.optim as optim
import torch.optim.lr_scheduler as lr_sched
import torch.nn as nn
from torch.utils.data import DataLoader
import pytorch_utils as pt_utils
import pprint
import os.path as osp
import os
import argparse
import torch

from pointnet2_msg_sem import Pointnet2MSG as Pointnet
from pointnet2_msg_sem import model_fn_decorator
from Tree3DSegLoader import Tree3DSemSeg

parser = argparse.ArgumentParser(description="Arg parser")
parser.add_argument(
    "-batch_size", type=int, default=16, help="Batch size [default: 16]"
)
parser.add_argument(
    "-num_points",
    type=int,
    default=2048,
    help="Number of points to train with [default: 2048]",
)
parser.add_argument(
    "-weight_decay",
    type=float,
    default=0,
    help="L2 regularization coeff [default: 0.0]",
)
parser.add_argument(
    "-lr", type=float, default=0.001, help="Initial learning rate [default: 0.001]"
)
parser.add_argument(
    "-lr_decay",
    type=float,
    default=0.5,
    help="Learning rate decay gamma [default: 0.5]",
)
parser.add_argument(
    "-decay_step",
    type=float,
    default=200000,
    help="Learning rate decay step [default: 20]",
)
parser.add_argument(
    "-bn_momentum",
    type=float,
    default=0.9,
    help="Initial batch norm momentum [default: 0.9]",
)
parser.add_argument(
    "-bn_decay",
    type=float,
    default=0.5,
    help="Batch norm momentum decay gamma [default: 0.5]",
)
parser.add_argument(
    "-checkpoint", type=str, default=None, help="Checkpoint to start from"
)
parser.add_argument(
    "-epochs", type=int, default=150, help="Number of epochs to train for"
)
parser.add_argument(
    "-run_name",
    type=str,
    default="sem_seg_run_1",
    help="Name for run in tensorboard_logger",
)
parser.add_argument(
    "-train",
    type=str,
    default="train",
    help="Path to the directory containing the training data",
)
parser.add_argument(
    "-validation",
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
    "-w_wood",
    type=float,
    default=1,
    help="Wood class weight [default: 1]",
)
parser.add_argument(
    "-w_leaf",
    type=float,
    default=1,
    help="Leaf class weight [default: 1]",
)
parser.add_argument("--visdom-port", type=int, default=8097)
parser.add_argument("--visdom", action="store_true")
parser.add_argument("--use_pca", action="store_true")
parser.add_argument("--use_intensity", action="store_true")

lr_clip = 1e-5
bnm_clip = 0.99

if __name__ == "__main__":
    args = parser.parse_args()

    use_pca = False


    test_set = Tree3DSemSeg(args.num_points, args.validation,args.use_pca,args.use_intensity)
    test_loader = DataLoader(
        test_set,
        batch_size=args.batch_size,
        shuffle=True,
        pin_memory=True,
        num_workers=2,
    )

    train_set = Tree3DSemSeg(args.num_points, args.train,args.use_pca,args.use_intensity)
    train_loader = DataLoader(
        train_set,
        batch_size=args.batch_size,
        pin_memory=True,
        num_workers=2,
        shuffle=True,
    )

    if args.use_pca:
        if args.use_intensity:
            model = Pointnet(num_classes=2, input_channels=4, use_xyz=True)
        else:
            model = Pointnet(num_classes=2, input_channels=3, use_xyz=True)
        #model = Pointnet(num_classes=2, input_channels=6, use_xyz=True)
    else:
        model = Pointnet(num_classes=2, input_channels=0, use_xyz=True)
    model.cuda()
    optimizer = optim.Adam(
        model.parameters(), lr=args.lr, weight_decay=args.weight_decay
    )

    lr_lbmd = lambda it: max(
        args.lr_decay ** (int(it * args.batch_size / args.decay_step)),
        lr_clip / args.lr,
    )
    bnm_lmbd = lambda it: max(
        args.bn_momentum
        * args.bn_decay ** (int(it * args.batch_size / args.decay_step)),
        bnm_clip,
    )

    # default value
    it = -1  # for the initialize value of `LambdaLR` and `BNMomentumScheduler`
    best_loss = 1e10
    start_epoch = 1

    # load status from checkpoint
    if args.checkpoint is not None:
        checkpoint_status = pt_utils.load_checkpoint(
            model, optimizer, filename=args.checkpoint.split(".")[0]
        )
        if checkpoint_status is not None:
            it, start_epoch, best_loss = checkpoint_status

    lr_scheduler = lr_sched.LambdaLR(optimizer, lr_lambda=lr_lbmd, last_epoch=it)
    bnm_scheduler = pt_utils.BNMomentumScheduler(
        model, bn_lambda=bnm_lmbd, last_epoch=it
    )

    it = max(it, 0)  # for the initialize value of `trainer.train`

    weights = [args.w_wood, args.w_leaf]
    class_weights = torch.FloatTensor(weights).cuda()
    model_fn = model_fn_decorator(nn.CrossEntropyLoss(weight=class_weights))

    if args.visdom:
        viz = pt_utils.VisdomViz(port=args.visdom_port)
    else:
        viz = pt_utils.CmdLineViz()

    viz.text(pprint.pformat(vars(args)))

    if not osp.isdir("checkpoints"):
        os.makedirs("checkpoints")

    trainer = pt_utils.Trainer(
        model,
        model_fn,
        optimizer,
        checkpoint_name="checkpoints/model_smeseg",
        best_name="checkpoints/model_smeseg_best",
        lr_scheduler=lr_scheduler,
        bnm_scheduler=bnm_scheduler,
        viz=viz,
    )

    trainer.train(
        it, start_epoch, args.epochs, train_loader, test_loader, best_loss=best_loss
    )

    torch.save(model, args.model)

    if start_epoch == args.epochs:
        _ = trainer.eval_epoch(test_loader)
