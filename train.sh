points=1024
train_dir=train/01_128
validation_dir=validation/01_128
model_path=model_terrain_01_128
epochs=3
python deepNetwork/train.py -train ../${train_dir} -validation ../${validation_dir} -model ${model_path} -num_points ${points} -epochs ${epochs} -batch_size 16 --use_pca --visdom
