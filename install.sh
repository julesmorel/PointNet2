echo "Building batches ..."
cd batches
cmake . >/dev/null
make
cd ..
echo "Building batches radius..."
cd batches_radius
cmake . >/dev/null
make
cd ..
echo "Building filter min ..."
cd filterMinPoints
cmake . >/dev/null
make
cd ..
echo "Building filterListFiles ..."
cd filterListFiles
cmake . >/dev/null
make
cd ..
echo "Building pca ..."
cd pca
cmake . >/dev/null
make
cd ..
echo "Building pca_radius ..."
cd pca_radius
cmake . >/dev/null
make
cd ..
echo "Building vote ..."
cd vote
cmake . >/dev/null
make
cd ..
echo "Building filter ..."
cd outliersFilter
cmake . >/dev/null
make
cd ..
echo "Building clustering ..."
cd clustering
cmake . >/dev/null
make
cd ..
echo "Installing the deep network and dependencies ..."
cd deepNetwork
python setup.py build_ext --inplace
cd ..
