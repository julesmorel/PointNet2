echo "Building batches ..."
cd batches
cmake . >/dev/null
make
cd ..
echo "Building filter min ..."
cd filterMinPoints
cmake . >/dev/null
make
cd ..
echo "Building pca ..."
cd pca
cmake . >/dev/null
make
cd ..
echo "Building vote ..."
cd vote
cmake . >/dev/null
make
cd ..
echo "Installing the deep network and dependencies ..."
cd deepNetwork
python setup.py build_ext --inplace
cd ..
