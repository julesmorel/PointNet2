# Parameters
RESOLUTION=0.05
R_PCA=0.05

if [ "$#" -ge  2 ]; then
  scriptsroot=$(dirname $0)
  filename=$1
  if [ -f $2 ]; then model_path=${@: -1}; else model_path=$scriptsroot/$2; fi
  dir=$(dirname "$filename")
  root=$(basename "${filename%.*}")
  filteredfile=$dir/${root}_filtered.asc
  $scriptsroot/filterListFiles/filterListFiles ${@:1:$#-1} $filteredfile 3D $RESOLUTION
  echo "* point cloud reading : OK"
  pcafile=$dir/${root}_pca.asc
  pca_radius/pca $filename $pcafile $R_PCA
  echo "* point cloud enrichment : OK"
  chunkedfile=$dir/${root}_chunked.asc
  counterfile=$dir/${root}_counter.asc
  centerfile=$dir/${root}_centers.asc
  batches_radius/batches $pcafile $chunkedfile $counterfile $centerfile 0.2 2048 4
  echo "* batches division : OK"
  predfile=$dir/${root}_prediction.asc
  python3 deepNetwork/predict.py -i ../$chunkedfile -o $predfile -model $model_path -num_points 2048 --use_pca
  echo "* inference : OK"
  resultfile=$dir/${root}_result.asc
  vote/aggregate $predfile $counterfile $resultfile
  echo "* vote : OK"
else
  echo Please provide a file to process
fi
