if [ "$1" ]; then
  filename=$1
  model_path=$2
  dir=$(dirname "$filename")
  root=$(basename "${filename%.*}")
  pcafile=$dir/${root}_pca.asc
  pca_radius/pca $filename $pcafile 0.1
  echo "* point cloud enrichment : OK"
  chunkedfile=$dir/${root}_chunked.asc
  counterfile=$dir/${root}_counter.asc
  centerfile=$dir/${root}_centers.asc
  batches_radius/batches $pcafile $chunkedfile $counterfile $centerfile 0.25 2048 4
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
