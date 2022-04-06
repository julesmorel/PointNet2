if [ "$1" ]; then
  scriptsroot=$(dirname $0)
  filename=$1
  if [ -f $2 ]; then model_path=$2; else model_path=$scriptsroot/$2; fi
  dir=$(dirname "$filename")
  root=$(basename "${filename%.*}")
  minfile=$dir/${root}_min.asc
  $scriptsroot/filterMinPoints/filterMinPoints $filename $minfile 0.2
  pcafile=$dir/${root}_pca.asc
  $scriptsroot/pca/pca $minfile $pcafile 64
  echo "* point cloud enrichment : OK"
  chunkedfile=$dir/${root}_chunked.asc
  counterfile=$dir/${root}_counter.asc
  centerfile=$dir/${root}_centers.asc
  $scriptsroot/batches/batches $pcafile $chunkedfile $counterfile $centerfile 1 1024 4
  echo "* batches division : OK"
  predfile=$dir/${root}_prediction.asc
  python $scriptsroot/deepNetwork/predict.py -i $chunkedfile -o $predfile -model $model_path --use_pca
  echo "* inference : OK"
  resultfile=$dir/${root}_result.asc
  $scriptsroot/vote/aggregate $predfile $counterfile $resultfile
  echo "* vote : OK"
  rm $filterMinPoints
  rm $pcafile
  rm $chunkedfile
  rm $counterfile
  rm $centerfile
  rm $predfile
else
  echo Please provide a file to process
fi
