# Parameters
RESOLUTION_XY=0.2
RESOLUTION_XYZ=0.01
K_PCA=64

#Extent
X_MIN=-20
X_MAX=20
Y_MIN=-20
Y_MAX=20

#Folder PDAL script
PDAL_FOLDER="pdal_scripts"

if [ "$#" -ge  2 ]; then
  scriptsroot=$(dirname $0)
  filename=$1
  if [ -f $2 ]; then model_path=${@: -1}; else model_path=$scriptsroot/$2; fi

  #crop and subsample slightly every input file
  echo "Cropping input files"
  for file in ${@:1:$#-1}
  do
    dirfile=$(dirname "$file")
    rootfile=$(basename "${file%.*}")
    croppedFile=$dirfile/${rootfile}cropped.laz
    pdal pipeline $PDAL_FOLDER/crop.json --writers.las.filename=$croppedFile --readers.las.filename=$file --filters.crop.bounds="([$X_MIN,$X_MAX],[$Y_MIN,$Y_MAX])" --filters.sample.radius="$RESOLUTION_XYZ"
    listCroppedFiles+=("$croppedFile")
  done

  dir=$(dirname "$filename")
  root=$(basename "${filename%.*}")
  minfile=$dir/${root}_min.asc
  $scriptsroot/filterListFiles/filterListFiles ${listCroppedFiles[@]} $minfile $RESOLUTION_XY
  pcafile=$dir/${root}_pca.asc
  $scriptsroot/pca/pca $minfile $pcafile $K_PCA
  echo "* point cloud enrichment : OK"
  chunkedfile=$dir/${root}_chunked.asc
  counterfile=$dir/${root}_counter.asc
  centerfile=$dir/${root}_centers.asc
  $scriptsroot/batches/batches $pcafile $chunkedfile $counterfile $centerfile 1 1024 4
  echo "* batches division : OK"
  predfile=$dir/${root}_prediction.asc
  python $scriptsroot/deepNetwork/predict.py -i ../$chunkedfile -o $predfile -model $model_path --use_pca
  echo "* inference : OK"
  resultfile=$dir/min_points_pointnet2.asc
  $scriptsroot/vote/aggregate $predfile $counterfile $resultfile
  echo "* vote : OK"
  rm $minfile
  rm $pcafile
  rm $chunkedfile
  rm $counterfile
  rm $centerfile
  rm $predfile
else
  echo "Please provide at least a file to process and a model"
fi
