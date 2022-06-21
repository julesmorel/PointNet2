# Parameters
RESOLUTION=0.005
R_PCA=0.05

#Extent
X_MIN=-10
X_MAX=10
Y_MIN=-10
Y_MAX=10

#Max number of points in tile
MaxNumberPoint=500000

#Folder PDAL script
PDAL_FOLDER="pdal_scripts"

if [ "$#" -ge  2 ]; then
  scriptsroot=$(dirname $0)
  if [ -f $2 ]; then model_path=${@: -1}; else model_path=$scriptsroot/$2; fi  

  #crop and subsample slightly every input file
  echo "Cropping input files"
  for file in ${@:1:$#-1}
  do
    dir=$(dirname "$file")
    root=$(basename "${file%.*}")
    croppedFile=$dir/${root}cropped.laz
    pdal pipeline $PDAL_FOLDER/crop.json --writers.las.filename=$croppedFile --readers.las.filename=$file --filters.crop.bounds="([$X_MIN,$X_MAX],[$Y_MIN,$Y_MAX])" --filters.sample.radius="$RESOLUTION"
    listCroppedFiles+=($croppedFile)
  done

  #merge every cropped file
  echo "Merging input files"
  pointsMerged=$dir/all.laz
  pdal merge $listCroppedFiles $pointsMerged
  for file in $listCroppedFiles
  do
    rm $file
  done 

  #tile the point cloud
  echo "Tilling the extent"
  pointsTile=$dir/tile.laz
  pdal split --capacity $MaxNumberPoint $pointsMerged $pointsTile

  echo "Processing the tiles one by one ..."
  for file in $dir/tile*
  do
      if [[ -f $file ]]; then
          echo $file
          dir=$(dirname "$file")
          root=$(basename "${file%.*}")
          filteredfile=$dir/${root}_filtered.asc
          $scriptsroot/filterListFiles/filterListFiles $file $X_MIN $X_MAX $Y_MIN $Y_MAX $filteredfile 3D $RESOLUTION
          #echo "* point cloud reading : OK"
          pcafile=$dir/${root}_pca.asc
          pca_radius/pca $filteredfile $pcafile $R_PCA
          #echo "* point cloud enrichment : OK"
          chunkedfile=$dir/${root}_chunked.asc
          counterfile=$dir/${root}_counter.asc
          centerfile=$dir/${root}_centers.asc
          batches_radius/batches $pcafile $chunkedfile $counterfile $centerfile 0.25 2048 4
          #echo "* batches division : OK"
          predfile=$dir/${root}_prediction.asc
          python3 deepNetwork/predict.py -i ../$chunkedfile -o $predfile -model $model_path -num_points 2048 --use_pca
          #echo "* inference : OK"
          resultfile=$dir/${root}_result.asc
          vote/aggregate $predfile $counterfile $resultfile
          listResultFiles+=($resultfile)
          #echo "* vote : OK"
          rm $predfile
          rm $counterfile
          rm $chunkedfile
          rm $centerfile
          rm $pcafile
          rm $filteredfile
          rm $file
      fi
  done
  rm $pointsMerged
  cat $listResultFiles >> $dir/result.xyz
else
   echo Please provide a list of las/laz files and a Pytorch model
fi   