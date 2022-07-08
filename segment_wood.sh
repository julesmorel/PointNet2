# Parameters
RESOLUTION=0.005
R_PCA=0.05

#Extent
X_MIN=-10
X_MAX=10
Y_MIN=-10
Y_MAX=10

#Size of the slice in Z (in order to remove the ground points)
ZMIN=1
ZMAX=25

#Max number of points in tile
MaxNumberPoint=500000

#Folder PDAL script
PDAL_FOLDER="pdal_scripts"

if [ "$#" -ge  3 ]; then
  scriptsroot=$(dirname $0)
  if [ -f ${@: -1} ]; then model_path=${@: -1}; else model_path=$scriptsroot/${@: -1}; fi  

  array=( $@ )
  len=${#array[@]}
  dtm_path=${array[$len-2]}

  #crop and subsample slightly every input file
  for file in ${@:1:$#-2}
  do
    echo "**********************************************"
    echo "Processing " $file " ..."
    dir=$(dirname "$file")
    root=$(basename "${file%.*}")
    croppedFile=$dir/${root}cropped.laz
    pdal pipeline $PDAL_FOLDER/crop.json --writers.las.filename=$croppedFile --readers.las.filename=$file --filters.crop.bounds="([$X_MIN,$X_MAX],[$Y_MIN,$Y_MAX])" --filters.sample.radius="$RESOLUTION"
    
    croppedFileNoGround=$dir/${root}NoGround.laz
    pdal pipeline $PDAL_FOLDER/filter_hag.json --writers.las.filename=$croppedFileNoGround --readers.las.filename=$croppedFile --filters.dem.raster=$dtm_path --filters.dem.limits="Z[-$ZMIN:$ZMAX]"
    rm $croppedFile

    pointsTile=$dir/tile.laz
    pdal split --capacity $MaxNumberPoint $croppedFileNoGround $pointsTile
    rm $croppedFileNoGround

    for file in $dir/tile*
    do
        if [[ -f $file ]]; then
            echo $file
            dir=$(dirname "$file")
            root=$(basename "${file%.*}")
            filteredfile=$dir/${root}_filtered.asc
            $scriptsroot/las2pcl/las2pcl $file $filteredfile
            pcafile=$dir/${root}_pca.asc
            $scriptsroot/pca_radius/pca $filteredfile $pcafile $R_PCA
            chunkedfile=$dir/${root}_chunked.asc
            counterfile=$dir/${root}_counter.asc
            centerfile=$dir/${root}_centers.asc
            $scriptsroot/batches_radius/batches $pcafile $chunkedfile $counterfile $centerfile 0.2 2048 4
            predfile=$dir/${root}_prediction.asc
            python3 $scriptsroot/deepNetwork/predict.py -i ../$chunkedfile -o $predfile -model $model_path -num_points 2048 --use_pca
            resultfile=$dir/${root}_result.asc
            $scriptsroot/vote/aggregate $predfile $counterfile $resultfile
            listResultFiles+=($resultfile)       
            rm $predfile
            rm $counterfile
            rm $chunkedfile
            rm $centerfile
            rm $pcafile
            rm $filteredfile
            rm $file
        fi
    done

    resultFile=$dir/${root}result.xyz   
    for file in ${listResultFiles[@]}
    do
      cat $file >> $resultFile   
      rm $file
    done 
    listCroppedFiles+=("$resultFile")

  done 

  pointsMerged=$dir/result.xyz
  $scriptsroot/merge/merge ${listCroppedFiles[@]} $pointsMerged
  for file in ${listCroppedFiles[@]}
  do
    rm $file 
  done  

else
   echo "Please provide a list of las/laz files, a dtm raster and a Pytorch model"
fi   