# Parameters
RESOLUTION=0.005
R_PCA=0.05

#Extent
X_MIN=-20
X_MAX=20
Y_MIN=-20
Y_MAX=20

#Size of the slice in Z (in order to remove the ground points)
ZMIN=0.2
ZMAX=25

#Max number of points in tile
MaxNumberPoint=500000

#Resolution of the final point cloud 
#Warning: if too small, PCL gives: Leaf size is too small for the input dataset. Integer indices would overflow
RESOLUTION_OUT=0.03

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

    #empty the array
    listResultFiles=()

    for tile in $dir/tile*
    do
        if [[ -f $tile ]]; then
            echo "   " $tile
            dir_tile=$(dirname "$tile")
            root_tile=$(basename "${tile%.*}")
            chunkedfile=$dir_tile/${root_tile}_chunked.asc
            counterfile=$dir_tile/${root_tile}_counter.asc
            $scriptsroot/prediction/prediction $tile $chunkedfile $counterfile $R_PCA 0.3 2048

            #filteredfile=$dir_tile/${root_tile}_filtered.asc
            #$scriptsroot/las2pcl/las2pcl $tile $filteredfile
            #pcafile=$dir_tile/${root_tile}_pca.asc
            #$scriptsroot/pca_radius/pca $filteredfile $pcafile $R_PCA
            #chunkedfile=$dir_tile/${root_tile}_chunked.asc
            #counterfile=$dir_tile/${root_tile}_counter.asc
            #centerfile=$dir_tile/${root_tile}_centers.asc
            #$scriptsroot/batches_radius/batches $pcafile $chunkedfile $counterfile $centerfile 0.2 2048 4
            
            predfile=$dir_tile/${root_tile}_prediction.asc
            python3 $scriptsroot/deepNetwork/predict.py -i ../$chunkedfile -o $predfile -model $model_path -num_points 2048 --use_pca
            resultfile=$dir_tile/${root_tile}_result.asc
            $scriptsroot/vote/aggregate $predfile $counterfile $resultfile
            listResultFiles+=($resultfile)
            rm $predfile
            rm $counterfile
            rm $chunkedfile
            #rm $centerfile
            #rm $pcafile
            #rm $filteredfile
            rm $tile
        fi
    done

    resultFileMerge=$dir/${root}result.xyz
    for fileResult in ${listResultFiles[@]}
    do
      cat $fileResult >> $resultFileMerge
      rm $fileResult
    done
    echo "   Append results in " $resultFileMerge
    listResultFilesMerged+=("$resultFileMerge")

  done

  pointsMerged=$dir/result.xyz
  $scriptsroot/merge/merge ${listResultFilesMerged[@]} $pointsMerged $RESOLUTION_OUT
  for f in ${listResultFilesMerged[@]}
  do
    rm $f
  done
else
   echo "Please provide a list of las/laz files, a dtm raster and a Pytorch model"
fi
