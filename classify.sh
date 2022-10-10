RESOLUTION=0.03

#Folder PDAL script
PDAL_FOLDER="pdal_scripts"

if [ "$#" -ge  3 ]; then
  scriptsroot=$(dirname $0)
  if [ -f ${@: -1} ]; then woodfile=${@: -1}; else woodfile=$scriptsroot/${@: -1}; fi
  array=( $@ )
  len=${#array[@]}
  dtm_path=${array[$len-2]}

  #crop and subsample slightly every input file
  for file in ${@:1:$#-2}
  do
    echo "**********************************************"
    echo "Classifying " $file " ..."
    dir=$(dirname "$file")
    root=$(basename "${file%.*}")
    fileClassGround=$dir/${root}_ground.laz
    pdal pipeline pdal_scripts/classify.json --writers.las.filename=$fileClassGround --readers.las.filename=$file --filters.hag_dem.raster=$dtm_path
    fileClassified=$dir/${root}_classified.laz
    ./classification $fileClassGround $woodfile $fileClassified $RESOLUTION
  done
else  
   echo "Please provide a list of las/laz files and a dtm raster"
fi