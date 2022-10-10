#Extent
X_MIN=-20
X_MAX=20
Y_MIN=-20
Y_MAX=20

#Limit in Z for the ground classification
LIMIT_Z=0.1

#Resolution for the classification of wood points
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
    pdal pipeline pdal_scripts/classify.json --writers.las.filename=$fileClassGround --readers.las.filename=$file --filters.hag_dem.raster=$dtm_path --filters.crop.bounds="([$X_MIN,$X_MAX],[$Y_MIN,$Y_MAX])"
    fileClassified=$dir/${root}_classified.laz
    ./classification/classification $fileClassGround $woodfile $fileClassified $RESOLUTION
    rm $fileClassGround
  done
else  
   echo "Please provide a list of las/laz files, a dtm raster and a ASCII file containing every trees points"
fi