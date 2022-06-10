if [ "$1" ]; then
  directory=$1
  echo Processing files in $directory
  parallel 'cloudcompare.CloudCompare -SILENT -C_EXPORT_FMT ASC -o {} -SS SPATIAL 0.005' ::: $directory/*.xyz
  rm $directory/*.xyz
  parallel 'pca_radius/pca {} {.}_pca.asc 0.05' ::: $directory/*.xyz
  find $directory -type f ! -name '*_pca.asc' -delete
  parallel 'batches_radius/batches {} {.}_chunked.asc {.}_counter.asc {.}_centers.asc 0.2 2048 4' ::: $directory/*.asc
  find $directory -type f ! -name '*_chunked.asc' -delete
else
  echo Please provide a working directory
fi
