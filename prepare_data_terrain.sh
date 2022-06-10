if [ "$1" ]; then
  directory=$1
  echo Processing files in $directory
  parallel 'filterMinPoints/filterMinPoints {} {.}_min.asc 0.2' ::: $directory/*.xyz
  rm $directory/*.xyz
  echo "filter min points done"	
  parallel 'pca/pca {} {.}_pca.asc 64' ::: $directory/*.asc
  find $directory -type f ! -name '*_pca.asc' -delete
  echo "pca done"		
  parallel 'batches/batches {} {.}_chunked.asc {.}_counter.asc {.}_centers.asc 1 1024 4' ::: $directory/*.asc
  find $directory -type f ! -name '*_chunked.asc' -delete
else
  echo Please provide a working directory
fi
