#include<iostream>
#include <fstream>
#include <string>
#include <regex>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include "../util/pointCloudFileReader.h"
#include "../util/offsetManager.h"

//check if a string represent a number
bool isNumber(std::string x){
  std::regex e ("^-?\\d*\\.?\\d+");
  if (std::regex_match (x,e)) return true;
  else return false;
}

bool fileExists (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

int main(int argc, char *argv[]){
  std::vector<std::string> filenames;
  std::string filenameOut;
  bool argsOk=true;

  //we need at least 3 args (+1 as the program name counts)
  if (argc > 2) {
    //the first args must be existing files
    //then comes the output file name
    for(int i=1;i<argc-1;i++){
      argsOk=argsOk && fileExists(argv[i]);
    }
    argsOk=argsOk && !isNumber(argv[argc-1]);
  }else{
    argsOk=false;
  }

  //if the args are not in the correct fomat, we just stop here
  //otherwise we store the args and carry on
  if(!argsOk){
    std::cout<<"Please specify arguments in the following order:"<<std::endl;
    std::cout<<"file_1 ... file_N output_file"<<std::endl;
    exit(0);
  }else{
    for(int i=1;i<argc-1;i++){
      filenames.push_back(argv[i]);
    }
    filenameOut=argv[argc-1];
  }

  pcl::PointCloud<pcl::PointXYZI> ptsOut;
  //concatenate every point clouds provided
  for(int i=0;i<filenames.size();i++){
    
    //read the offset for the current file if it exists
    offsetManager offsetM(filenames.at(i));
    double offset_x = offsetM.getOffsetX();
    double offset_y = offsetM.getOffsetY();

    //read the points in the file 
    pcl::PointCloud<pcl::PointXYZI> pts;   
    pointCloudFileReader::read(filenames.at(i),pts,offset_x,offset_y);

    //store of the offset if it was not stored before
    offsetM.setOffset(offset_x,offset_y);

    //concatenate the points  
    ptsOut+=pts;
  }

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (ptsOut.makeShared());

  double cellSize=0.05;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud (ptsOut.makeShared());
  voxel_grid.setLeafSize (cellSize, cellSize, cellSize);
  pcl::PointCloud<pcl::PointXYZI> ptsFiltered;
  voxel_grid.filter(ptsFiltered);

  std::ofstream outfile;
  outfile.open(filenameOut, std::ios_base::app);
  for(int i=0;i<ptsFiltered.size();i++){
    pcl::PointXYZI currentPt = ptsFiltered.at(i);
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    int numberN =  kdtree.radiusSearch(currentPt,cellSize,k_indices,k_sqr_distances);
    if(numberN>0){
      double score=0;
      for( int j = 0; j< k_indices.size() ;j++){
        pcl::PointXYZI p = ptsOut.at(k_indices.at(j));
        score+=p.intensity/((double)numberN);
      }
      outfile <<currentPt.x<<" "<<currentPt.y<<" "<<currentPt.z<<" "<<score<<'\n';
    }
  }
}