#include<iostream>
#include <fstream>
#include <string>
#include <regex>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/grid_minimum.h>
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

  //dump the points in the output file
  std::ofstream outfile;
  outfile.open(filenameOut, std::ios_base::app);
  for(int i=0;i<ptsOut.size();i++){
    pcl::PointXYZI p = ptsOut.at(i);
    outfile <<p.x<<" "<<p.y<<" "<<p.z<<" "<<(int)p.intensity<<'\n';
  }
}