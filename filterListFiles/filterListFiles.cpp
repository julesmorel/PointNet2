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
  float res=0.1;
  bool argsOk=true;

  //we need at least 3 args (+1 as the program name counts)
  if (argc > 3) {
    //the first args must be existing files
    //the comes the output file name
    //the last one must be a double
    for(int i=1;i<argc-2;i++){
      argsOk=argsOk && fileExists(argv[i]);
    }
    argsOk=argsOk && !isNumber(argv[argc-2]);
    argsOk=argsOk && isNumber(argv[argc-1]);
  }else{
    argsOk=false;
  }

  //if the args are not in the correct fomat, we just stop here
  //otherwise we store the args and carry on
  if(!argsOk){
    std::cout<<"Please specify arguments in the following order:"<<std::endl;
    std::cout<<"file_1 ... file_N output_file resolution"<<std::endl;
    exit(0);
  }else{
    for(int i=1;i<argc-2;i++){
      filenames.push_back(argv[i]);
    }

    filenameOut=argv[argc-2]; 
    res=std::stod(argv[argc-1]);
  }

  //store the offset in X and Y if we deal with georeferenced las file
  double offset_x=0.;
  double offset_y=0.;
  //check if the offset file already exist and read it
  boost::filesystem::path localFolder(filenameOut);
  std::string offsetFileName = localFolder.parent_path().string() + boost::filesystem::path::preferred_separator + "offset.txt";
  if(std::ifstream(offsetFileName)){
    //std::cout<<"Reading offset in "<<offsetFileName<<std::endl;
    std::ifstream file(offsetFileName);
    if (file.is_open()) {
      std::string line;
      getline(file, line);
      std::vector<std::string> results;
      boost::split(results, line, [](char c){return c == ' ';});
      offset_x=std::stod (results.at(0));
      offset_y=std::stod (results.at(1));
    }
  }

  pcl::PointCloud<pcl::PointXYZI> ptsOut;
  //concatenate every point clouds provided
  for(int i=0;i<filenames.size();i++){
    //read the offset for the current file if it exists
    offsetManager offsetM(filenames.at(i));
    double offset_x = offsetM.getOffsetX();
    double offset_y = offsetM.getOffsetY();
    //read the points in the file and keep the minimum points per cell
    pcl::PointCloud<pcl::PointXYZI> pts;    
    pcl::PointCloud<pcl::PointXYZI> ptsMin;
    pointCloudFileReader::read(filenames.at(i),pts,offset_x,offset_y);
    //store of the offset if it was not stored before
    offsetM.setOffset(offset_x,offset_y);
    pcl::GridMinimum<pcl::PointXYZI> gm(res);
    gm.setInputCloud (pts.makeShared());
    gm.filter(ptsMin);
    ptsOut+=ptsMin;
  }

  //get the minimum points of the concatenation
  pcl::PointCloud<pcl::PointXYZI> ptsOutMin;
  pcl::GridMinimum<pcl::PointXYZI> gm(res);
  gm.setInputCloud (ptsOut.makeShared());
  gm.filter(ptsOutMin);

  //dump the points in the output file
  std::ofstream outfile;
  outfile.open(filenameOut, std::ios_base::app);
  for(int i=0;i<ptsOutMin.size();i++){
    pcl::PointXYZI p = ptsOutMin.at(i);
    outfile <<p.x<<" "<<p.y<<" "<<p.z<<" "<<(int)p.intensity<<std::endl;
  }
}
