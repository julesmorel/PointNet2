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

#include "../util/pointCloudFileReader.h"

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
  Extent limits;
  std::string filteringType;
  float res=0.1;
  bool argsOk=true;

  //we need at least 3 args (+1 as the program name counts)
  if (argc > 8) {
    //the first args must be existing files
    //the comes the output file name
    //the one before the last must be "2D" or "3D"
    //the last one must be a double
    for(int i=1;i<argc-7;i++){
      argsOk=argsOk && fileExists(argv[i]);
    }
    argsOk=argsOk && isNumber(argv[argc-7]);
    argsOk=argsOk && isNumber(argv[argc-6]);
    argsOk=argsOk && isNumber(argv[argc-5]);
    argsOk=argsOk && isNumber(argv[argc-4]);
    argsOk=argsOk && !isNumber(argv[argc-3]);
    std::string tmp=argv[argc-2];
    argsOk=argsOk && (tmp=="2D" || tmp=="3D");
    argsOk=argsOk && isNumber(argv[argc-1]);
  }else{
    argsOk=false;
  }

  //if the args are not in the correct fomat, we just stop here
  //otherwise we store the args and carry on
  if(!argsOk){
    std::cout<<"Please specify arguments in the following order:"<<std::endl;
    std::cout<<"file_1 ... file_N xmin xmax ymin ymax output_file 2D(or 3D) resolution"<<std::endl;
    exit(0);
  }else{
    for(int i=1;i<argc-7;i++){
      filenames.push_back(argv[i]);
    }
    limits.xMin=std::stod(argv[argc-7]);
    limits.xMax=std::stod(argv[argc-6]);
    limits.yMin=std::stod(argv[argc-5]);
    limits.yMax=std::stod(argv[argc-4]);
    filenameOut=argv[argc-3];
    filteringType=argv[argc-2];
    res=std::stod(argv[argc-1]);
  }

  //store the offset in X and Y if we deql with georeferenced las file
  double offset_x=0.;
  double offset_y=0.;

  pcl::PointCloud<pcl::PointXYZI> ptsOut;
  //concatenate every point clouds provided
  for(int i=0;i<filenames.size();i++){
    pcl::PointCloud<pcl::PointXYZI> pts;    
    pcl::PointCloud<pcl::PointXYZI> ptsMin;
    if(filteringType=="2D"){
      pointCloudFileReader::read(filenames.at(i),pts,offset_x,offset_y,limits);
      pcl::GridMinimum<pcl::PointXYZI> gm(res);
      gm.setInputCloud (pts.makeShared());
      gm.filter(ptsMin);
    }else if(filteringType=="3D"){
      pointCloudFileReader::read(filenames.at(i),pts,offset_x,offset_y,limits,res);
      ptsMin=pts;
    }
    ptsOut+=ptsMin;
  }

  //if we process las/laz files, we save the offset
  if(offset_x!=0. || offset_y!=0.){
    boost::filesystem::path localFolder(filenameOut);
    std::string offsetFileName = localFolder.parent_path().string() + boost::filesystem::path::preferred_separator + "offset.txt";
    std::cout<<"Saving offset in "<<offsetFileName<<std::endl;
    std::ofstream outfile;
    outfile.open(offsetFileName, std::ios_base::app);
    outfile<<offset_x<<" "<<offset_y<<std::endl;
    outfile.close();
  }

  //get the minimum points of the concatenation
  pcl::PointCloud<pcl::PointXYZI> ptsOutMin;
  if(filteringType=="2D"){
      pcl::GridMinimum<pcl::PointXYZI> gm(res);
      gm.setInputCloud (ptsOut.makeShared());
      gm.filter(ptsOutMin);
  }else if(filteringType=="3D"){
      ptsOutMin=ptsOut;
  }

  //dump the points in the output file
  std::ofstream outfile;
  outfile.open(filenameOut, std::ios_base::app);
  for(int i=0;i<ptsOutMin.size();i++){
    pcl::PointXYZI p = ptsOutMin.at(i);
    outfile <<p.x<<" "<<p.y<<" "<<p.z<<" "<<(int)p.intensity<<std::endl;
  }
}
