#include<iostream>
#include <fstream>
#include <string>
#include <regex>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/algorithm/string.hpp>
#include <pcl/filters/grid_minimum.h>

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

void readAsciiFile(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points)
{
  std::ifstream file(filename);
  if (file.is_open()) {
    std::string line;
    while (getline(file, line)) {
      std::vector<std::string> results;
      boost::split(results, line, [](char c){return c == ' ';});

      pcl::PointXYZI p;
      p.x=std::stod (results.at(0));
      p.y=std::stod (results.at(1));
      p.z=std::stod (results.at(2));
      //if the intensity is available,we store it, otherwise we just set it to 0
      if(results.size()>3){
        p.intensity=std::stod (results.at(3));
      }else{
        p.intensity=0.;
      }
      points.push_back(p);
    }
    file.close();
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
    //the one before the last must be a string
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
    std::cout<<"file_1 ... file_N output resolution"<<std::endl;
    exit(0);
  }else{
    for(int i=1;i<argc-2;i++){
      filenames.push_back(argv[i]);
    }
    filenameOut=argv[argc-2];
    res=std::stod(argv[argc-1]);
  }

  pcl::PointCloud<pcl::PointXYZI> ptsOut;
  //concatenate every minimum points
  for(int i=0;i<filenames.size();i++){
    pcl::PointCloud<pcl::PointXYZI> pts;
    readAsciiFile(filenames.at(i),pts);
    pcl::GridMinimum<pcl::PointXYZI> gm(res);
    gm.setInputCloud (pts.makeShared());
    pcl::PointCloud<pcl::PointXYZI> ptsMin;
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
