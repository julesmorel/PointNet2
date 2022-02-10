#include<iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/algorithm/string.hpp>
#include <pcl/filters/grid_minimum.h>

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
      p.intensity=std::stod (results.at(3));
      points.push_back(p);
    }
    file.close();
  }
}

int main(int argc, char *argv[]){
  std::string filename;
  std::string filenameOut;
  float res=0.1;

  if (argc == 4) {
    filename = argv[1];
    filenameOut = argv[2];
    res=std::stod(argv[3]);
  }else{
    std::cout<<"Please specify a file to process"<<std::endl;
    exit(0);
  }

  pcl::PointCloud<pcl::PointXYZI> pts;
  readAsciiFile(filename,pts);

  pcl::GridMinimum<pcl::PointXYZI> gm(res);
  gm.setInputCloud (pts.makeShared());
  pcl::PointCloud<pcl::PointXYZI> ptsOut;
  gm.filter (ptsOut);

  std::ofstream outfile;
  outfile.open(filenameOut, std::ios_base::app);
  for(int i=0;i<ptsOut.size();i++){
    pcl::PointXYZI p = ptsOut.at(i);
    outfile <<p.x<<" "<<p.y<<" "<<p.z<<" "<<(int)p.intensity<<std::endl;
  }
}
