#include<iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/algorithm/string.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <sys/stat.h>

void readAsciiFile(std::string filename, pcl::PointCloud<pcl::PointXYZ>& points)
{
  std::ifstream file(filename);
  if (file.is_open()) {
    std::string line;
    while (getline(file, line)) {
      std::vector<std::string> results;
      boost::split(results, line, [](char c){return c == ' ';});

      pcl::PointXYZ p;
      try {
      p.x=std::stod (results.at(0));
      p.y=std::stod (results.at(1));
      p.z=std::stod (results.at(2));
      points.push_back(p);
      } catch (const std::exception& e) {  }
    }
    file.close();
  }
}

int main(int argc, char *argv[]){
  std::string filenameOut,filenameIn;
  int meanK = 50;
  double stddevMulThresh = 1.;
  double radiusSearch=0.;
  int MinNeighborsInRadius=10;

  if (argc == 7) {
    filenameIn = argv[1];
    filenameOut = argv[2];
    meanK = std::stoi (argv[3]);
    stddevMulThresh = std::stod (argv[4]);
    radiusSearch = std::stod (argv[5]);
    MinNeighborsInRadius = std::stoi (argv[6]);
  }else{
    std::cout<<"Please specify a file to process"<<std::endl;
    exit(0);
  }

  pcl::PointCloud<pcl::PointXYZ> pts;
  readAsciiFile(filenameIn,pts);

  std::cout<<"File read:"<<pts.size()<<std::endl;

  pcl::PointCloud<pcl::PointXYZ> ptsFiltered;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (pts.makeShared());
  sor.setMeanK (meanK);
  sor.setStddevMulThresh (stddevMulThresh);
  sor.filter (ptsFiltered);

  std::cout<<pts.size()-ptsFiltered.size()<<" points filtered (StatisticalOutlierRemoval)"<<std::endl;

  pcl::PointCloud<pcl::PointXYZ> ptsFilteredRadius;
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(ptsFiltered.makeShared());
  outrem.setRadiusSearch(radiusSearch);
  outrem.setMinNeighborsInRadius(MinNeighborsInRadius);
  outrem.filter (ptsFilteredRadius);

  std::cout<<ptsFiltered.size()-ptsFilteredRadius.size()<<" points filtered (RadiusOutlierRemoval)"<<std::endl;

  std::ofstream outfile;
  outfile.open(filenameOut, std::ios_base::app);

  for(int i=0;i<ptsFilteredRadius.size();i++){
      pcl::PointXYZ currentPt = ptsFilteredRadius.at(i);
      outfile <<currentPt.x<<" "<<currentPt.y<<" "<<currentPt.z<<'\n';
  }
}
