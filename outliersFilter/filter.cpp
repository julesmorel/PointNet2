#include<iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/algorithm/string.hpp>

#include <pcl/filters/statistical_outlier_removal.h>

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
      p.x=std::stod (results.at(0));
      p.y=std::stod (results.at(1));
      p.z=std::stod (results.at(2));
      points.push_back(p);
    }
    file.close();
  }
}

int main(int argc, char *argv[]){
  std::string filenameOut,filenameIn;
  int meanK = 50;
  double stddevMulThresh = 1.;

  if (argc == 5) {
    filenameIn = argv[1];
    filenameOut = argv[2];
    meanK = std::stoi (argv[3]);
    stddevMulThresh = std::stod (argv[4]);
    //std::cout<<"Processing file (wood) : "+filenameWood<<std::endl;
  }else{
    std::cout<<"Please specify a file to process"<<std::endl;
    exit(0);
  }

  pcl::PointCloud<pcl::PointXYZ> pts;
  readAsciiFile(filenameIn,pts);
  pcl::PointCloud<pcl::PointXYZ> ptsFiltered;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (pts.makeShared());
  sor.setMeanK (meanK);
  sor.setStddevMulThresh (stddevMulThresh);
  sor.filter (ptsFiltered);

std::ofstream outfile;
outfile.open(filenameOut, std::ios_base::app);

for(int i=0;i<ptsFiltered.size();i++){
    pcl::PointXYZ currentPt = ptsFiltered.at(i);
    outfile <<currentPt.x<<" "<<currentPt.y<<" "<<currentPt.z<<std::endl;
  }
}
