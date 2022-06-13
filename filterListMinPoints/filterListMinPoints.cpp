#include<iostream>
#include <fstream>
#include <string>
#include <regex>
#include <stdio.h>
#include <stdlib.h>
#include <memory>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/algorithm/string.hpp>
#include <pcl/filters/grid_minimum.h>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/Options.hpp>

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

void readLasFile(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points)
{
  pdal::Options options;
  options.add("filename", filename);
  pdal::PointTable table;
  pdal::LasReader las_reader;
  las_reader.setOptions(options);
  las_reader.prepare(table);
  pdal::PointViewSet point_view_set = las_reader.execute(table);
  pdal::PointViewPtr point_view = *point_view_set.begin();
  pdal::Dimension::IdList dims = point_view->dims();
  pdal::LasHeader las_header = las_reader.header();

  double offset_x = las_header.offsetX();
  double offset_y = las_header.offsetY();
  double offset_z = las_header.offsetZ();
  std::cout<<"Offset "<<offset_x<<" "<<offset_y<<" "<<offset_z<<std::endl;

  unsigned int n_features = las_header.pointCount();
  std::cout<<"Reading "<<n_features<<" points"<<std::endl;
  for (pdal::PointId idx = 0; idx < point_view->size(); ++idx) {
    using namespace pdal::Dimension;
    pcl::PointXYZI p;
    p.x = point_view->getFieldAs<double>(Id::X, idx);
    p.y = point_view->getFieldAs<double>(Id::Y, idx);
    p.z = point_view->getFieldAs<double>(Id::Z, idx);
    p.intensity=0.;
    points.push_back(p);
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
    std::string extension = filenames.at(i).substr(filenames.at(i).find_last_of(".") + 1);
    if(extension == "las" || extension == "laz")
    {
      readLasFile(filenames.at(i),pts);
    }
    else if(extension == "xyz" || extension == "asc")
    {
      readAsciiFile(filenames.at(i),pts);
    }else{
      std::cout<<filenames.at(i)<<" is not in the correct format. Please provide .las, .laz or ASCII files"<<std::endl;
      exit(0);
    } 

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
