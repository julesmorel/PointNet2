#include "pointCloudFileReader.h"

#include <memory>

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/Options.hpp>

#include <boost/algorithm/string.hpp>

void pointCloudFileReader::read(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points, double& offset_x, double& offset_y)
{    
    //std::cout<<"Reading "<<filename<<std::endl;    
    std::string extension = filename.substr(filename.find_last_of(".") + 1);   
    if(extension == "las" || extension == "laz")
    {
      readLasFile(filename,points,offset_x,offset_y);
    }
    else if(extension == "xyz" || extension == "asc")
    {
      readAsciiFile(filename,points);
    }else{
      std::cout<<filename<<" is not in the correct format. Please provide .las, .laz or ASCII files"<<std::endl;
    } 
}

void pointCloudFileReader::read(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points)
{    
    //std::cout<<"Reading "<<filename<<std::endl;    
    std::string extension = filename.substr(filename.find_last_of(".") + 1);   
    if(extension == "las" || extension == "laz")
    {
      readLasFile(filename,points);
    }
    else if(extension == "xyz" || extension == "asc")
    {
      readAsciiFile(filename,points);
    }else{
      std::cout<<filename<<" is not in the correct format. Please provide .las, .laz or ASCII files"<<std::endl;
    } 
}

void pointCloudFileReader::readAsciiFile(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points)
{
  pcl::PointCloud<pcl::PointXYZI> pts;
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

void pointCloudFileReader::readLasFile(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points, double& offset_x, double& offset_y)
{
  //define the input file
  pdal::Options options;
  options.add("filename", filename);
  pdal::PointTable table;
  pdal::LasReader las_reader;
  las_reader.setOptions(options);

  las_reader.prepare(table);
  pdal::PointViewSet point_view_set = las_reader.execute(table);
  
  //read the list of points
  pdal::PointViewPtr point_view = *point_view_set.begin();
  pdal::Dimension::IdList dims = point_view->dims();
  pdal::LasHeader las_header = las_reader.header();

  std::cout.precision(12);
  //we consider the offset as the center of the first plot we process
  if(offset_x==0. && offset_y==0.){
    offset_x=0.5*las_header.minX()+0.5*las_header.maxX();
    offset_y=0.5*las_header.minY()+0.5*las_header.maxY();
    if(offset_x!=0. || offset_y!=0.){
      std::cout<<"Setting offset to : "<<offset_x<<" "<<offset_y<<std::endl;
    }  
  }
  
  //because PCL encodes the points coordinates in float, we need to translate the point by a (-offset_x,-offset_y) vector
  for (pdal::PointId idx = 0; idx < point_view->size(); ++idx) {  
    using namespace pdal::Dimension;
    pcl::PointXYZI p;
    double fieldX=point_view->getFieldAs<double>(Id::X, idx);
    double fieldY=point_view->getFieldAs<double>(Id::Y, idx);
    double fieldZ=point_view->getFieldAs<double>(Id::Z, idx);
    double classification=point_view->getFieldAs<double>(Id::Classification, idx);
    double x = fieldX-offset_x;
    double y = fieldY-offset_y;
    double z = fieldZ;
    p.x=x;
    p.y=y;
    p.z=z;
    p.intensity=classification;       
    points.push_back(p);  
  }
}

void pointCloudFileReader::readLasFile(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points)
{
  //define the input file
  pdal::Options options;
  options.add("filename", filename);
  pdal::PointTable table;
  pdal::LasReader las_reader;
  las_reader.setOptions(options);

  las_reader.prepare(table);
  pdal::PointViewSet point_view_set = las_reader.execute(table);
  
  //read the list of points
  pdal::PointViewPtr point_view = *point_view_set.begin();
  pdal::Dimension::IdList dims = point_view->dims();
  pdal::LasHeader las_header = las_reader.header();

  std::cout.precision(12);
  
  for (pdal::PointId idx = 0; idx < point_view->size(); ++idx) {  
    using namespace pdal::Dimension;
    pcl::PointXYZI p;
    double fieldX=point_view->getFieldAs<double>(Id::X, idx);
    double fieldY=point_view->getFieldAs<double>(Id::Y, idx);
    double fieldZ=point_view->getFieldAs<double>(Id::Z, idx);
    double classification=point_view->getFieldAs<double>(Id::Classification, idx);
    double x = fieldX;
    double y = fieldY;
    double z = fieldZ;
    p.x=x;
    p.y=y;
    p.z=z;
    p.intensity=classification;       
    points.push_back(p);  
  }
}