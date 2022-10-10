#include<iostream>
#include <fstream>
#include <string>
#include <regex>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>

#include "../util/pointCloudFileReader.h"
#include "../util/offsetManager.h"

int main(int argc, char *argv[]){
  std::string filenameIn;
  std::string filenameWood;
  std::string filenameOut;
  double resolution;

  //we need at least 3 args (+1 as the program name counts)
  if (argc == 5) {
      filenameIn = argv[1];
      filenameWood = argv[2];
      filenameOut = argv[3];
      resolution = std::stod(argv[4]);
  }else{
      std::cout<<"Please specify 3 files and the resolution: input, wood, output and resolution"<<std::endl;
      exit(0);
  }
    
  //read the offset for the current file if it exists
  offsetManager offsetM(filenameIn);
  double offset_x = offsetM.getOffsetX();
  double offset_y = offsetM.getOffsetY();

  //read the points in the file 
  pcl::PointCloud<pcl::PointXYZI> pts;   
  pointCloudFileReader::read(filenameIn,pts,offset_x,offset_y);

  //store of the offset if it was not stored before
  offsetM.setOffset(offset_x,offset_y);

  pcl::PointCloud<pcl::PointXYZI> ptsWood;   
  pointCloudFileReader::read(filenameWood,ptsWood,offset_x,offset_y);

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (ptsWood.makeShared());

  for(int i=0;i<pts.size();i++){
    pcl::PointXYZI currentPt = pts.at(i);
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    int numberN =  kdtree.radiusSearch(currentPt,resolution,k_indices,k_sqr_distances);
    if(numberN>0){
      pts.at(i).intensity=3;
    }
  } 

  pdal::Options options;
  options.add("filename", filenameOut);

  pdal::PointTable table;
  table.layout()->registerDim(pdal::Dimension::Id::X);
  table.layout()->registerDim(pdal::Dimension::Id::Y);
  table.layout()->registerDim(pdal::Dimension::Id::Z);
  table.layout()->registerDim(pdal::Dimension::Id::Classification);

  pdal::PointViewPtr view(new pdal::PointView(table));

  for(int i=0;i<pts.size();i++){
    pcl::PointXYZI p = pts.at(i);
    view->setField(pdal::Dimension::Id::X, i, p.x+offset_x);
    view->setField(pdal::Dimension::Id::Y, i, p.y+offset_y);
    view->setField(pdal::Dimension::Id::Z, i, p.z);
    view->setField(pdal::Dimension::Id::Classification, i, p.intensity);
  }

  pdal::BufferReader reader;
  reader.addView(view);

  pdal::StageFactory factory;

  // Set second argument to 'true' to let factory take ownership of
  // stage and facilitate clean up.
  pdal::Stage *writer = factory.createStage("writers.las");

  writer->setInput(reader);
  writer->setOptions(options);
  writer->prepare(table);
  writer->execute(table);
}