#include<iostream>
#include <fstream>
#include <string>
#include <regex>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

// #include "pybind11/embed.h"
// #include "pybind11/numpy.h"
// #include <pybind11/stl.h> 

#include "../util/pointCloudFileReader.h"
#include "../util/offsetManager.h"

struct pca_ratio {   // Declare PERSON struct type
    float r1; 
    float r2;
    float r3;
};

int main(int argc, char *argv[]){
  std::string filenameIn;
  std::string filenameOut;
  std::string filenameOutCounter;
  bool argsOk=true;
  float radius=0.1f;
  float cellSize=0.1f;
  int numberNeighbors=2048;

  //we need at least 3 args (+1 as the program name counts)
  if (argc > 6) {
    filenameIn = argv[1];
    filenameOut = argv[2];
    filenameOutCounter = argv[3];
    radius = std::stod(argv[4]);
    cellSize = std::stod(argv[5]);
    numberNeighbors = std::stoi(argv[6]);
  }else{
    std::cout<<"Please specify arguments in the following order:"<<std::endl;
    std::cout<<"file_1 ... file_N output_file"<<std::endl;
    exit(0);
  }

  pcl::PointCloud<pcl::PointXYZI> points;
    
  //read the offset for the current file if it exists
  //offsetManager offsetM(filenameIn);
  //double offset_x = offsetM.getOffsetX();
  //double offset_y = offsetM.getOffsetY();

  //read the points in the file 
  //pointCloudFileReader::read(filenameIn,points,offset_x,offset_y);
  pointCloudFileReader::read(filenameIn,points);
  //std::cout<<"Input point cloud: "<<points.size()<<" points"<<std::endl;

  //store of the offset if it was not stored before
  //offsetM.setOffset(offset_x,offset_y);

  //Compute the PCA
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtreePCA;
  kdtreePCA.setInputCloud (points.makeShared());
  pcl::PointCloud<pcl::PointXYZI> ptsPCA;   
  std::vector<pca_ratio> listPcaRatios;
  for(int i=0;i<points.size();i++){
    pcl::PointXYZI currentPt = points.at(i);
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    int numberN =  kdtreePCA.radiusSearch(currentPt,radius,k_indices,k_sqr_distances);

    //Need at least 3 points to compute the local PCA
    if(numberN>3){
      pcl::PointCloud<pcl::PointXYZI>   ptsCloudPCA;
      for( int j = 0; j< k_indices.size() ;j++){
        pcl::PointXYZI p = points.at(k_indices.at(j));
        ptsCloudPCA.push_back(p);
      }
      pcl::PCA< pcl::PointXYZI > pca;
      pca.setInputCloud(ptsCloudPCA.makeShared());
      Eigen::Vector3f eigenValues = pca.getEigenValues();
      pca_ratio ratios;
      ratios.r1 = eigenValues(2)/(eigenValues(0)+eigenValues(1)+eigenValues(2));
      ratios.r2 = eigenValues(1)/(eigenValues(0)+eigenValues(1)+eigenValues(2));
      ratios.r3 = eigenValues(0)/(eigenValues(0)+eigenValues(1)+eigenValues(2));

      ptsPCA.push_back(currentPt);
      listPcaRatios.push_back(ratios);
    }
  }
  //std::cout<<"PCA point cloud: "<<ptsPCA.size()<<" points"<<std::endl;

  //Retrieving the centers of the occupied cells in the octree
  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZI> octree(cellSize);
  octree.setInputCloud(ptsPCA.makeShared());
  octree.defineBoundingBox();
  octree.addPointsFromInputCloud();
  pcl::octree::OctreePointCloud<pcl::PointXYZI>::AlignedPointTVector centroids;
  octree.getVoxelCentroids(centroids);

  pcl::PointCloud<pcl::PointXYZI>::Ptr ptsFiltered2(new pcl::PointCloud<pcl::PointXYZI>);
  ptsFiltered2->points.assign(centroids.begin(), centroids.end());
  ptsFiltered2->width = uint32_t(centroids.size());
  ptsFiltered2->height = 1;
  ptsFiltered2->is_dense = true;

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud (ptsFiltered2);
  voxel_grid.setLeafSize (cellSize, cellSize, cellSize);
  pcl::PointCloud<pcl::PointXYZI> ptsFiltered;
  voxel_grid.filter(ptsFiltered);
  //std::cout<<ptsFiltered.size()<<" centers!"<<std::endl;

  //Creation of the batches
  pcl::PointCloud<pcl::PointXYZI>   ptsCloudChunks;
  std::vector<pca_ratio> labelsChunks;
  std::map<int, int> mapChuncks;
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (ptsPCA.makeShared());

  for(int i=0;i<ptsFiltered.size();i++){
    pcl::PointXYZI currentPt = ptsFiltered.at(i);
    std::vector<int> k_indices(numberNeighbors);
    std::vector<float> k_sqr_distances(numberNeighbors);
    int numberN =  kdtree.nearestKSearch(currentPt,numberNeighbors,k_indices,k_sqr_distances);
    if(numberN>0){

        for( int j = 0; j< k_indices.size() ;j++){
            pcl::PointXYZI p = ptsPCA.at(k_indices.at(j));
            ptsCloudChunks.push_back(p);
            labelsChunks.push_back(listPcaRatios.at(k_indices.at(j)));
            std::map<int, int>::iterator it = mapChuncks.find(k_indices.at(j));
            if (it != mapChuncks.end()){
              it->second = it->second+1;
            }else{
                mapChuncks.insert(std::pair<int, int>(k_indices.at(j),1));
            }
        }
    }
  }
  //std::cout<<"Chunks point cloud: "<<ptsCloudChunks.size()<<" points"<<std::endl;

  std::ofstream outfile;
  outfile.open(filenameOut, std::ios_base::app);
  for(int i=0;i<ptsCloudChunks.size();i++){
    pcl::PointXYZI p = ptsCloudChunks.at(i);
    pca_ratio pca = labelsChunks.at(i);
    outfile <<p.x<<" "<<p.y<<" "<<p.z<<" "<<pca.r1<<" "<<pca.r2<<" "<<pca.r3<<'\n';
  }

  std::ofstream outfile2;
  outfile2.open(filenameOutCounter, std::ios_base::app);
  for(int i=0;i<ptsPCA.size();i++){
    pcl::PointXYZI p = ptsPCA.at(i);
    int counter = mapChuncks.find(i)->second;
    outfile2<<p.x<<" "<<p.y<<" "<<p.z<<" "<<counter<<'\n';
  }

  // std::vector<double> predicted_label0;
  // std::vector<double> predicted_label1;

  // //Loading python interpreter
  // pybind11::scoped_interpreter guard{}; 

  // //Adding package directoy to python path
  // pybind11::module sys = pybind11::module::import("sys");
  // (sys.attr("path")).attr("append")("../deepNetwork");
  
  // //Importing module
  // pybind11::object mod = pybind11::module::import("predictOneBatch");
  
  // //Loading pytorch model in object instance 
  // pybind11::object pred = mod.attr("inference");
  // pybind11::object predictor = pred("../models/wood_segmentation/model_seg_sologne");

  // //Retrieving address of the prediction function
  // pybind11::object pred_func = pred.attr("run");
    
  // //Sending the batches for prediction
  // int batchesNumber=ptsCloudChunks.size()/numberNeighbors;
  // for(int k=0;k<batchesNumber;k++){
  //   std::vector<std::vector<double>> data;
  //   for(int n=0;n<numberNeighbors;n++){
  //     //use of STL container for direct casting to numpy array
  //     std::vector<double> enriched_point;
  //     pcl::PointXYZI p = ptsCloudChunks.at(k*numberNeighbors+n);
  //     pca_ratio r = labelsChunks.at(k*numberNeighbors+n);
  //     enriched_point.push_back(p.x);
  //     enriched_point.push_back(p.y);
  //     enriched_point.push_back(p.z);
  //     enriched_point.push_back(r.r1);
  //     enriched_point.push_back(r.r2);
  //     enriched_point.push_back(r.r3);
  //     data.push_back(enriched_point);
  //   }
  //   //Predicting the label of the current batch of points
  //   pybind11::array_t<double> res = pred_func(predictor,data);
  //   for(int n=0;n<numberNeighbors;n++){
  //     predicted_label0.push_back(res.at(n));
  //   }
  //   for(int n=numberNeighbors;n<2*numberNeighbors;n++){
  //     predicted_label1.push_back(res.at(n));
  //   }
  // }
  // std::cout<<"pts: "<<ptsCloudChunks.size()<<" labels 0: "<<predicted_label0.size()<<" and 1: "<<predicted_label1.size()<<std::endl;   

  // pcl::KdTreeFLANN<pcl::PointXYZI> kdtreeChunks;
  // kdtreeChunks.setInputCloud (ptsCloudChunks.makeShared());

  // //Voting process
  // std::ofstream outfile;
  // outfile.open(filenameOut, std::ios_base::app);
  // for(int i=0;i<ptsPCA.size();i++){
  //   pcl::PointXYZI currentPt = ptsPCA.at(i);
  //   int numberOfPointsToFind = mapChuncks.find(i)->second;
  //   int counter=0;
  //   if(numberOfPointsToFind>0){
  //     std::vector<int> k_indices(numberOfPointsToFind);
  //     std::vector<float> k_sqr_distances(numberOfPointsToFind);

  //     int numberN =  kdtreeChunks.nearestKSearch(currentPt,numberOfPointsToFind,k_indices,k_sqr_distances);

  //     double sumPred0=0.;
  //     double sumPred1=0.;
  //     if(numberN>0){
  //       for( int j = 0; j< k_indices.size() ;j++){
  //         sumPred0+=predicted_label0.at(k_indices.at(j));
  //         sumPred1+=predicted_label1.at(k_indices.at(j));
  //       }
  //     }
  //     double pred0Ratio = sumPred0/(double)numberOfPointsToFind;
  //     double pred1Ratio = sumPred1/(double)numberOfPointsToFind;
  //     int classif=0;
  //     if(pred1Ratio>pred0Ratio){
  //         classif=1;
  //     }
  //     outfile <<currentPt.x<<" "<<currentPt.y<<" "<<currentPt.z<<" "<<classif<<std::endl;
  //   }
  // }
}
