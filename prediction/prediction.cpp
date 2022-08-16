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

#include "pybind11/embed.h"
#include "pybind11/numpy.h"
#include <pybind11/stl.h> 

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
  bool argsOk=true;
  float radius=0.1f;
  float cellSize=0.1f;
  int numberNeighbors=2048;

  //we need at least 3 args (+1 as the program name counts)
  if (argc > 4) {
    filenameIn = argv[1];
    filenameOut = argv[2];
    radius = std::stod(argv[3]);
    cellSize = std::stod(argv[4]);
    numberNeighbors = std::stoi(argv[5]);
  }else{
    std::cout<<"Please specify arguments in the following order:"<<std::endl;
    std::cout<<"file_1 ... file_N output_file"<<std::endl;
    exit(0);
  }

  pcl::PointCloud<pcl::PointXYZI> points;
    
  //read the offset for the current file if it exists
  offsetManager offsetM(filenameIn);
  double offset_x = offsetM.getOffsetX();
  double offset_y = offsetM.getOffsetY();

  //read the points in the file 
  pointCloudFileReader::read(filenameIn,points,offset_x,offset_y);
  std::cout<<"Input point cloud: "<<points.size()<<" points"<<std::endl;

  //store of the offset if it was not stored before
  offsetM.setOffset(offset_x,offset_y);

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
  std::cout<<ptsFiltered.size()<<" centers!"<<std::endl;

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
  std::cout<<ptsCloudChunks.size()<<" points in chuncks!"<<std::endl;

  pybind11::scoped_interpreter guard{}; 
  //pybind11::module pred = pybind11::module::import("../deepNetwork/predictOneBatch");
    pybind11::module sys = pybind11::module::import("sys");
    (sys.attr("path")).attr("append")("../deepNetwork");
    pybind11::object mod = pybind11::module::import("predictOneBatchStub");
    std::cout<<"Module imported"<<std::endl;
    pybind11::object pred = mod.attr("inference");
    pybind11::object predictor = pred("modelName");
    std::cout<<"object ok"<<std::endl;
    pybind11::object r = pred.attr("run");
    std::vector<std::vector<double>> data;
    data.push_back(std::vector<double>{1,2,3});
    data.push_back(std::vector<double>{4,5,6});
    std::cout<<"predicting.."<<std::endl;
    pybind11::array_t<double> res = r(predictor,data);
    pybind11::print(res);
    //pybind11::array_t<double> res = r(predictor,data);
    //double* cls = reinterpret_borrow<double*>(res);
    //double* cls = res.cast<double*>();
    //double* input_ptr = reinterpret_cast<double*>(res);
    //auto v = new std::vector<double>(r(predictor,data));
    //std::vector<double> &cls = res.cast<std::vector<double>>();
    //std::vector<double> cls = res;
    std::cout<<"casting done"<<std::endl;
    std::cout<<res.at(0)<<std::endl;

  //Sending the batches for prediction
  // int batchesNumber=ptsCloudChunks.size()/numberNeighbors;
  // for(int k=0;k<ptsFiltered.size()-1;k++){
  //   // std::vector<pcl::PointXYZI>::const_iterator first = ptsCloudChunks.begin() + batchesNumber*k;
  //   // std::vector<pcl::PointXYZI>::const_iterator last = ptsCloudChunks.begin() + batchesNumber*(k+1);
  //   // pcl::PointCloud<pcl::PointXYZI> batchesPoints(first,last);

  //   std::vector<pca_ratio>::const_iterator firstPCA = labelsChunks.begin() + batchesNumber*k;
  //   std::vector<pca_ratio>::const_iterator lastPCA = labelsChunks.begin() + batchesNumber*(k+1);
  //   std::vector<pca_ratio> batchesPCA(firstPCA,lastPCA);
  // }

}
