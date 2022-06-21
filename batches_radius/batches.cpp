#include<iostream>
#include <fstream>
#include <string>
#include <map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/algorithm/string.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/common/common.h>

#include <sys/stat.h>

  void readAsciiFile(std::string filename, pcl::PointCloud<pcl::PointXYZ>& points, std::vector<std::string>& labels)
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

        std::string sum="";
        for(int i=3;i<results.size();i++)
        {
            sum+=" "+results.at(i);
        }
        labels.push_back(sum);
      }
      file.close();
    }
  }

  int main(int argc, char *argv[]){
    std::string filenameOut,filenameIn,filenameOutChunks,filenameOutChunksCenters;
    double cellSize;
    int numberNeighbors;
    int labelSpot;

    if (argc == 8) {
      filenameIn = argv[1];
      filenameOut = argv[2];
      filenameOutChunks = argv[3];
      filenameOutChunksCenters = argv[4];
      cellSize = std::stod(argv[5]);
      numberNeighbors = std::stoi(argv[6]);
      labelSpot = std::stoi(argv[7]);
    }else{
      std::cout<<"Please specify a file to process"<<std::endl;
      exit(0);
    }

    pcl::PointCloud<pcl::PointXYZ> pts;
    std::vector<std::string> labels;
    readAsciiFile(filenameIn,pts,labels);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (pts.makeShared());

     pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree(cellSize);
     octree.setInputCloud(pts.makeShared());
     octree.defineBoundingBox();
     octree.addPointsFromInputCloud();
     pcl::octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointTVector centroids;
     octree.getVoxelCentroids(centroids);

     pcl::PointCloud<pcl::PointXYZ>::Ptr ptsFiltered2(new pcl::PointCloud<pcl::PointXYZ>);
     ptsFiltered2->points.assign(centroids.begin(), centroids.end());
     ptsFiltered2->width = uint32_t(centroids.size());
     ptsFiltered2->height = 1;
     ptsFiltered2->is_dense = true;

     pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
     voxel_grid.setInputCloud (ptsFiltered2);
     voxel_grid.setLeafSize (cellSize, cellSize, cellSize);
     pcl::PointCloud<pcl::PointXYZ> ptsFiltered;
     voxel_grid.filter(ptsFiltered);

     pcl::PointCloud<pcl::PointXYZ>   ptsCloudChunks;
     std::vector<std::string> labelsChunks;
     std::map<int, int> mapChuncks;

     std::ofstream outfileCenters;
     outfileCenters.open(filenameOutChunksCenters, std::ios_base::app);
     for(int i=0;i<ptsFiltered.size();i++){
         pcl::PointXYZ currentPt = ptsFiltered.at(i);
         std::vector<int> k_indices(numberNeighbors);
         std::vector<float> k_sqr_distances(numberNeighbors);
         int numberN =  kdtree.nearestKSearch(currentPt,numberNeighbors,k_indices,k_sqr_distances);
         if(numberN>0){

             for( int j = 0; j< k_indices.size() ;j++){
                 pcl::PointXYZ p = pts.at(k_indices.at(j));
                 ptsCloudChunks.push_back(p);
                 labelsChunks.push_back(labels.at(k_indices.at(j)));
                 std::map<int, int>::iterator it = mapChuncks.find(k_indices.at(j));
                 if (it != mapChuncks.end()){
                   it->second = it->second+1;
                 }else{
                     mapChuncks.insert(std::pair<int, int>(k_indices.at(j),1));
                 }
             }
         }
         outfileCenters <<currentPt.x<<" "<<currentPt.y<<" "<<currentPt.z<<" "<<i<<'\n';
     }

     std::ofstream outfile;
     outfile.open(filenameOut, std::ios_base::app);
     for(int i=0;i<ptsCloudChunks.size();i++){
        pcl::PointXYZ p = ptsCloudChunks.at(i);
        std::string label = labelsChunks.at(i);
        outfile <<p.x<<" "<<p.y<<" "<<p.z<<label<<'\n';
     }

     std::ofstream outfile2;
     outfile2.open(filenameOutChunks, std::ios_base::app);
     for(int i=0;i<pts.size();i++){
       pcl::PointXYZ p = pts.at(i);
       int counter = mapChuncks.find(i)->second;
       std::vector<std::string> results;
       boost::split(results, labels.at(i), [](char c){return c == ' ';});
       int label = std::stoi(results.at(labelSpot));
       outfile2<<p.x<<" "<<p.y<<" "<<p.z<<" "<<counter<<" "<<label<<'\n';
     }
}
