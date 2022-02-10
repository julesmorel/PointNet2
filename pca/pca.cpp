#include<iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/pca.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/filters/crop_box.h>

#include <boost/algorithm/string.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
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
  std::string filenameOut,filenameIn;
  int numberNeighbors = 16;

  if (argc == 4) {
    filenameIn = argv[1];
    filenameOut = argv[2];
    numberNeighbors = std::stoi (argv[3]);
    //std::cout<<"Processing file (wood) : "+filenameWood<<std::endl;
  }else{
    std::cout<<"Please specify a file to process"<<std::endl;
    exit(0);
  }

  pcl::PointCloud<pcl::PointXYZ> pts;
  std::vector<std::string> labels;
  readAsciiFile(filenameIn,pts,labels);

  std::cout<<filenameOut<<" "<<pts.size()<<std::endl;

pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
kdtree.setInputCloud (pts.makeShared());

std::ofstream outfile;
outfile.open(filenameOut, std::ios_base::app);

for(int i=0;i<pts.size();i++){
    pcl::PointXYZ currentPt = pts.at(i);
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    int numberN =  kdtree.nearestKSearch(currentPt,numberNeighbors,k_indices,k_sqr_distances);

    if(numberN>3){
        pcl::PointCloud<pcl::PointXYZ>   ptsCloudPCA;
        for( int j = 0; j< k_indices.size() ;j++){
            pcl::PointXYZ p = pts.at(k_indices.at(j));
            ptsCloudPCA.push_back(p);
        }
        pcl::PCA< pcl::PointXYZ > pca;
        pca.setInputCloud(ptsCloudPCA.makeShared());
        Eigen::Vector3f eigenValues = pca.getEigenValues();
        float ratio = eigenValues(2)/(eigenValues(0)+eigenValues(1)+eigenValues(2));
        float ratio1 = eigenValues(1)/(eigenValues(0)+eigenValues(1)+eigenValues(2));
        float ratio2 = eigenValues(0)/(eigenValues(0)+eigenValues(1)+eigenValues(2));

      outfile <<currentPt.x<<" "<<currentPt.y<<" "<<currentPt.z<<" "<<ratio<<" "<<ratio1<<" "<<ratio2<<labels.at(i)<<std::endl;
    }
  }
}
