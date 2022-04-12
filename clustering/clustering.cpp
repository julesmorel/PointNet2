#include<iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/algorithm/string.hpp>

#include <pcl/segmentation/extract_clusters.h>

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
  double clusterTolerance = 1.;
  int minClusterSize=10;
  int maxClusterSize=1000;

  if (argc == 6) {
    filenameIn = argv[1];
    filenameOut = argv[2];
    clusterTolerance = std::stod (argv[3]);
    minClusterSize = std::stoi (argv[4]);
    maxClusterSize = std::stoi (argv[5]);
  }else{
    std::cout<<"Please specify a file to process"<<std::endl;
    exit(0);
  }

  pcl::PointCloud<pcl::PointXYZ> pts;
  readAsciiFile(filenameIn,pts);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (pts.makeShared());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (clusterTolerance); // 2cm
  ec.setMinClusterSize (minClusterSize);
  ec.setMaxClusterSize (maxClusterSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pts.makeShared());
  ec.extract (cluster_indices);

  //let's find the largest cluster
  int idLargestCluster=0;
  int sizeLargestCluster=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    if(it->indices.size()>sizeLargestCluster)
    {
      idLargestCluster=it-cluster_indices.begin();
      sizeLargestCluster=it->indices.size();
    }
  }

  //retrieve the points corresponding to this cluster
  pcl::PointCloud<pcl::PointXYZ> ptsFiltered;
  for(int i=0;i<sizeLargestCluster;i++){
    ptsFiltered.push_back(pts.at(cluster_indices.at(idLargestCluster).indices[i]));
  }

  std::cout<<pts.size()-ptsFiltered.size()<<" points filtered"<<std::endl;

  std::ofstream outfile;
  outfile.open(filenameOut, std::ios_base::app);

  for(int i=0;i<ptsFiltered.size();i++){
      pcl::PointXYZ currentPt = ptsFiltered.at(i);
      outfile <<currentPt.x<<" "<<currentPt.y<<" "<<currentPt.z<<'\n';
  }
}
