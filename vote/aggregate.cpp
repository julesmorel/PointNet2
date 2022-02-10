#include<iostream>
#include <fstream>
#include <string>

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

#include <sys/stat.h>

void readAsciiFile(std::string filename, pcl::PointCloud<pcl::PointXYZ>& points, std::vector<int>& ind, std::vector<double>& pred0, std::vector<double>& pred1)
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

        ind.push_back(std::stoi(results.at(3)));
        pred0.push_back(std::stod(results.at(4)));
        pred1.push_back(std::stod(results.at(5)));
      }
      file.close();
    }
  }

  void readAsciiFileChunked(std::string filename, pcl::PointCloud<pcl::PointXYZ>& points, std::vector<int>& chunks, std::vector<int>& labels)
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

          chunks.push_back(std::stoi(results.at(3)));
          labels.push_back(std::stoi(results.at(4)));
        }
        file.close();
      }
    }

    int main(int argc, char *argv[]){
      std::string filenamePred,filenameChunks,filenameOut;
      int batchSize;

      if (argc == 4) {
        filenamePred = argv[1];
        filenameChunks = argv[2];
        filenameOut = argv[3];
      }else{
        std::cout<<"Please specify a file to process"<<std::endl;
        exit(0);
      }

      pcl::PointCloud<pcl::PointXYZ> ptsPred;
      std::vector<int> labelsPred;
      std::vector<double> pred0;
      std::vector<double> pred1;
      readAsciiFile(filenamePred,ptsPred,labelsPred,pred0,pred1);

      pcl::PointCloud<pcl::PointXYZ> ptsChunks;
      std::vector<int> chunksCounter;
      std::vector<int> labels;
      readAsciiFileChunked(filenameChunks,ptsChunks,chunksCounter,labels);

      pcl::KdTreeFLANN<pcl::PointXYZ> kdtreePred;
      kdtreePred.setInputCloud (ptsPred.makeShared());

      std::ofstream outfile;
      outfile.open(filenameOut, std::ios_base::app);

      for(int i=0;i<ptsChunks.size();i++){

        pcl::PointXYZ currentPt = ptsChunks.at(i);
        int numberOfPointsToFind = chunksCounter.at(i);
        int counter=0;
        if(numberOfPointsToFind>0){
          //std::cout<<"numberOfPointsToFind : "<<numberOfPointsToFind<<std::endl;
          std::vector<int> k_indices(numberOfPointsToFind);
          std::vector<float> k_sqr_distances(numberOfPointsToFind);

          int numberN =  kdtreePred.nearestKSearch(currentPt,numberOfPointsToFind,k_indices,k_sqr_distances);

          int summedLabels=0;
          double sumPred0=0.;
          double sumPred1=0.;
          if(numberN>0){
            for( int j = 0; j< k_indices.size() ;j++){
              if(labelsPred.at(k_indices.at(j)) != -1)
              {
                summedLabels += labelsPred.at(k_indices.at(j));
                counter++;
              }
              sumPred0+=pred0.at(k_indices.at(j));
              sumPred1+=pred1.at(k_indices.at(j));
            }
          }
          float ratio = (float)summedLabels/(float)counter;
          double pred0Ratio = sumPred0/(double)numberOfPointsToFind;
          double pred1Ratio = sumPred1/(double)numberOfPointsToFind;

          int classif=0;
          if(pred1Ratio>pred0Ratio){
              classif=1;
          }
          int lab = labels.at(i);
          outfile <<currentPt.x<<" "<<currentPt.y<<" "<<currentPt.z<<" "<<classif<<" "<<lab<<std::endl;
        }
      }
    }
