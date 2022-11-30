#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class pointCloudFileReader
{
public:
    static void read(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points, double& offset_x, double& offset_y);   
    static void readAsciiFile(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points); 
    static void readLasFile(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points, double& offset_x, double& offset_y); 

    static void read(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points);   
    static void readLasFile(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points); 
};