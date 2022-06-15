#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct Extent
{
   int xMin, xMax, yMin, yMax;
};

class pointCloudFileReader
{
public:
    static void read(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points, double& offset_x, double& offset_y, Extent limits);   
    static void readAsciiFile(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points, Extent limits); 
    static void readLasFile(std::string filename, pcl::PointCloud<pcl::PointXYZI>& points, double& offset_x, double& offset_y, Extent limits); 
};