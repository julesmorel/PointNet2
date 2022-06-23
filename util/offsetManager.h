#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class offsetManager
{
public:
    offsetManager(std::string _filename);
    
    bool isStored(){return offset_stored;}
    double getOffsetX(){return offset_x;}
    double getOffsetY(){return offset_y;}   
    void setOffset(double _offset_x,double _offset_y);
private:
    //filenam of the current point cloud
    std::string filename;
    //offset file name, i.e. "offset.txt" in the same folder as the point cloud file
    std::string offsetFileName;
    bool offset_stored;
    double offset_x;
    double offset_y; 
};