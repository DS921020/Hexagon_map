#ifndef hexagon_map_H
#define hexagon_map_H
#include "nav_msgs/OccupancyGrid.h"
#include "hexagon_map/point.h"
#include <vector>
#include <vector>
#include <deque>
#include <set>
#include <queue>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

struct GridData
{
	GridData():top(-9999),bottom(9999),numPoint(0){}
    CPoint2d Centerp; //六边形栅格中心点
    float top;//栅格内点云高程的最大值
    float bottom;//栅格内点云高程的最小值
    uint32_t numPoint;//栅格内包含点的个数
};


class hexagon_map
{

public:

  	float Cover_R;       //六边形地图覆盖的半径
	float Resulution_L;  //每个六边形的边长，六边形栅格的分辨率
	int widthnum;        //最终六边形地图最外一圈六边形个数
	int limitednum;		//生成地图过程中用于控制生成六边形过程
	
	float RectangleL;   //投影时使用的矩形栅格长
	float RectangleW;   //投影时使用的矩形栅格宽

	// int count;


	nav_msgs::OccupancyGrid gridmap;  //基于底层的ros栅格地图生成六边形地图
	std::vector<CPoint2d> Pointset;   //主螺旋线上六边形的中心点
	std::vector<int> flagmap; //和栅格地图大小相同用来表示这些栅格是否已经被六边形占据或者六边形的属性
	std::queue<CPoint2d> Pointqueue; //六边形生成地图过程中用于保存位于主螺旋线上未被生成六边形的中心点
	std::vector<GridData> HexGridData; //存储投影到六边形的栅格信息


	hexagon_map(float R,float resolution);
	~hexagon_map();
	void buildmap();
	bool isInpointset(CPoint2d &Checkpoint);
	void drawplogon(const float &C_x,const float &C_y,const float &L,int count);
	void drawallmap();
	void updateMap(sensor_msgs::PointCloud2 &pointClouds_msg);
	int findindexoccpmap(CPoint2d becheckp);
	CPoint2d JudgeHexCenpoint(CPoint2d &Checkpoint);
	nav_msgs::OccupancyGrid getmap();
};
#endif 
