#include <ros/ros.h>
#include <iostream>
#include <ros/console.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "nav_msgs/OccupancyGrid.h"
#include "hexagon_map/point.h"
#include "hexagon_map/hexagon_map.h"
#include <fstream>
#include <iomanip>
#include <vector>
#include <deque>
#include <set>
#include <queue>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>

// nav_msgs::OccupancyGrid gridmap;
// int count = 250;
// void drawplogon(const float &C_x,const float &C_y,const float &L)
// {
//     int dx = floor((C_x - gridmap.info.origin.position.x)/gridmap.info.resolution);
//     int dy = floor((C_y - gridmap.info.origin.position.y)/gridmap.info.resolution);
    
//     float h = L * cos(30.0/180.0*3.1415926);
	
//    	int h_index = ceil(h/gridmap.info.resolution);


//     for(int i=-h_index;i<=h_index;i++)
//     {
//         float this_h = h - i * gridmap.info.resolution;
		
//         if(i<0)
//         {
//             this_h = h + i * gridmap.info.resolution;
//         }
//         else
//         {
//             this_h = h - i * gridmap.info.resolution;

//         }

//         float addlen = this_h * tan(30.0/180.0*3.1415926);
		
//         int addlen_index = ceil(addlen/gridmap.info.resolution);
		
//         int L_index = ceil(L/2/gridmap.info.resolution);
		
// 		int half_index = addlen_index + L_index;

//         for(int j=-half_index;j<=half_index;j++)
//         {
//             int deta_x = dx + j;
//             int deta_y = dy + i;
//             int index = deta_x + deta_y * gridmap.info.width;
//             if(gridmap.data[index]==0)
//             {
//                   gridmap.data[index] = count;
//             }
//         }
//     }
// }

// std::vector<CPoint2d> Pointset;

// bool isInpointset(CPoint2d &Checkpoint)
// {
//     for(int i = 0;i<Pointset.size();i++)
//     {
//         CPoint2d tempoint = Pointset[i];

//         float dis = (tempoint.x - Checkpoint.x) * (tempoint.x - Checkpoint.x) 
//                     + (tempoint.y - Checkpoint.y) * (tempoint.y - Checkpoint.y); 
        
//         if(dis<0.0001)
//         {
//             return true;
//         }
//     }
//     return false;
// }

float R = 7.0;
float resolution = 0.1;
int count = 0;

hexagon_map* hexagon_map_;
ros::Publisher *g_path1Publisher;
ros::Subscriber Getcloudpointssuber; 


void PointcloudCallback(sensor_msgs::PointCloud2 msg)
{
	count++;
	if(count==2)
	{
		hexagon_map_->updateMap(msg);
		std::cout<<"------- "<<std::endl;
		g_path1Publisher->publish(hexagon_map_->gridmap);
		count = 0;
	}
	
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Hexagon_map");
    ros::NodeHandle node;
	
    g_path1Publisher = new ros::Publisher;
    *g_path1Publisher = node.advertise<nav_msgs::OccupancyGrid>("Hexagon_map", 20);

    Getcloudpointssuber = node.subscribe("/rslidar_points", 100, &PointcloudCallback);

    hexagon_map_ = new hexagon_map(R,resolution);

    // std::cout<<"input num "<<std::endl;
    // float X,Y;
    // std::cin>>X;
    // std::cin>>Y;
    // CPoint2d tempoint;
    // tempoint.x = X;
    // tempoint.y = Y;

    // CPoint2d Centerp;
    // Centerp = hexagon_map_.JudgeHexCenpoint(tempoint);
    // std::cout<<"Centerp is "<<Centerp.x<<" "<<Centerp.y<<std::endl;
    // int count = 250;
    // hexagon_map_.drawplogon(Centerp.x,Centerp.y,hexagon_map_.Resulution_L,count);
    
 //    gridmap.header.frame_id = "/camera_init";
 //    gridmap.header.stamp = ros::Time::now();
 //    gridmap.info.resolution = 0.01;
 //    gridmap.info.width = 3600;
 //    gridmap.info.height = 3600;
 //    gridmap.info.origin.position.x = -18.0;
 //    gridmap.info.origin.position.y = -18.0;
 //    gridmap.info.origin.position.z = 0.0;

 //    gridmap.data.assign(gridmap.info.width*gridmap.info.height,0);

 //    std::vector<int> flagmap;
 //    flagmap.assign(gridmap.info.width*gridmap.info.height,0);

 //    float L = 0.2;
	// std::cout<<L<<std::endl;
	
	// CPoint2d Centerpoint;
	// Centerpoint.x = 0.0;
	// Centerpoint.y = 0.0;
	

	// std::queue<CPoint2d> Pointqueue;

 //    int geshu = 0;

 //    std::cout<<"--------Centerpoint--------- "<<Centerpoint.x<<" "<<Centerpoint.y<<std::endl;
 //    int d_x = (Centerpoint.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
 //    int d_y = (Centerpoint.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
 //    int inde_x = d_x + d_y * gridmap.info.width;
 //    std::cout<<"inde_x "<<inde_x<<std::endl;
 //    flagmap[inde_x] = 1;
 //    geshu++;

 //    Pointset.push_back(Centerpoint);
    
 //    Pointqueue.push(Centerpoint);

	// while(!Pointqueue.empty())
	// {
	// 	int size = Pointset.size();
 //        int size1 = Pointqueue.size();
	// 	std::cout<<"------------------------ "<<size<<std::endl;
 //        std::cout<<"Pointqueue size is "<<size1<<std::endl;
 //        std::cout<<"geshu is ........"<<std::endl;        
 //        std::cout<<geshu<<std::endl;
 //        if(geshu==6163)
 //        {
 //            break;

 //        }
	// 	/*if(size==91)
	// 	{
	// 		break;
	// 	}*/

	// 	int angle[]={60,120,180,240,300};
		
	// 	CPoint2d Genepoint;
	// 	Genepoint = Pointqueue.front();
	// 	Pointqueue.pop();

	// 	std::cout<<"loop begin_________________________Genepoint "<<Genepoint.x<<" "<<Genepoint.y<<std::endl;

	// 	CPoint2d tempoint;
		
	// 	tempoint.x = Genepoint.x + 4.5 * L;
	// 	tempoint.y = Genepoint.y + -L * tan(60*3.1415926/180)/2;
	// 	tempoint.pointangle = atan2(-L * tan(60*3.1415926/180)/2, 4.5 * L);
	// 	float R = sqrt(4.5 * L * 4.5 * L + -L * tan(60*3.1415926/180)/2 * -L * tan(60*3.1415926/180)/2);

	// 	int dx_ = (tempoint.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
 //        int dy_ = (tempoint.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
 //        int index_ = dx_ + dy_ * gridmap.info.width;
	// 	std::cout<<"tempoint "<<tempoint.x<<" "<<tempoint.y<<std::endl;
 //        std::cout<<" index_ "<<index_<<std::endl;
 //        geshu++;

 //        if(flagmap[index_]!=0 || isInpointset(tempoint))
 //        {   
 //             std::cout<<" chongfu_____ "<<std::endl;
 //        }
	//     else
	//     {
 //            Pointset.push_back(tempoint);
	// 		Pointqueue.push(tempoint);
 //            flagmap[index_] = 1;
	//     }



	// 	for(int i=0;i<5;i++)
	// 	{
	// 		CPoint2d Othertempoint;
	// 		Othertempoint.x = Genepoint.x + R * cos((tempoint.pointangle * 180 /3.1415926 + angle[i])*3.1415926/180);
	// 		Othertempoint.y = Genepoint.y + R * sin((tempoint.pointangle * 180 /3.1415926 + angle[i])*3.1415926/180);
		    
		    
		    
	// 	    int d_x2 = (Othertempoint.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
	// 	    int d_y2 = (Othertempoint.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
	// 	    int inde_x2 = d_x2 + d_y2 * gridmap.info.width;
	// 	    //flagmap[inde_x2] = 1;
	// 		std::cout<<"-------first Othertempoint "<<Othertempoint.x<<" "<<Othertempoint.y<<std::endl;
	// 	    std::cout<<"inde_x2 "<<inde_x2<<std::endl;
 //            geshu++;

	// 		if(flagmap[inde_x2]!=0 || isInpointset(Othertempoint))
 //            {
 //                std::cout<<"chongfu"<<inde_x2<<std::endl;
 //                continue;
 //            }
 //            else
 //            {
 //                Pointset.push_back(Othertempoint);
	// 			Pointqueue.push(Othertempoint);
 //            	flagmap[inde_x2] = 1;
 //            }
	// 	}
	// }
    

/*	
	int angle[]={60,120,180,240,300};
	
	CPoint2d tempoint;
	tempoint.x = 4.5 * L;
	tempoint.y = -L * tan(60*3.1415926/180)/2;
	tempoint.pointangle = atan2(tempoint.y,tempoint.x); 
	float R = sqrt(tempoint.x * tempoint.x + tempoint.y * tempoint.y);


    std::cout<<"----first tempoint "<<tempoint.x<<" "<<tempoint.y<<std::endl;
    int d_x1 = (tempoint.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
    int d_y1 = (tempoint.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
    int inde_x1 = d_x1 + d_y1 * gridmap.info.width;
    flagmap[inde_x1] = 1;
    std::cout<<"inde_x1 "<<inde_x1<<std::endl;

	Pointset.push_back(tempoint);
	
	for(int i=0;i<5;i++)
	{
		CPoint2d Othertempoint;
		Othertempoint.x = Centerpoint.x + R * cos((tempoint.pointangle * 180 /3.1415926 + angle[i])*3.1415926/180);
		Othertempoint.y = Centerpoint.y + R * sin((tempoint.pointangle * 180 /3.1415926 + angle[i])*3.1415926/180);
        
        
        std::cout<<"-------first Othertempoint "<<Othertempoint.x<<" "<<Othertempoint.y<<std::endl;
        int d_x2 = (Othertempoint.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
        int d_y2 = (Othertempoint.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
        int inde_x2 = d_x2 + d_y2 * gridmap.info.width;
        flagmap[inde_x2] = 1;
        std::cout<<"inde_x2 "<<inde_x2<<std::endl;

		Pointset.push_back(Othertempoint);
		
        //std::cout<<" Othertempoint　"<<Othertempoint.x<<" "<<Othertempoint.y<<std::endl;
	}


    
    std::cout<<Pointset.size()<<std::endl;
    std::cout<<"----------------------------"<<std::endl;

    std::vector<CPoint2d> PointsetCopy;
    PointsetCopy = Pointset;

    for(int i=1;i<Pointset.size();i++)
    {
        CPoint2d Cpoint;
        Cpoint = Pointset[i];
        std::cout<<"--------- shengcheng Cpoint------------------------ "<<Cpoint.x<<" "<<Cpoint.y<<std::endl;

        CPoint2d tem;
        tem.x = Cpoint.x + 4.5 * L;
	    tem.y = Cpoint.y + -L * tan(60*3.1415926/180)/2;
	    tem.pointangle = atan2(-L * tan(60*3.1415926/180)/2, 4.5 * L);
        

        std::cout<<"tem "<<tem.x<<" "<<tem.y<<std::endl;
        int dx_ = (tem.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
        int dy_ = (tem.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
        int index_ = dx_ + dy_ * gridmap.info.width;
        std::cout<<" index_ "<<index_<<std::endl;

        if(flagmap[index_]!=0)
        {   
             std::cout<<" chongfu_____ "<<std::endl;
        }
	    else
	    {
            PointsetCopy.push_back(tem);
            flagmap[index_] = 1;
	    }
   

        //PointsetCopy.push_back(tem);
        //flagmap[inde_x2] = 1;
      

        for(int j=0;j<5;j++)
	    {

		    CPoint2d Othertempoint1;
		    Othertempoint1.x = Cpoint.x + R * cos((tempoint.pointangle * 180 /3.1415926 + angle[j])*3.1415926/180);
		    Othertempoint1.y = Cpoint.y + R * sin((tempoint.pointangle * 180 /3.1415926 + angle[j])*3.1415926/180);
        

            std::cout<<"Othertempoint1 "<<Othertempoint1.x<<" "<<Othertempoint1.y<<std::endl;
            int d_x21 = (Othertempoint1.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
            int d_y21 = (Othertempoint1.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
            int inde_x21 = d_x21 + d_y21 * gridmap.info.width;
            std::cout<<"inde_x21 "<<inde_x21<<std::endl;


            
            if(flagmap[inde_x21]!=0)
            {
                std::cout<<"chongfu"<<inde_x21<<std::endl;
                continue;
            }
            else
            {
                 PointsetCopy.push_back(Othertempoint1);
                 flagmap[inde_x21] = 1;
            }
    
           // PointsetCopy.push_back(Othertempoint1);
	    }
   }

    std::cout<<"-------- "<<PointsetCopy.size()<<std::endl;

    Pointset.clear();
    Pointset = PointsetCopy;
*/
    // std::cout<<"--------------- Pointset.size() "<<Pointset.size()<<std::endl;
    // for(int i=0;i<Pointset.size();i++)
    // {
    //     std::cout<<"Pointset "<<Pointset[i].x<<" "<<Pointset[i].y<<std::endl;
    //     drawplogon(Pointset[i].x,Pointset[i].y,L);
    // }

 //    int singleangle[]={30,90,150,210,270,330};
	// float r1 = L * tan(60*3.1415926/180);
	
	// for(int i = 0;i<Pointset.size();i++)
	// {
	// 	count = count - 2;
	// 	drawplogon(Pointset[i].x,Pointset[i].y,L);
		
  
	// 	for(int j=0;j<6;j++)
	// 	{
	// 			CPoint2d temppoint;
	// 			temppoint.x = Pointset[i].x + r1 * cos(singleangle[j] * 3.1415926/180);
	// 			temppoint.y = Pointset[i].y + r1 * sin(singleangle[j] * 3.1415926/180);
	// 			count = count - 2;
	// 			drawplogon(temppoint.x,temppoint.y,L);
	// 	}
	// }



   /* for(int i = 1;i<Pointset.size();i++)
    {
        CPoint2d Cpoint;
        Cpoint = Pointset[i];

        CPoint2d tempoint;
        tempoint.x = Cpoint.x + 4.5 * L;
	    tempoint.y = Cpoint.y + -L * tan(60*3.1415926/180)/2;
	    tempoint.pointangle = atan2(-L * tan(60*3.1415926/180)/2, 4.5 * L); 

        Pointset.push_back(tempoint);

        for(int j=0;j<5;j++)
	    {
		    CPoint2d Othertempoint1;
		    Othertempoint1.x = Cpoint.x + R * cos((tempoint.pointangle * 180 /3.1415926 + angle[j])*3.1415926/180);
		    Othertempoint1.y = Cpoint.y + R * sin((tempoint.pointangle * 180 /3.1415926 + angle[j])*3.1415926/180);
		    Pointset.push_back(Othertempoint1);
		
            std::cout<<" Othertempoint1　"<<Othertempoint1.x<<" "<<Othertempoint1.y<<std::endl;
	    }
    }*/


	
	
	
	
	/*
	for(int i = -4;i<=4;i++)
		//int i = 0;
		for(int j=-5;j<=5;j++)
		{
			if(j%2==0)
			{
				C_x = 0.0 + j * 3 * L /2;
				C_y = 0.0 + i * tan(60*3.1415926/180) * L;
			}
			else
			{
				C_x = 0.0 + j * 3 * L /2;
				C_y = 0.0 + i * tan(60*3.1415926/180) * L + 
							tan(60*3.1415926/180) * L / 2;
			
			}
			//std::cout<<"C_x C_y "<<C_x<<" "<<C_y<<std::endl;
			drawplogon(C_x, C_y,L);
			count = count + 5;
		}*/
	
    /*L = 0.05;
    C_x = 0.0;
    C_y = 0.0;
    drawplogon(C_x,C_y,L);


    L = 0.05;
    C_x = 0.0 * L;
    C_y = -1.7321 * L;
    drawplogon(C_x, C_y,L);

    //L = 0.1;
    C_x = -1.5 * L;
    C_y = -0.866 * L;
    drawplogon(C_x, C_y,L);


    //L = 0.1;
    C_x = -1.5 * L;
    C_y = 0.866 * L;
    drawplogon(C_x, C_y,L);


    //L = 0.1;
    C_x = 0.0 * L;
    C_y = 1.7321 * L;
    drawplogon(C_x, C_y,L);

    //L = 0.1;
    C_x = 1.5 * L;
    C_y = 0.866 * L;
    drawplogon(C_x, C_y,L);

    
    //L = 0.1;
    C_x = 1.5 * L;
    C_y = -0.866 * L;
    drawplogon(C_x, C_y,L);*/

	// ros::Rate r(30);
	// while(ros::ok())
	// {
		
	// 	g_path1Publisher->publish(hexagon_map_.gridmap);
	// 	 r.sleep();

	// }
   
     ros::spin();

     delete hexagon_map_;

}
