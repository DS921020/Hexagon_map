#include "hexagon_map/hexagon_map.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

hexagon_map::hexagon_map(float R,float resolution)
{
	std::cout<<"R "<<R<<" resolution "<<Resulution_L<<std::endl;
	
	Cover_R = R;
	Resulution_L = resolution;

	RectangleL = Resulution_L * 1.5;
	RectangleW = tan(60 * 3.1415926/180) * Resulution_L;
	
	widthnum = ceil(Cover_R/(sqrt(21.0) * Resulution_L));
	
	std::cout<<"widthnum "<<widthnum<<std::endl;
	
	limitednum = 1;
	int lastHexcount = 1;
	for(int i=2;i<=widthnum;i++)
	{
		//std::cout<<"i "<<i<<std::endl;
		//std::cout<<"limitednum "<<limitednum<<std::endl;
		limitednum = limitednum + 6 * lastHexcount;
		//std::cout<<"limitednum "<<limitednum<<std::endl;
		lastHexcount = (i - 1) * 6;
	}
	
	std::cout<<"limitednum "<<limitednum<<std::endl;

	int occupymapwidth = ceil(4.5 * Resulution_L * ( widthnum - 1) + 3 * Resulution_L);
	int occupymapheight = ceil(sin(60 * 3.1415926 /180) * 5 * Resulution_L * (widthnum - 1) + sin(60 * 3.1415926 /180) * 4 * Resulution_L);
	
	int occpymapsize = 0;
	if(occupymapwidth>occupymapheight)
	{
		occpymapsize = occupymapwidth;
	}
	else
	{
		occpymapsize = occupymapheight;
	}
	
	if(occpymapsize%2!=0)
	{
		occpymapsize = occpymapsize + 1;
	}
	
	gridmap.header.frame_id = "/rslidar";
    gridmap.header.stamp = ros::Time::now();
    gridmap.info.resolution = 0.01;
    gridmap.info.width = int(float(occpymapsize * 2)/0.01);
    gridmap.info.height = int(float(occpymapsize * 2)/0.01);
    gridmap.info.origin.position.x = -float(occpymapsize);
    gridmap.info.origin.position.y = -float(occpymapsize);
    gridmap.info.origin.position.z = 0.0;
	
    std::cout<<"gridmap.info.resolution "<<gridmap.info.resolution
    <<" gridmap.info.width "<<gridmap.info.width<<" gridmap.info.height "
    <<gridmap.info.height<<" gridmap.info.origin.position "<<gridmap.info.origin.position.x
    <<" "<< gridmap.info.origin.position.y<<std::endl;

	gridmap.data.assign(gridmap.info.width*gridmap.info.height,0);
	flagmap.assign(gridmap.info.width*gridmap.info.height,0);
	
	buildmap();
}

hexagon_map::~hexagon_map()
{
	
	
	
}

CPoint2d hexagon_map::JudgeHexCenpoint(CPoint2d &Checkpoint)
{
	int cx = floor((Checkpoint.x - 0.0)/RectangleL);
	int cy = floor((Checkpoint.y - 0.0)/RectangleW);

	// std::cout<<"RectangleL "<<RectangleL<<" RectangleW "<<RectangleW<<std::endl;
	// std::cout<<"cx "<<cx<<" cy "<<cy<<std::endl;


	CPoint2d finalpoint;
	CPoint2d hexcenpoint1;
	CPoint2d hexcenpoint2;
	CPoint2d hexcenpoint3;
	float dis = 10000;

	if(cx%2==0)
	{
		hexcenpoint1.x = cx * RectangleL;
		hexcenpoint1.y = cy * RectangleW;

		float temdis = Checkpoint.Dist(hexcenpoint1);
		if(temdis<dis)
		{
			finalpoint = hexcenpoint1;
			dis = temdis;
		}


		hexcenpoint2.x = cx * RectangleL;
		hexcenpoint2.y = (cy + 1) * RectangleW;

		float temdis1 = Checkpoint.Dist(hexcenpoint2);
		if(temdis1<dis)
		{
			finalpoint = hexcenpoint2;
			dis = temdis1;
		}


		hexcenpoint3.x = (cx + 1) * RectangleL;
		hexcenpoint3.y = (cy + 0.5) * RectangleW;

		float temdis2 = Checkpoint.Dist(hexcenpoint3);
		if(temdis2<dis)
		{
			finalpoint = hexcenpoint3;
			dis = temdis2;
		}

		// std::cout<<"hexcenpoint1 "<<hexcenpoint1.x<<" hexcenpoint1 "<<hexcenpoint1.y<<std::endl;
		// std::cout<<"hexcenpoint2 "<<hexcenpoint2.x<<" hexcenpoint2 "<<hexcenpoint2.y<<std::endl;
		// std::cout<<"hexcenpoint3 "<<hexcenpoint3.x<<" hexcenpoint3 "<<hexcenpoint3.y<<std::endl;
	}
	else
	{
		hexcenpoint1.x = cx * RectangleL;
		hexcenpoint1.y = (cy + 0.5) * RectangleW;


		float temdis = Checkpoint.Dist(hexcenpoint1);
		if(temdis<dis)
		{
			finalpoint = hexcenpoint1;
			dis = temdis;
		}

		hexcenpoint2.x = (cx + 1) * RectangleL;
		hexcenpoint2.y =  cy  * RectangleW;

		float temdis1 = Checkpoint.Dist(hexcenpoint2);
		if(temdis1<dis)
		{
			finalpoint = hexcenpoint2;
			dis = temdis1;
		}

		hexcenpoint3.x = (cx + 1) * RectangleL;
		hexcenpoint3.y = (cy + 1) * RectangleW;

		float temdis2 = Checkpoint.Dist(hexcenpoint3);
		if(temdis2<dis)
		{
			finalpoint = hexcenpoint3;
			dis = temdis2;
		}

		// std::cout<<"--------hexcenpoint1 "<<hexcenpoint1.x<<" hexcenpoint1 "<<hexcenpoint1.y<<std::endl;
		// std::cout<<"hexcenpoint2 "<<hexcenpoint2.x<<" hexcenpoint2 "<<hexcenpoint2.y<<std::endl;
		// std::cout<<"hexcenpoint3 "<<hexcenpoint3.x<<" hexcenpoint3 "<<hexcenpoint3.y<<std::endl;
	}

	return finalpoint;
}

void hexagon_map::drawplogon(const float &C_x,const float &C_y,const float &L,int count)
{
    int dx = floor((C_x - gridmap.info.origin.position.x)/gridmap.info.resolution);
    int dy = floor((C_y - gridmap.info.origin.position.y)/gridmap.info.resolution);
    
    float h = L * cos(30.0/180.0*3.1415926);
	
   	int h_index = ceil(h/gridmap.info.resolution);


    for(int i=-h_index;i<=h_index;i++)
    {
        float this_h = h - i * gridmap.info.resolution;
		
        if(i<0)
        {
            this_h = h + i * gridmap.info.resolution;
        }
        else
        {
            this_h = h - i * gridmap.info.resolution;

        }

        float addlen = this_h * tan(30.0/180.0*3.1415926);
		
        int addlen_index = ceil(addlen/gridmap.info.resolution);
		
        int L_index = ceil(L/2/gridmap.info.resolution);
		
		int half_index = addlen_index + L_index;

        for(int j=-half_index;j<=half_index;j++)
        {
            int deta_x = dx + j;
            int deta_y = dy + i;
            int index = deta_x + deta_y * gridmap.info.width;
            //if(gridmap.data[index]==0)
            //{
            	gridmap.data[index] = count;
            //}
        }
    }
}

bool hexagon_map::isInpointset(CPoint2d &Checkpoint)
{
    for(int i = 0;i<Pointset.size();i++)
    {
        CPoint2d tempoint = Pointset[i];

        float dis = (tempoint.x - Checkpoint.x) * (tempoint.x - Checkpoint.x) 
                    + (tempoint.y - Checkpoint.y) * (tempoint.y - Checkpoint.y); 
        
        if(dis<0.0001)
        {
            return true;
        }
    }
    return false;
}

void hexagon_map::drawallmap()
{
	int singleangle[]={30,90,150,210,270,330};
	float r1 = Resulution_L * tan(60*3.1415926/180);
	int count = 10;
	for(int i = 0;i<Pointset.size();i++)
	{
		//count = count - 2;
		drawplogon(Pointset[i].x,Pointset[i].y,Resulution_L,count);
		
  
		for(int j=0;j<6;j++)
		{
				CPoint2d temppoint;
				temppoint.x = Pointset[i].x + r1 * cos(singleangle[j] * 3.1415926/180);
				temppoint.y = Pointset[i].y + r1 * sin(singleangle[j] * 3.1415926/180);
				//count = count - 2;
				drawplogon(temppoint.x,temppoint.y,Resulution_L,count);
		}
	}
}

void hexagon_map::buildmap()
{
	CPoint2d Centerpoint;
	Centerpoint.x = 0.0;
	Centerpoint.y = 0.0;
	
	int limitcount = 0;
	
	// std::cout<<"--------Centerpoint--------- "<<Centerpoint.x<<" "<<Centerpoint.y<<std::endl;
    int d_x = (Centerpoint.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
    int d_y = (Centerpoint.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
    int inde_x = d_x + d_y * gridmap.info.width;
    // std::cout<<"inde_x "<<inde_x<<std::endl;
    flagmap[inde_x] = 1;
    limitcount++;

    Pointset.push_back(Centerpoint);
    
    Pointqueue.push(Centerpoint);
	
	while(!Pointqueue.empty())
	{
		int size = Pointset.size();
        int size1 = Pointqueue.size();
		// std::cout<<"------------------------ "<<size<<std::endl;
  //       std::cout<<"Pointqueue size is "<<size1<<std::endl;
  //       std::cout<<"limitcount is ........"<<std::endl;        
  //       std::cout<<limitcount<<std::endl;
        if(limitcount==limitednum)
        {
            break;

        }

		int angle[]={60,120,180,240,300};
		
		CPoint2d Genepoint;
		Genepoint = Pointqueue.front();
		Pointqueue.pop();

		// std::cout<<"loop begin_________________________Genepoint "<<Genepoint.x<<" "<<Genepoint.y<<std::endl;

		CPoint2d tempoint;
		
		tempoint.x = Genepoint.x + 4.5 * Resulution_L;
		tempoint.y = Genepoint.y + -Resulution_L * tan(60*3.1415926/180)/2;
		tempoint.pointangle = atan2(-Resulution_L * tan(60*3.1415926/180)/2, 4.5 * Resulution_L);
		float R = sqrt(4.5 * Resulution_L * 4.5 * Resulution_L + -Resulution_L * tan(60*3.1415926/180)/2 * -Resulution_L * tan(60*3.1415926/180)/2);

		int dx_ = (tempoint.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
        int dy_ = (tempoint.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
        int index_ = dx_ + dy_ * gridmap.info.width;
		// std::cout<<"tempoint "<<tempoint.x<<" "<<tempoint.y<<std::endl;
  //       std::cout<<" index_ "<<index_<<std::endl;
        limitcount++;

        if(flagmap[index_]!=0 || isInpointset(tempoint))
        {   
             // std::cout<<" chongfu_____ "<<std::endl;
        }
	    else
	    {
            Pointset.push_back(tempoint);
			Pointqueue.push(tempoint);
            flagmap[index_] = 1;
	    }



		for(int i=0;i<5;i++)
		{
			CPoint2d Othertempoint;
			Othertempoint.x = Genepoint.x + R * cos((tempoint.pointangle * 180 /3.1415926 + angle[i])*3.1415926/180);
			Othertempoint.y = Genepoint.y + R * sin((tempoint.pointangle * 180 /3.1415926 + angle[i])*3.1415926/180);
		    
		    
		    
		    int d_x2 = (Othertempoint.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
		    int d_y2 = (Othertempoint.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
		    int inde_x2 = d_x2 + d_y2 * gridmap.info.width;
		    //flagmap[inde_x2] = 1;
			// std::cout<<"-------first Othertempoint "<<Othertempoint.x<<" "<<Othertempoint.y<<std::endl;
		 //    std::cout<<"inde_x2 "<<inde_x2<<std::endl;
            limitcount++;

			if(flagmap[inde_x2]!=0 || isInpointset(Othertempoint))
            {
                // std::cout<<"chongfu"<<inde_x2<<std::endl;
                continue;
            }
            else
            {
                Pointset.push_back(Othertempoint);
				Pointqueue.push(Othertempoint);
            	flagmap[inde_x2] = 1;
            }
		}
	}
	
	std::cout<<"--------------- Pointset.size() "<<Pointset.size()<<std::endl;
    //int count = 10;
    for(int i=0;i<Pointset.size();i++)
    {
        std::cout<<"Pointset "<<Pointset[i].x<<" "<<Pointset[i].y<<std::endl;
        //drawplogon(Pointset[i].x,Pointset[i].y,Resulution_L,count);
    }

    //count = 100;
	drawallmap();
}


int hexagon_map::findindexoccpmap(CPoint2d becheckp)
{
	// std::cout<<"get in "<<std::endl;
	for(int i=0;i<HexGridData.size();i++)
	{

		float dis = HexGridData[i].Centerp.Dist(becheckp);

		if(dis<0.0001)
		{
			return i;
		}
	}
	return -1;
}


void hexagon_map::updateMap(sensor_msgs::PointCloud2 &pointClouds_msg)
{
	if(!HexGridData.empty())
	{
		HexGridData.clear();
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::moveFromROSMsg(pointClouds_msg,*cloud);
	
	float minx = gridmap.info.origin.position.x;
	float miny = gridmap.info.origin.position.y;
	float maxx = minx * (-1);
	float maxy = miny * (-1);

	std::cout<<"cloud->points.size() "<<cloud->points.size()<<std::endl;
	std::cout<<" minx miny"<<minx<<" miny "<<miny<<" maxx "<<maxx<<" maxy "<<maxy<<std::endl;


	pcl::PointCloud<pcl::PointXYZI>::Ptr prcloud (new pcl::PointCloud<pcl::PointXYZI>);
	for(int i=0;i<cloud->points.size();i++)//对所有激光点处理
	{
		if(cloud->points[i].x < minx || cloud->points[i].y < miny
			|| cloud->points[i].x > maxx || cloud->points[i].y > maxy)
		{
			// std::cout<<"out of side.......... "<<std::endl;
			continue;
		}

		prcloud->points.push_back(cloud->points[i]);
	}

	std::cout<<"prcloud "<<prcloud->points.size()<<std::endl;


	HexGridData.assign(gridmap.info.width * gridmap.info.height,GridData());


	for(int j=0;j<prcloud->points.size();j++)
	{
		CPoint2d Checkpoint;
		Checkpoint.x = prcloud->points[j].x;
		Checkpoint.y = prcloud->points[j].y;

		CPoint2d Centerpoint;
		Centerpoint = JudgeHexCenpoint(Checkpoint);

		int dx,dy;
		dx = (Centerpoint.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
		dy = (Centerpoint.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
		
        int index = dx + dy * gridmap.info.width;
        int index_up = index + 1;
        int index_down = index - 1;

        if(index<0 || index>(gridmap.info.width * gridmap.info.height) 
        	|| (gridmap.data[index] == 0))
        {
        	// std::cout<<"index not____________________________________"<<std::endl;
        	continue;
        }
        else
        {
        	// std::cout<<"shengyushengyu //////////////////////////////////////////////////"<<std::endl;
        	if(index_up<(gridmap.info.width * gridmap.info.height) && gridmap.data[index_up] == 0)
        	{
        		continue;
        	}

        	if(index_down>0 && gridmap.data[index_down] == 0)
        	{
        		continue;
        	}

        	// std::cout<<"gggggggggggggggbbbb222222222222222bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb "<<std::endl;

        	if(HexGridData[index].top<prcloud->points[j].z)
        	{
        		HexGridData[index].top = prcloud->points[j].z;
        	}

        	if(HexGridData[index].bottom>prcloud->points[j].z)
        	{
        		HexGridData[index].bottom = prcloud->points[j].z;
        	}

        	HexGridData[index].numPoint++;
        	HexGridData[index].Centerp = Centerpoint;

		}
	}


/*	for(int j=0;j<prcloud->points.size();j++)
	{
		CPoint2d Checkpoint;
		Checkpoint.x = prcloud->points[j].x;
		Checkpoint.y = prcloud->points[j].y;

		CPoint2d Centerpoint;
		Centerpoint = JudgeHexCenpoint(Checkpoint);

		int dx,dy;
		dx = (Centerpoint.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
		dy = (Centerpoint.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
		
        int index = dx + dy * gridmap.info.width;
        int index_up = index + 1;
        int index_down = index - 1;

        if(index<0 || index>(gridmap.info.width * gridmap.info.height) 
        	|| (gridmap.data[index] == 0))
        {
        	// std::cout<<"index not____________________________________"<<std::endl;
        	continue;
        }
        else
        {
        	// std::cout<<"shengyushengyu //////////////////////////////////////////////////"<<std::endl;
        	if(index_up<(gridmap.info.width * gridmap.info.height) && gridmap.data[index_up] == 0)
        	{
        		continue;
        	}

        	if(index_down>0 && gridmap.data[index_down] == 0)
        	{
        		continue;
        	}

        	// std::cout<<"gggggggggggggggbbbb222222222222222bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb "<<std::endl;

        	GridData tem;
        	tem.Centerp.x = Centerpoint.x;
        	tem.Centerp.y = Centerpoint.y;
        	tem.top = -10000;
        	tem.bottom = 10000;

        	int findresi = findindexoccpmap(Centerpoint);

        	if(findresi==-1)
        	{
        		tem.top = prcloud->points[j].z;
        		tem.bottom = prcloud->points[j].z;
        		tem.numPoint = 1;
        		HexGridData.push_back(tem);
        	}
        	else
        	{
        		if(HexGridData[findresi].top<prcloud->points[j].z)
        		{
        			HexGridData[findresi].top = prcloud->points[j].z;
        		}

        		if(HexGridData[findresi].bottom>prcloud->points[j].z)
        		{
        			HexGridData[findresi].bottom = prcloud->points[j].z;
        		}

        		HexGridData[findresi].numPoint++;
        	}
		}
	}*/


	// 	CPoint2d Checkpoint;
	// 	Checkpoint.x = cloud->points[i].x;
	// 	Checkpoint.y = cloud->points[i].y;

	// 	CPoint2d Centerpoint;
	// 	Centerpoint = JudgeHexCenpoint(Checkpoint);

	// 	int dx,dy;
	// 	dx = (Centerpoint.x - gridmap.info.origin.position.x)/gridmap.info.resolution;
	// 	dy = (Centerpoint.y - gridmap.info.origin.position.y)/gridmap.info.resolution;
		
 //        int index = dx + dy * gridmap.info.width;
 //        int index_up = index + 1;
 //        int index_down = index - 1;

 //        if(index<0 || index>(gridmap.info.width * gridmap.info.height) 
 //        	|| (gridmap.data[index] == 0))
 //        {
 //        	// std::cout<<"index not____________________________________"<<std::endl;
 //        	continue;
 //        }
 //        else
 //        {
 //        	// std::cout<<"shengyushengyu //////////////////////////////////////////////////"<<std::endl;
 //        	if(index_up<(gridmap.info.width * gridmap.info.height) && gridmap.data[index_up] == 0)
 //        	{
 //        		continue;
 //        	}

 //        	if(index_down>0 && gridmap.data[index_down] == 0)
 //        	{
 //        		continue;
 //        	}

 //        	// std::cout<<"gggggggggggggggbbbb222222222222222bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb "<<std::endl;

 //        	GridData tem;
 //        	tem.Centerp.x = Centerpoint.x;
 //        	tem.Centerp.y = Centerpoint.y;
 //        	tem.top = -10000;
 //        	tem.bottom = 10000;

 //        	int findresi = findindexoccpmap(Centerpoint);

 //        	if(findresi==-1)
 //        	{
 //        		tem.top = cloud->points[i].z;
 //        		tem.bottom = cloud->points[i].z;
 //        		tem.numPoint = 1;
 //        		HexGridData.push_back(tem);
 //        	}
 //        	else
 //        	{
 //        		if(HexGridData[findresi].top<cloud->points[i].z)
 //        		{
 //        			HexGridData[findresi].top = cloud->points[i].z;
 //        		}

 //        		if(HexGridData[findresi].bottom>cloud->points[i].z)
 //        		{
 //        			HexGridData[findresi].bottom = cloud->points[i].z;
 //        		}

 //        		HexGridData[findresi].numPoint++;
 //        	}
 //        }
	// }

	drawallmap();

	float threshold_height = 0.1;

	int obstacle = 250;
	std::cout<<"HexGridData.size() "<<HexGridData.size()<<std::endl;

	for(int i=0;i<HexGridData.size();i++)
	{
		if(HexGridData[i].numPoint==0)
		{
			continue;
		}
		
		if((HexGridData[i].top - HexGridData[i].bottom)>threshold_height)
		{
			drawplogon(HexGridData[i].Centerp.x,HexGridData[i].Centerp.y,Resulution_L,obstacle);
		}

	}

	// std::cout<<"bbbbbb "<<std::endl;
}

nav_msgs::OccupancyGrid hexagon_map::getmap()
{
	return gridmap;
}
