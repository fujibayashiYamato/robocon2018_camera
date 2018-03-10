#ifndef PCL_UTIL_HPP
#define PCL_UTIL_HPP

#include <math.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/filters/passthrough.h>

#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <pcl/filters/project_inliers.h>

#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
//#include <pcl/filters/uniform_sampling.h>//ver1.8
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <tf/transform_broadcaster.h>

#include "least_squares_method.hpp"

#include <vector>
using namespace std;

#define ORIGIN_X 0
#define ORIGIN_Y 0
#define N_RING_X 0.515
#define N_RING_Y 3.275
#define N_RING_Z 2.4
#define G_RING_X 0.515
#define G_RING_Y 6.535
#define G_RING_Z 3.4

#define TZ1_X 4.565
#define TZ1_Y 3.275
#define TZ1_Z 0.0
#define TZ2_X 6.565
#define TZ2_Y 3.275
#define TZ2_Z 0.0
#define TZ3_X 6.565
#define TZ3_Y 6.535
#define TZ3_Z 0.0

#define APPVOX_VALUE 0.03f

void addFloorCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float x0,float y0,float x1,float y1,int r,int g,int b);
void addSphereCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float x,float y,float z,int r,int g,int b);
void approximateVoxelGridContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
void coatView(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
void rotationX(float *point,float angle);
void rotationX(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle);
void rotationY(float *point,float angle);
void rotationY(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle);
void rotationZ(float *point,float angle);
void rotationZ(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle);
void mergeCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
void moveCloud(float *point,float moveX,float moveY,float moveZ);
void moveCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float moveX,float moveY,float moveZ);
void passThroughContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);

void addFloorCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float x0,float y0,float x1,float y1,int r,int g,int b)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr campusFloor(new pcl::PointCloud<pcl::PointXYZRGBA>);
  campusFloor->points.resize (21*21);
  float pieceX = (x1 - x0)/20.0;
  float pieceY = (y1 - y0)/20.0;
  for(int i = 0;i <= 20; i++){
    for(int j = 0;j <= 20; j++){
      int k = 21 * i + j;
      campusFloor->points[k].x = x0 + i * pieceX;
      campusFloor->points[k].y = y0 + j * pieceY;
      campusFloor->points[k].z = 0;
      campusFloor->points[k].r = r;
      campusFloor->points[k].g = g;
      campusFloor->points[k].b = b;
    }
  }
  mergeCloud(cloud,campusFloor,cloud);
}

void addSphereCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float x,float y,float z,int r,int g,int b)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr campusSphere(new pcl::PointCloud<pcl::PointXYZRGBA>);
	campusSphere->points.resize (7);
	float value[7][3] = {
			{0.0,0.0,0.0},
			{0.0,0.0,0.01},
			{0.0,0.01,0.0},
			{0.01,0.0,0.0},
			{0.0,0.0,-0.01},
			{0.0,-0.01,0.0},
			{-0.01,0.0,0.0}
			};

	for(int i = 0;i < 7;i++){
		campusSphere->points[i].x = x+value[i][0];
		campusSphere->points[i].y = y+value[i][1];
		campusSphere->points[i].z = z+value[i][2];
		campusSphere->points[i].r = r;
		campusSphere->points[i].g = g;
		campusSphere->points[i].b = b;
	}
	mergeCloud(cloud,campusSphere,cloud);
}

void approximateVoxelGridContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> appvox;
	appvox.setInputCloud(cloud);
	appvox.setLeafSize(APPVOX_VALUE,APPVOX_VALUE,APPVOX_VALUE);
	appvox.filter(*cloud_filtered);
}

void coatView(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coatCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	for(int i = 0;i<=100;i++){
		addSphereCloud(coatCloud,N_RING_X+ORIGIN_X,N_RING_Y+ORIGIN_Y,0.02*i,255,0,0);
		addSphereCloud(coatCloud,G_RING_X+ORIGIN_X,G_RING_Y+ORIGIN_Y,0.03*i,255,255,0);
	}
	for(int i = 0;i<=360;i = i + 2){
		addSphereCloud(coatCloud,N_RING_X+ORIGIN_X,(N_RING_Y+ORIGIN_Y)+0.4*cos(i*M_PI/180.0),N_RING_Z+0.4*sin(i*M_PI/180.0),255,0,0);
		addSphereCloud(coatCloud,G_RING_X+ORIGIN_X,(G_RING_Y+ORIGIN_Y)+0.4*cos(i*M_PI/180.0),G_RING_Z+0.4*sin(i*M_PI/180.0),255,255,0);
	}
	for(int i = 0;i<=5;i++){
		for(int j = 0;j<=360;j = j + 2){
			addSphereCloud(coatCloud,(-3.22+ORIGIN_X)+0.6*cos(j*M_PI/180.0),(6.535+ORIGIN_Y)+0.6*sin(j*M_PI/180.0),0.321+0.0158*i,255,255,0);
		}
	}
	for(int i = 1;i<12;i++){
		for(int j = 0;j<=360;j = j + 4){
			addSphereCloud(coatCloud,(-3.22+ORIGIN_X) + 0.05 * i * cos(j*M_PI/180.0),(6.535+ORIGIN_Y) + 0.05 * i * sin(j*M_PI/180.0),0.321,255,255,0);
		}
	}
	addFloorCloud(coatCloud,0.500,0.500,-0.500,-0.500,255,0,0);
	addFloorCloud(coatCloud,-0.500,0.500,-1.300,-0.500,0,255,0);
	addFloorCloud(coatCloud,-1.300,0.500,-2.300,-0.500,255,0,0);
	addFloorCloud(coatCloud,-2.300,0.500,-4.520,-0.500,0,255,0);
	addFloorCloud(coatCloud,-4.520,0.500,-6.520,-0.500,255,255,0);

	addFloorCloud(coatCloud,7.550,0.500,0.500,-0.500,0,255,0);
	addFloorCloud(coatCloud,7.550,1.630,0.500,0.500,0,255,0);
	addFloorCloud(coatCloud,0.500,4.890,-6.520,0.500,0,255,0);

	addFloorCloud(coatCloud,7.550,4.890,2.550,1.660,0,255,0);
	addFloorCloud(coatCloud,7.550,8.150,4.550,4.920,0,255,0);
	mergeCloud(cloud,coatCloud,cloud);
}

void rotationX(float *point,float angle)
{
	float temY = point[1];
	float temZ = point[2];
	point[1] = temY * cos(angle) + temZ * sin(angle);
	point[2] = -temY * sin(angle) + temZ * cos(angle);
}

void rotationX(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle)
{
	float temY = 0;
	float temZ = 0;
	for(int i = 0, y = 0; y < cloud->height; y++) {
		for(int x = 0; x < cloud->width; x++, i++) {
			pcl::PointXYZRGBA &p = cloud->points[i];
			temY = p.y;
			temZ = p.z;
			p.y = temY * cos(angle) + temZ * sin(angle);
			p.z = -temY * sin(angle) + temZ * cos(angle);
		}
	}
}

void rotationY(float *point,float angle)
{
	float temX = point[0];
	float temZ = point[2];
	point[0] = temX * cos(angle) - temZ * sin(angle);
	point[2] = temX * sin(angle) + temZ * cos(angle);
}

void rotationY(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle)
{
	float temX = 0;
	float temZ = 0;
	for(int i = 0, y = 0; y < cloud->height; y++) {
		for(int x = 0; x < cloud->width; x++, i++) {
			pcl::PointXYZRGBA &p = cloud->points[i];
			temX = p.x;
			temZ = p.z;
			p.x = temX * cos(angle) - temZ * sin(angle);
			p.z = temX * sin(angle) + temZ * cos(angle);
		}
	}
}

void rotationZ(float *point,float angle)
{
	float temX = point[0];
	float temY = point[1];
	point[0] = temX * cos(angle) + temY * sin(angle);
	point[1] = -temX * sin(angle) + temY * cos(angle);
}

void rotationZ(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle)
{
	float temX = 0;
	float temY = 0;
	for(int i = 0, y = 0; y < cloud->height; y++) {
		for(int x = 0; x < cloud->width; x++, i++) {
			pcl::PointXYZRGBA &p = cloud->points[i];
			temX = p.x;
			temY = p.y;
			p.x = temX * cos(angle) + temY * sin(angle);
			p.y = -temX * sin(angle) + temY * cos(angle);
		}
	}
}

void mergeCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	*cloud_filtered = *cloud1;//１つ目の点群を統合先に格納する
	*cloud_filtered += *cloud2;//２つ目の点群を統合先に格納する。
}

void moveCloud(float *point,float moveX,float moveY,float moveZ){
	point[0] += moveX;
	point[1] += moveY;
	point[2] += moveZ;
}

void moveCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float moveX,float moveY,float moveZ){
	for(int i = 0, y = 0; y < cloud->height; y++) {
		for(int x = 0; x < cloud->width; x++, i++) {
			pcl::PointXYZRGBA &p = cloud->points[i];
			p.x += moveX;
			p.y += moveY;
			p.z += moveZ;
		}
	}
}

void passThroughContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.5, 0.5);
	pass.filter(*cloud_filtered);

	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-5.0, 0.0);
	pass.filter(*cloud_filtered);

	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(1.0, 7.0);
	pass.filter(*cloud_filtered);
}

#endif // PCL_UTIL_HPP
