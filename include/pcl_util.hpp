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
#include "util.hpp"

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
#define G_CAP_X -3.220
#define G_CAP_Y 6.535
#define G_CAP_Z 0.321


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

void addSphereCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float x,float y,float z,int r,int g,int b);//指定座標に点を取る
void addSphereCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Coord<float,float> &coord,int r,int g,int b);//指定座標に点を取る
void approximateVoxelGridContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
void approximateVoxelGridContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered,float appvoxValue);
void backgroundSubtraction(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_first, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_latest, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
void radiusOutlierRemovalContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);//passThroughContainerの後でなければならない
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
void passThroughContainerZ(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered,float GT,float LT);
void pointsCut(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered,Coord<float,float> GT,Coord<float,float> LT,bool mode);//true 切り取った方 false あまりの方

//指定座標に点を取る
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

//指定座標に点を取る
void addSphereCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Coord<float,float> &coord,int r,int g,int b)
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
		campusSphere->points[i].x = coord.cartesianX()+value[i][0];
		campusSphere->points[i].y = coord.cartesianY()+value[i][1];
		campusSphere->points[i].z = coord.cartesianZ()+value[i][2];
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

void approximateVoxelGridContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered,float appvoxValue)
{
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> appvox;
	appvox.setInputCloud(cloud);
	appvox.setLeafSize(appvoxValue,appvoxValue,appvoxValue);
	appvox.filter(*cloud_filtered);
}

void backgroundSubtraction(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_first, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_latest, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	//このプログラムのフィルターの根幹。点群を用いた背景差分。
  double resolution = 0.9;
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA> octree (resolution);
	octree.setInputCloud (cloud_first);
	octree.addPointsFromInputCloud ();
	octree.switchBuffers ();
	octree.setInputCloud (cloud_latest);
	octree.addPointsFromInputCloud ();

	std::vector<int> newPointIdxVector;

	octree.getPointIndicesFromNewVoxels (newPointIdxVector);

	cloud_filtered->width = cloud_first->points.size() + cloud_first->points.size();
	cloud_filtered->height = 1;
	cloud_filtered->points.resize (cloud_filtered->width * cloud_filtered->height);

	int n = 0;
	for(size_t i = 0; i < newPointIdxVector.size (); i++)
	{
		cloud_filtered->points[i].x = cloud_latest->points[newPointIdxVector[i]].x;
		cloud_filtered->points[i].y = cloud_latest->points[newPointIdxVector[i]].y;
		cloud_filtered->points[i].z = cloud_latest->points[newPointIdxVector[i]].z;
    cloud_filtered->points[i].r = cloud_latest->points[newPointIdxVector[i]].r;
		cloud_filtered->points[i].g = cloud_latest->points[newPointIdxVector[i]].g;
		cloud_filtered->points[i].b = cloud_latest->points[newPointIdxVector[i]].b;
    cloud_filtered->points[i].a = cloud_latest->points[newPointIdxVector[i]].a;
		n++;
	}
	cloud_filtered->width = n;
	cloud_filtered->height = 1;
	cloud_filtered->points.resize (cloud_filtered->width * cloud_filtered->height);
}

void radiusOutlierRemovalContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> outrem;
  // build the filter
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.015);
  outrem.setMinNeighborsInRadius (2);
  // apply filter
  outrem.filter (*cloud_filtered);
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
	/*float temY = 0;
	float temZ = 0;
	for(int i = 0, y = 0; y < cloud->height; y++) {
		for(int x = 0; x < cloud->width; x++, i++) {
			pcl::PointXYZRGBA &p = cloud->points[i];
			temY = p.y;
			temZ = p.z;
			p.y = temY * cos(angle) + temZ * sin(angle);
			p.z = -temY * sin(angle) + temZ * cos(angle);
		}
	}*/
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform (1,1) = cos (angle);
  transform (1,2) = sin(angle);
  transform (2,1) = -sin (angle);
  transform (2,2) = cos (angle);
  transform (0,3) = 0.0;
  pcl::transformPointCloud (*cloud, *cloud, transform);
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
	/*float temX = 0;
	float temZ = 0;
	for(int i = 0, y = 0; y < cloud->height; y++) {
		for(int x = 0; x < cloud->width; x++, i++) {
			pcl::PointXYZRGBA &p = cloud->points[i];
			temX = p.x;
			temZ = p.z;
			p.x = temX * cos(angle) - temZ * sin(angle);
			p.z = temX * sin(angle) + temZ * cos(angle);
		}
	}*/
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform (0,0) = cos (angle);
  transform (0,2) = -sin(angle);
  transform (2,0) = sin (angle);
  transform (2,2) = cos (angle);
  transform (0,3) = 0.0;
  pcl::transformPointCloud (*cloud, *cloud, transform);
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
	/*float temX = 0;
	float temY = 0;
	for(int i = 0, y = 0; y < cloud->height; y++) {
		for(int x = 0; x < cloud->width; x++, i++) {
			pcl::PointXYZRGBA &p = cloud->points[i];
			temX = p.x;
			temY = p.y;
			p.x = temX * cos(angle) + temY * sin(angle);
			p.y = -temX * sin(angle) + temY * cos(angle);
		}
	}*/
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform (0,0) = cos (angle);
  transform (0,1) = sin(angle);
  transform (1,0) = -sin (angle);
  transform (1,1) = cos (angle);
  transform (0,3) = 0.0;
  pcl::transformPointCloud (*cloud, *cloud, transform);
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
	/*for(int i = 0, y = 0; y < cloud->height; y++) {
		for(int x = 0; x < cloud->width; x++, i++) {
			pcl::PointXYZRGBA &p = cloud->points[i];
			p.x += moveX;
			p.y += moveY;
			p.z += moveZ;
		}
	}*/
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << moveX,moveY,moveZ;
	pcl::transformPointCloud (*cloud, *cloud, transform);
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
	pass.setFilterLimits(1.5, 7.0);
	pass.filter(*cloud_filtered);
}

void passThroughContainerZ(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered,float GT,float LT)
{
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(GT, LT);
	pass.filter(*cloud_filtered);
}

void pointsCut(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered,Coord<float,float> GT,Coord<float,float> LT,bool mode)
{
  //この関数のmode=falseの時の動作は、解決策が見つからなくやむなくしたもののため、使いたくない。

  // build the condition
  if(mode){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr buf(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::ConditionAnd<pcl::PointXYZRGBA>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGBA>);
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGBA> ("x", pcl::ComparisonOps::GT, GT.cartesianX())));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGBA> ("x", pcl::ComparisonOps::LT, LT.cartesianX())));

    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGBA> ("y", pcl::ComparisonOps::GT, GT.cartesianY())));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGBA> ("y", pcl::ComparisonOps::LT, LT.cartesianY())));

    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGBA> ("z", pcl::ComparisonOps::GT, GT.cartesianZ())));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGBA> ("z", pcl::ComparisonOps::LT, LT.cartesianZ())));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZRGBA> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*buf);
    *cloud_filtered = *buf;
  }else{
    *cloud_filtered = *cloud;
    for(size_t i = 0; i < cloud_filtered->points.size (); i++)
    {
      if(GT.cartesianX() <= cloud_filtered->points[i].x && LT.cartesianX() >= cloud_filtered->points[i].x){
        if(GT.cartesianY() <= cloud_filtered->points[i].y && LT.cartesianY() >= cloud_filtered->points[i].y){
          if(GT.cartesianZ() <= cloud_filtered->points[i].z && LT.cartesianZ() >= cloud_filtered->points[i].z){
            cloud_filtered->points[i].x = NULL;
            cloud_filtered->points[i].y = NULL;
            cloud_filtered->points[i].z = NULL;
          }
        }
      }
    }
  }
}

#endif // PCL_UTIL_HPP
