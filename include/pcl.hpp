#ifndef ORBIT_PREDICION_HPP
#define ORBIT_PREDICION_HPP

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

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

#define DIST_VALUE 0.14f
#define APPVOX_VALUE 0.03f
#define CLUSTERING_NUM 51
#define ORIGIN_X 0
#define ORIGIN_Y 0
#define N_RING_X 0.515
#define N_RING_Y 3.275
#define N_RING_Z 2.4
#define G_RING_X 0.515
#define G_RING_Y 6.535
#define G_RING_Z 3.4
#define SHUTTLE_D 0.140

class Orbit{
public:
	Orbit();
	void setup();
	void setCameraPosXYZ(float X,float Y,float Z);
	void setCameraAngleXYZ(float X,float Y,float Z);
	void setInitCameraPosXYZ(float X,float Y,float Z);
	void setInitCameraAngleXYZ(float X,float Y,float Z);
	void addPoint(float X,float Y,float Z);
	void cycle();
	//float getPointX();
	//float getPointY(float pointZ);
	float getPointY();
	float getPointZ(float pointX);
	void addPointView(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
	bool passCheckN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float* Y,float* Z);
	bool passCheckG();

	void passThroughContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
	void approximateVoxelGridContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
	void radiusOutlierRemovalContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
	void statisticalOutlierRemovalContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
	void clusteringContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
	float computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud);
	void mergeCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
	void colorPaint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,int r , int g, int b);
	void addSphereCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float x,float y,float z,int r,int g,int b);
	void addFloorCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float x0,float y0,float x1,float y1,int r,int g,int b);
	void filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
	void rotationX(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle);
	void rotationX(float *point,float angle);
	void rotationY(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle);
	void rotationY(float *point,float angle);
	void rotationZ(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle);
	void rotationZ(float *point,float angle);
	void moveCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float moveX,float moveY,float moveZ);
	void moveCloud(float *point,float moveX,float moveY,float moveZ);
	void coatView(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
private:
	Lsm lsm;
	float coeA;
	float coeB;
	float coeC;
	float cameraPos[3];
	float cameraAngle[3];
	float initCameraPos[3];
	float initCameraAngle[3];
	float N_ringPos[3];
	float G_ringPos[3];
	//float avePointX;
	float avePointY;
	vector<float> pointX;
	vector<float> pointY;
	vector<float> pointZ;
	//int pointNum;
};

Orbit::Orbit(){
	//pointNum = 0;
	coeA = 0;
	coeB = 0;
	coeC = 0;
	avePointY = 0;
	cameraPos[0] = 0;
	cameraPos[1] = 0;
	cameraPos[2] = 0;
	cameraAngle[0] = 0;
	cameraAngle[1] = 0;
	cameraAngle[2] = 0;
	initCameraPos[0] = 0;
	initCameraPos[1] = 0;
	initCameraPos[2] = 0;
	initCameraAngle[0] = 0;
	initCameraAngle[1] = 0;
	initCameraAngle[2] = 0;
	N_ringPos[0] = N_RING_X;
	N_ringPos[1] = N_RING_Y;
	N_ringPos[2] = N_RING_Z;
	G_ringPos[0] = G_RING_X;
	G_ringPos[1] = G_RING_Y;
	G_ringPos[2] = G_RING_Z;
}

void Orbit::setup(){
	coeA = 0;
	coeB = 0;
	coeC = 0;
	avePointY = 0;
	pointX.clear();
	pointY.clear();
	pointZ.clear();
	cameraPos[0] = 0;
	cameraPos[1] = 0;
	cameraPos[2] = 0;
	cameraAngle[0] = 0;
	cameraAngle[1] = 0;
	cameraAngle[2] = 0;
	initCameraPos[0] = 0;
	initCameraPos[1] = 0;
	initCameraPos[2] = 0;
	initCameraAngle[0] = 0;
	initCameraAngle[1] = 0;
	initCameraAngle[2] = 0;
	N_ringPos[0] = N_RING_X;
	N_ringPos[1] = N_RING_Y;
	N_ringPos[2] = N_RING_Z;
	G_ringPos[0] = G_RING_X;
	G_ringPos[1] = G_RING_Y;
	G_ringPos[2] = G_RING_Z;
}

void Orbit::setCameraPosXYZ(float X,float Y,float Z){
	cameraPos[0] = X;
	cameraPos[1] = Y;
	cameraPos[2] = Z;
}

void Orbit::setCameraAngleXYZ(float X,float Y,float Z){
	cameraAngle[0] = X;
	cameraAngle[1] = Y;
	cameraAngle[2] = -1.0 * Z;
}

void Orbit::setInitCameraPosXYZ(float X,float Y,float Z){
	initCameraPos[0] = X;
	initCameraPos[1] = Y;
	initCameraPos[2] = Z;
}

void Orbit::setInitCameraAngleXYZ(float X,float Y,float Z){
	initCameraAngle[0] = X;
	initCameraAngle[1] = Y;
	initCameraAngle[2] = -1.0 * Z;
}

void Orbit::addPoint(float posX,float posY,float posZ){
	pointX.push_back(posX);
	pointY.push_back(posY);
	pointZ.push_back(posZ);
	//printf("x:%2.3f y:%2.3f z:%2.3f\n",point[0],point[1],point[2]);
}

void Orbit::cycle(){
	lsm.sai(pointX, pointZ, pointY.size());
	coeC = lsm.x[0];
	coeB = lsm.x[1];
	coeA = lsm.x[2];
	//printf("a:%f b:%f c:%f\n", coeA, coeB, coeC);

	float sam = 0;
	for(int i = 0;i < pointY.size();i++){
		sam += pointY[i];
	}
	avePointY = sam/pointY.size();
}

//float Orbit::getPointX(){return avePointX;}
//float Orbit::getPointY(float pointZ){return coeA * pointZ * pointZ + coeB * pointZ + coeC;}//カメラの座標系
float Orbit::getPointY(){return avePointY;}
float Orbit::getPointZ(float pointX){return coeA * pointX * pointX + coeB * pointX + coeC;}//ロボットの座標系



void Orbit::addPointView(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
	for(int i = 0;i < pointX.size();i++){
		addSphereCloud(cloud,pointX[i],pointY[i],pointZ[i],255,0,0);
	}
	for(int i = 0;i < 100;i++){
		addSphereCloud(cloud,float(-0.07 * i),getPointY(),getPointZ(float(-0.07 * i)),0,255,0);
	}

	rotationY(cloud,initCameraAngle[1]);
	rotationZ(cloud,initCameraAngle[2]);

	moveCloud(cloud,initCameraPos[0],initCameraPos[1],initCameraPos[2]);
}

bool Orbit::passCheckN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float* Y,float* Z){
	float shuttlePoint[3];
	float ringX = N_RING_X - initCameraPos[0];

	ringX = ringX / cos(cameraAngle[2]);

	bool whileFlg = true;
	float approximaRingX = -7.0;
	int mode = 0;

	while(whileFlg){
		shuttlePoint[0] = approximaRingX;
		shuttlePoint[1] = getPointY();
		shuttlePoint[2] = getPointZ(approximaRingX);
		rotationY(shuttlePoint,cameraAngle[1]);
		//printf("ringX:%f shuttlePoint[0]:%f approximaRingX:%f\n",ringX,shuttlePoint[0],approximaRingX);
		switch(mode)
		{
		case 0:
			if(ringX > shuttlePoint[0]){
				approximaRingX += 1.0;
			}else{
				approximaRingX -= 1.0;
				mode++;
			}
			break;
		case 1:
			if(ringX > shuttlePoint[0]){
				approximaRingX += 0.1;
			}else{
				approximaRingX -= 0.1;
				mode++;
			}
			break;
		case 2:
			if(ringX > shuttlePoint[0]){
				approximaRingX += 0.01;
			}else{
				approximaRingX -= 0.01;
				mode++;
			}
			break;
		case 3:
			if(ringX > shuttlePoint[0]){
				approximaRingX += 0.001;
			}else{
				//approximaRingX -= 0.001;
				whileFlg = false;
			}
			break;
		}
	}
	//rotationY(shuttlePoint,cameraAngle[1]);
	rotationZ(shuttlePoint,cameraAngle[2]);
	//rotationY(shuttlePoint,cameraAngle[1]);*/
	moveCloud(shuttlePoint,initCameraPos[0],initCameraPos[1],initCameraPos[2]);
	addSphereCloud(cloud,shuttlePoint[0],shuttlePoint[1],shuttlePoint[2],0,0,255);

	*Y = shuttlePoint[1] - N_RING_Y;
	*Z = shuttlePoint[2] - N_RING_Z;

	if(N_RING_Y - (0.4 - SHUTTLE_D/2.0) <= shuttlePoint[1]){
		if(N_RING_Y + (0.4 - SHUTTLE_D/2.0)>= shuttlePoint[1]){
			if(sqrt(pow(0.4 - SHUTTLE_D/2.0,2)-pow(shuttlePoint[1] - N_RING_Y,2)) + N_RING_Z >= shuttlePoint[2]){
				if(-1.0 * sqrt(pow(0.4 - SHUTTLE_D/2.0,2)-pow(shuttlePoint[1] - N_RING_Y,2)) + N_RING_Z <= shuttlePoint[2]){
					return true;
				}
			}
		}
	}
	return false;
}
bool Orbit::passCheckG(){return false;}


void Orbit::passThroughContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.5, 0.5);
	pass.filter(*cloud_filtered);

	/*pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-1.0, 0.0);
	pass.filter(*cloud_filtered);*/

	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(1.0, 5.0);
	pass.filter(*cloud_filtered);
}

void Orbit::approximateVoxelGridContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> appvox;
	appvox.setInputCloud(cloud);
	appvox.setLeafSize(APPVOX_VALUE,APPVOX_VALUE,APPVOX_VALUE);
	appvox.filter(*cloud_filtered);
}

void Orbit::radiusOutlierRemovalContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> outrem;
	// build the filter
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(0.1);
	outrem.setMinNeighborsInRadius (10);
	// apply filter
	outrem.filter (*cloud_filtered);
}

void Orbit::statisticalOutlierRemovalContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.00);
	sor.filter (*cloud_filtered);
}

void Orbit::clusteringContainer(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(0.1);//塊の認識の隙間の許容？単位m0.09
	ec.setMinClusterSize(CLUSTERING_NUM-30);//最小個数-45
	ec.setMaxClusterSize(CLUSTERING_NUM+30);//最大個数
	ec.setSearchMethod(tree);//クラスタリングの手法
	ec.setInputCloud(cloud);//点群入力
	ec.extract(cluster_indices);//クラスター情報を引数に出力
	/**ここまででクラスタリングは終了している**/

	int j = 0;

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		int k = 0;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
			temCloud->points.resize (k+1);
			temCloud->points[k].x = cloud->points[*pit].x;
			temCloud->points[k].y = cloud->points[*pit].y;
			temCloud->points[k].z = cloud->points[*pit].z;
			temCloud->points[k].r = 255;
			temCloud->points[k].g = 0;
			temCloud->points[k].b = 0;
		k++;
		}
		Eigen::Vector4f xyz_centroid;
	  pcl::compute3DCentroid(*temCloud, xyz_centroid);//重心を計算
		//printf("%f:%f:%f\n",xyz_centroid[0],xyz_centroid[1],xyz_centroid[2]);

		float dist = 0;
		for (int i = 0;i < k;i++) {
			float value = (temCloud->points[i].x - xyz_centroid[0])*(temCloud->points[i].x - xyz_centroid[0]);
			value += (temCloud->points[i].y - xyz_centroid[1])*(temCloud->points[i].y - xyz_centroid[1]);
			value += (temCloud->points[i].z - xyz_centroid[2])*(temCloud->points[i].z - xyz_centroid[2]);
			if(dist < value)dist = value;
		}
		dist = pow(dist,0.5);
		//printf("%f\n",dist);
		if(dist >= DIST_VALUE -0.06  && dist <= DIST_VALUE + 0.06){
			float xyz_centroid_buf[3];
			for(int i = 0;i < 3;i++){
				xyz_centroid_buf[i] = xyz_centroid[i];
			}
			rotationX(xyz_centroid_buf,M_PI/2.0);
      rotationZ(xyz_centroid_buf,-1.0*M_PI/2.0);
      rotationY(xyz_centroid_buf,cameraAngle[1]);
      rotationZ(xyz_centroid_buf,cameraAngle[2]);
      moveCloud(xyz_centroid_buf,cameraPos[0]-initCameraPos[0],cameraPos[1]-initCameraPos[1],cameraPos[2]-initCameraPos[2]);
			//printf("y=%f\n",cameraPos[1]-initCameraPos[1]);
			addPoint(xyz_centroid_buf[0],xyz_centroid_buf[1],xyz_centroid_buf[2]);//カメラの座標からロボットの座標に変換しながら保存する
			addSphereCloud(temCloud,xyz_centroid_buf[0],xyz_centroid_buf[1],xyz_centroid_buf[2],255,0,0);
			mergeCloud(cloud_filtered,temCloud,cloud_filtered);
		}
		j++;
	}
}

float Orbit::computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
	float res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud (cloud);

	for (size_t i = 0; i < cloud->size (); ++i)
	{
		if (! pcl_isfinite ((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt (sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

void Orbit::mergeCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	*cloud_filtered = *cloud1;//１つ目の点群を統合先に格納する
	*cloud_filtered += *cloud2;//２つ目の点群を統合先に格納する。
}

void Orbit::colorPaint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,int r , int g, int b)
{
	//cloud->points.resize (cloud->width * cloud->height);
	for(int i = 0, y = 0; y < cloud->height; y++) {
		for(int x = 0; x < cloud->width; x++, i++) {
			pcl::PointXYZRGBA &p = cloud->points[i];
			p.r = r;
			p.g = g;
			p.b = b;
		}
	}
}

void Orbit::addSphereCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float x,float y,float z,int r,int g,int b){
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

void Orbit::addFloorCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float x0,float y0,float x1,float y1,int r,int g,int b){
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

void Orbit::filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passThroughCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxelGridConCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	passThroughContainer(cloud,passThroughCloud);
	approximateVoxelGridContainer(passThroughCloud,voxelGridConCloud);

	*cloud_filtered = *voxelGridConCloud;
}

void Orbit::rotationX(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle)
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

void Orbit::rotationX(float *point,float angle){
	float temY = point[1];
	float temZ = point[2];
	point[1] = temY * cos(angle) + temZ * sin(angle);
	point[2] = -temY * sin(angle) + temZ * cos(angle);
}

void Orbit::rotationY(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle)
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

void Orbit::rotationY(float *point,float angle){
	float temX = point[0];
	float temZ = point[2];
	point[0] = temX * cos(angle) - temZ * sin(angle);
	point[2] = temX * sin(angle) + temZ * cos(angle);
}

void Orbit::rotationZ(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float angle)
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

void Orbit::rotationZ(float *point,float angle){
	float temX = point[0];
	float temY = point[1];
	point[0] = temX * cos(angle) + temY * sin(angle);
	point[1] = -temX * sin(angle) + temY * cos(angle);
}

void Orbit::moveCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float moveX,float moveY,float moveZ){
	for(int i = 0, y = 0; y < cloud->height; y++) {
		for(int x = 0; x < cloud->width; x++, i++) {
			pcl::PointXYZRGBA &p = cloud->points[i];
			p.x += moveX;
			p.y += moveY;
			p.z += moveZ;
		}
	}
}

void Orbit::moveCloud(float *point,float moveX,float moveY,float moveZ){
	point[0] += moveX;
	point[1]  += moveY;
	point[2]  += moveZ;
}

void Orbit::coatView(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
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

#endif // ORBIT_PREDICION_HPP
