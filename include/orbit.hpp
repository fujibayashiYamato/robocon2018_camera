#ifndef ORBIT_HPP
#define ORBIT_HPP

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

#include "util.hpp"
#include "pcl_util.hpp"
#include "least_squares_method.hpp"

#define CAMERA_SETUP_POS_X 0.0//ロボットから見たカメラの位置
#define CAMERA_SETUP_POS_Y 0.0//ロボットから見たカメラの位置
#define CAMERA_SETUP_POS_Z 0.0//ロボットから見たカメラの位置

#define CAMERA_ANGLE_Y -22.5//degry カメラの設置時のy軸の角度
#define DIST_VALUE 0.14f
#define CLUSTERING_NUM 51
#define SHUTTLE_D 0.14f

class Orbit{
public:
	Orbit();
  void setup();
  void addPointView(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
  void addShuttlePoint(float posX,float posY,float posZ);
  void coeCreate();
  void coordConversion(float *point);//ロボットの座標系に合わせる
  void coordConversion(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);//ロボットの座標系に合わせる
  void filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
  float getPointY();
  float getPointZ(float pointX);
  bool passCheckN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float* Y,float* Z);
	bool passCheckG();
  void setRobotXYZ(Coord<float,float> &coord);
	void setInitRobotXYZ(Coord<float,float> &coord);
	void shuttleDiscovery(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
  void shuttleDiscovery(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);
private:
  Coord<float,float> initRobotPos;//射出時のロボットの位置
  Coord<float,float> robotPos;//現在のロボットの位置
  vector<float> shuttlePointX;
	vector<float> shuttlePointY;
	vector<float> shuttlePointZ;
  Lsm lsm;
	float coeA;
	float coeB;
	float coeC;
  float avePointY;
};

Orbit::Orbit()
{
  coeA = 0.0;
  coeB = 0.0;
  coeC = 0.0;
  avePointY = 0.0;
}

void Orbit::setup()
{
  shuttlePointX.clear();
	shuttlePointY.clear();
	shuttlePointZ.clear();
  coeA = 0.0;
  coeB = 0.0;
  coeC = 0.0;
  avePointY = 0.0;
}

void Orbit::addPointView(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	for(int i = 0;i < shuttlePointX.size();i++){
		addSphereCloud(cloud,shuttlePointX[i],shuttlePointY[i],shuttlePointZ[i],255,0,0);
	}
	for(int i = 0;i < 100;i++){
		addSphereCloud(cloud,float(-0.07 * i),getPointY(),getPointZ(float(-0.07 * i)),0,255,0);
	}

  rotationZ(cloud,-1.0 * robotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける
  moveCloud(cloud,initRobotPos.cartesianX(),initRobotPos.cartesianY(),initRobotPos.cartesianZ());
  //printf("%3.5f:%3.5f:%3.5f\n",initRobotPos.cartesianX(),initRobotPos.cartesianY(),initRobotPos.cartesianZ());
}

void Orbit::addShuttlePoint(float posX,float posY,float posZ)
{
	shuttlePointX.push_back(posX);
	shuttlePointY.push_back(posY);
	shuttlePointZ.push_back(posZ);
}

void Orbit::coeCreate()
{
  lsm.sai(shuttlePointX, shuttlePointZ, shuttlePointX.size());
	coeC = lsm.x[0];
	coeB = lsm.x[1];
	coeA = lsm.x[2];

	float sam = 0;
	for(int i = 0;i < shuttlePointY.size();i++){
		sam += shuttlePointY[i];
	}
	avePointY = sam/shuttlePointY.size();
}

void Orbit::coordConversion(float *point)
{
  rotationX(point,M_PI_2);
  rotationZ(point,-1.0 *M_PI_2);
  rotationY(point,CAMERA_ANGLE_Y * M_PI/180.0);
  moveCloud(point,CAMERA_SETUP_POS_X,CAMERA_SETUP_POS_Y,CAMERA_SETUP_POS_Z);
  //rotationZ(point,-1.0 * robotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける
  //moveCloud(point,robotPos.cartesianX(),robotPos.cartesianY(),robotPos.cartesianZ());
}

void Orbit::coordConversion(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
  rotationX(cloud,M_PI_2);
  rotationZ(cloud,-1.0 * M_PI_2);
  rotationY(cloud,CAMERA_ANGLE_Y * M_PI/180.0);
  moveCloud(cloud,CAMERA_SETUP_POS_X,CAMERA_SETUP_POS_Y,CAMERA_SETUP_POS_Z);
  //rotationZ(cloud,-1.0 * robotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける
  //moveCloud(cloud,robotPos.cartesianX(),robotPos.cartesianY(),robotPos.cartesianZ());
}

void Orbit::filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passThroughCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxelGridConCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	passThroughContainer(cloud,passThroughCloud);
	approximateVoxelGridContainer(passThroughCloud,voxelGridConCloud);

	*cloud_filtered = *voxelGridConCloud;
}

float Orbit::getPointY(){return avePointY;}
float Orbit::getPointZ(float pointX){return coeA * pointX * pointX + coeB * pointX + coeC;}//ロボットの座標系

bool Orbit::passCheckN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float* Y,float* Z)
{
  float shuttlePoint[3] = {0.0};
  float relativeRingPos[3] = {0.0};
  float ringX = 0.0;

  //リングの位置をロボットから見た相対的なものに
	relativeRingPos[0] = N_RING_X - initRobotPos.cartesianX();
  relativeRingPos[1] = N_RING_Y - initRobotPos.cartesianY();
  relativeRingPos[2] = N_RING_Z - initRobotPos.cartesianZ();

  ringX = relativeRingPos[0] / cos(-1.0 * initRobotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける

  //リングのxをつかい、シャトルの位置を予想
  shuttlePoint[0] = ringX;
  shuttlePoint[1] = getPointY();
  shuttlePoint[2] = getPointZ(ringX);

  rotationZ(shuttlePoint,-1.0 * initRobotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける

  *Y = shuttlePoint[1] - relativeRingPos[1];
	*Z = shuttlePoint[2] - relativeRingPos[2];

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr buf(new pcl::PointCloud<pcl::PointXYZRGBA>);
  float campusPoint[3] = {0.0};
  for(int i = 0;i<3;i++){campusPoint[i] = shuttlePoint[i];}
  moveCloud(campusPoint,initRobotPos.cartesianX(),initRobotPos.cartesianY(),initRobotPos.cartesianZ());
  addSphereCloud(buf,campusPoint[0],campusPoint[1],campusPoint[2],0,0,255);
  mergeCloud(cloud,buf,cloud);

  //輪の中にあればtrue,違うのであればfalse
  if(relativeRingPos[1] - (0.4 - SHUTTLE_D/2.0) <= shuttlePoint[1]){
		if(relativeRingPos[1] + (0.4 - SHUTTLE_D/2.0)>= shuttlePoint[1]){
			if(sqrt(pow(0.4 - SHUTTLE_D/2.0,2)-pow(shuttlePoint[1] - relativeRingPos[1],2)) + relativeRingPos[2] >= shuttlePoint[2]){
				if(-1.0 * sqrt(pow(0.4 - SHUTTLE_D/2.0,2)-pow(shuttlePoint[1] - relativeRingPos[1],2)) + relativeRingPos[2] <= shuttlePoint[2]){
					return true;
				}
			}
		}
	}
	return false;
}

bool Orbit::passCheckG()
{

}

void Orbit::setRobotXYZ(Coord<float,float> &coord){this->robotPos = coord;}
void Orbit::setInitRobotXYZ(Coord<float,float> &coord){this->initRobotPos = coord;}

void Orbit::shuttleDiscovery(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
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

    //重心位置の算出
		Eigen::Vector4f xyz_centroid;
	  pcl::compute3DCentroid(*temCloud, xyz_centroid);

    //一番大きい箇所までの直径の算出
		float dist = 0;
		for (int i = 0;i < k;i++) {
			float value = (temCloud->points[i].x - xyz_centroid[0])*(temCloud->points[i].x - xyz_centroid[0]);
			value += (temCloud->points[i].y - xyz_centroid[1])*(temCloud->points[i].y - xyz_centroid[1]);
			value += (temCloud->points[i].z - xyz_centroid[2])*(temCloud->points[i].z - xyz_centroid[2]);
			if(dist < value)dist = value;
		}
		dist = pow(dist,0.5);

    //一番大きい箇所までの直径が指定よりも大きければ消去する
		if(dist >= DIST_VALUE -0.06  && dist <= DIST_VALUE + 0.06){
			float xyz_centroid_buf[3];
			for(int i = 0;i<3;i++){xyz_centroid_buf[i] = xyz_centroid[i];}

      rotationZ(xyz_centroid_buf,(-1.0 * robotPos.angleZ()) - (-1.0 * initRobotPos.angleZ()));
			moveCloud(xyz_centroid_buf,robotPos.cartesianX()-initRobotPos.cartesianX(),robotPos.cartesianY()-initRobotPos.cartesianY(),robotPos.cartesianZ()-initRobotPos.cartesianZ());

      //行ったこと:座標系をロボットに合わる、カメラのy軸の回転を補正、ロボットが回転することによるズレを補正、ロボットが移動することによるズレを補正
      //行っていないこと:カメラのz軸の回転を補正、カメラの原点をロボットの位置に移動
      addShuttlePoint(xyz_centroid_buf[0],xyz_centroid_buf[1],xyz_centroid_buf[2]);
      //printf("%3.5f %3.5f %3.5f\n",xyz_centroid_buf[0],xyz_centroid_buf[1],xyz_centroid_buf[2]);
    }
	}
}

void Orbit::shuttleDiscovery(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
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

    //重心位置の算出
		Eigen::Vector4f xyz_centroid;
	  pcl::compute3DCentroid(*temCloud, xyz_centroid);

    //一番大きい箇所までの直径の算出
		float dist = 0;
		for (int i = 0;i < k;i++) {
			float value = (temCloud->points[i].x - xyz_centroid[0])*(temCloud->points[i].x - xyz_centroid[0]);
			value += (temCloud->points[i].y - xyz_centroid[1])*(temCloud->points[i].y - xyz_centroid[1]);
			value += (temCloud->points[i].z - xyz_centroid[2])*(temCloud->points[i].z - xyz_centroid[2]);
			if(dist < value)dist = value;
		}
		dist = pow(dist,0.5);

    //一番大きい箇所までの直径が指定よりも大きければ消去する
		if(dist >= DIST_VALUE -0.06  && dist <= DIST_VALUE + 0.06){
			float xyz_centroid_buf[3];
			for(int i = 0;i<3;i++){xyz_centroid_buf[i] = xyz_centroid[i];}

      rotationZ(xyz_centroid_buf,(-1.0 * robotPos.angleZ()) - (-1.0 * initRobotPos.angleZ()));
			moveCloud(xyz_centroid_buf,robotPos.cartesianX()-initRobotPos.cartesianX(),robotPos.cartesianY()-initRobotPos.cartesianY(),robotPos.cartesianZ()-initRobotPos.cartesianZ());

      //行ったこと:座標系をロボットに合わる、カメラのy軸の回転を補正、ロボットが回転することによるズレを補正、ロボットが移動することによるズレを補正
      //行っていないこと:カメラのz軸の回転を補正、カメラの原点をロボットの位置に移動
      addShuttlePoint(xyz_centroid_buf[0],xyz_centroid_buf[1],xyz_centroid_buf[2]);
      //printf("%3.5f %3.5f %3.5f\n",xyz_centroid_buf[0],xyz_centroid_buf[1],xyz_centroid_buf[2]);

      //描画するために原点を今のロボットの位置に移動する,またカメラのz軸の回転を補正
      mergeCloud(cloud_filtered,temCloud,cloud_filtered);
      moveCloud(cloud_filtered,robotPos.cartesianX()-initRobotPos.cartesianX(),robotPos.cartesianY()-initRobotPos.cartesianY(),robotPos.cartesianZ()-initRobotPos.cartesianZ());
			addSphereCloud(cloud_filtered,xyz_centroid_buf[0],xyz_centroid_buf[1],xyz_centroid_buf[2],0,255,0);//重心の描写
      rotationZ(cloud_filtered,-1.0 * robotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける
      moveCloud(cloud_filtered,robotPos.cartesianX(),robotPos.cartesianY(),robotPos.cartesianZ());
		}
	}
}

#endif // ORBIT_HPP
