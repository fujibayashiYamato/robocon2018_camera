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
#define CAMERA_SETUP_POS_Z 0.12//0.07//ロボットから見たカメラの位置

#define CAMERA_ANGLE_Y -23//-25.75//-22.5//degry カメラの設置時のy軸の角度
#define DIST_VALUE 0.14f
#define CLUSTERING_NUM 51
#define SHUTTLE_D 0.14f

class Orbit{
public:
	Orbit();
  void setup();
  void addPointView(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);//シャトルの軌跡、取得した位置の表示
  void addShuttlePoint(float posX,float posY,float posZ);//引数に入れたシャトルコックの座標を保存する
	int checkArea();//ロボットがどのエリアにいるのかを確認する 0:エリア外 1:TZ1 2:TZ2 3:TZ3
  void coeCreate();//y=ax^2+bx+cのabcを計算し保存する
  void coordConversion(float *point);//ロボットの座標系に合わせる
  void coordConversion(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);//ロボットの座標系に合わせる
	int cupCheck(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);//ゴールデンカップに入ったかか確認 0:カップに入っていない 1:リングに入った 2:TZ3ではない
	void filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);//シャトルを見つける前の動作を軽くするためのフィルター類
	float getGeinAir();//擬似的な空気抵抗を返す
	float getGeinAir(float pointX);//擬似的な空気抵抗を使った時のxの減少値
	float getPointX(float pointZ);//z=ax^2+bx+cのzを求めるバージョン
	float getPointY();//avePointYを返す
  float getPointZ(float pointX);//z=ax^2+bx+c
	int passCheck(float* Y,float* Z);//リングを通ったか確認 0:リングに入っていない 1:リングに入った 2:エリアに入っていない 本番用
	int passCheck(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float* Y,float* Z);//リングを通ったか確認 0:リングに入っていない 1:リングに入った 2:エリアに入っていない 表示用
	void ringCut(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);//ノイズ除去の目的でリングのある位置の点群を消す 相対的な座標用 z軸回転前
	void ringCutNotMove(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);//ノイズ除去の目的でリングのある位置の点群を消す 絶対的な座標用 z軸回転後
	void setCameraSetupPos(Coord<float,float> &coord);//ロボットから見たカメラの位置を保存する
	void setGeinAir(float value);//geinAirを更新する
	void setRobotXYZ(Coord<float,float> &coord);//ロボットの現在位置を保存する
	void setInitRobotXYZ(Coord<float,float> &coord);//ロボットの射出位置を保存する
	void shiftCorrection(float* value);//ロボットが射出位置にいない時ようにズレを補正する
	void shuttleDiscovery(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);//シャトルコックを見つけ座標を保存する 本番用
  void shuttleDiscovery(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered);//シャトルコックを見つけ座標を保存する 表示用
private:
  Coord<float,float> initRobotPos;//射出時のロボットの位置
  Coord<float,float> robotPos;//現在のロボットの位置
	Coord<float,float> cameraSetupPos;//ロボットから見たカメラの位置
  vector<float> shuttlePointX;//シャトルコックの位置 X座標
	vector<float> shuttlePointY;//シャトルコックの位置 Y座標
	vector<float> shuttlePointZ;//シャトルコックの位置 Z座標
	vector<float> shuttleTime;//シャトルコック発見時の時間 プログラム開始時から
  Lsm lsmXZ;//近似曲線の係数を決めるクラス
  float avePointY;//Yの平均値 基本的にまっすぐ飛ぶため
	struct timeval recTime;//時間
	time_t old_sec;//時間
	suseconds_t old_usec;//時間
	float geinAir;//擬似的な空気抵抗
};

Orbit::Orbit()
{
	shuttlePointX.clear();
	shuttlePointY.clear();
	shuttlePointZ.clear();
	shuttleTime.clear();
	geinAir = 0.0;
  avePointY = 0.0;
	cameraSetupPos.cartesianX(CAMERA_SETUP_POS_X);
	cameraSetupPos.cartesianY(CAMERA_SETUP_POS_Y);
	cameraSetupPos.cartesianZ(CAMERA_SETUP_POS_Z);
	cameraSetupPos.angleY(CAMERA_ANGLE_Y * M_PI/180.0);

	gettimeofday(&recTime, NULL);
	old_sec = recTime.tv_sec;
	old_usec = recTime.tv_usec;
}

void Orbit::setup()
{
  shuttlePointX.clear();
	shuttlePointY.clear();
	shuttlePointZ.clear();
	shuttleTime.clear();
  avePointY = 0.0;
}

//シャトルの軌跡、取得した位置の表示
void Orbit::addPointView(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	for(int i = 0;i < shuttlePointX.size();i++){
		addSphereCloud(cloud,shuttlePointX[i],shuttlePointY[i],shuttlePointZ[i],255,0,0);
	}

	for(int i = 0;i < 150;i++){
		addSphereCloud(cloud,float(-0.1 * i),getPointY(),getPointZ(float(-0.1 * i)),0,255,0);
	}

	int area = checkArea();
	if(area == 3){
		float x;
		for(int i = 100;i < 150;i++){
			x = (1.0-sqrtf(1.0-4.0*geinAir*float(-0.1 * i)))/(2.0*geinAir);
			addSphereCloud(cloud,x,getPointY(),getPointZ(float(-0.1 * i)),0,255,0);
		}
	}

  rotationZ(cloud,-1.0 * initRobotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける
  moveCloud(cloud,initRobotPos.cartesianX(),initRobotPos.cartesianY(),initRobotPos.cartesianZ());
  //printf("%3.5f:%3.5f:%3.5f\n",initRobotPos.cartesianX(),initRobotPos.cartesianY(),initRobotPos.cartesianZ());
}

//引数に入れたシャトルコックの座標を保存する
void Orbit::addShuttlePoint(float posX,float posY,float posZ)
{
	shuttlePointX.push_back(posX);
	shuttlePointY.push_back(posY);
	shuttlePointZ.push_back(posZ);
	gettimeofday(&recTime, NULL);
	float timeNow = (recTime.tv_sec - old_sec) + (recTime.tv_usec - old_usec)/1000.0/1000.0;
	shuttleTime.push_back(timeNow);
	printf("addShuttlePoint\t%f\t%f\t%f\n",posX,posZ,timeNow);
}

//ロボットがどのエリアにいるのかを確認する 0:エリア外 1:TZ1 2:TZ2 3:TZ3
int Orbit::checkArea()
{
	if(TZ1_X - 0.985 <= initRobotPos.cartesianX() && TZ1_X + 0.985 >= initRobotPos.cartesianX() && TZ1_Y - 1.615 <= initRobotPos.cartesianY() && TZ1_Y + 1.615 >= initRobotPos.cartesianY()){
		return 1;
	}else if(TZ2_X - 0.985 <= initRobotPos.cartesianX() && TZ2_X + 0.985 >= initRobotPos.cartesianX() && TZ2_Y - 1.615 <= initRobotPos.cartesianY() && TZ2_Y + 1.615 >= initRobotPos.cartesianY()){
		return 2;
	}else if(TZ3_X - 0.985 <= initRobotPos.cartesianX() && TZ3_X + 0.985 >= initRobotPos.cartesianX() && TZ3_Y - 1.615 <= initRobotPos.cartesianY() && TZ3_Y + 1.615 >= initRobotPos.cartesianY()){
		return 3;
	}else{
		return 0;
	}
}

//y=ax^2+bx+cのabcを計算し保存する
void Orbit::coeCreate()
{
  lsmXZ.sai(shuttlePointX, shuttlePointZ, shuttlePointX.size());
	printf("coe\t%f\t%f\t%f\n",lsmXZ.x[2],lsmXZ.x[1],lsmXZ.x[0]);

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
  rotationY(point,cameraSetupPos.angleY());
  moveCloud(point,cameraSetupPos.cartesianX(),cameraSetupPos.cartesianY(),cameraSetupPos.cartesianZ());
  //rotationZ(point,-1.0 * robotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける
  //moveCloud(point,robotPos.cartesianX(),robotPos.cartesianY(),robotPos.cartesianZ());
}

void Orbit::coordConversion(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
  rotationX(cloud,M_PI_2);
  rotationZ(cloud,-1.0 * M_PI_2);
  rotationY(cloud,cameraSetupPos.angleY());
  moveCloud(cloud,cameraSetupPos.cartesianX(),cameraSetupPos.cartesianY(),cameraSetupPos.cartesianZ());
  //rotationZ(cloud,-1.0 * robotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける
  //moveCloud(cloud,robotPos.cartesianX(),robotPos.cartesianY(),robotPos.cartesianZ());
}

//ゴールデンカップに入ったかか確認 0:カップに入っていない 1:リングに入った 2:TZ3ではない
int Orbit::cupCheck(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	float shuttlePoint[3] = {0.0};
	float relativeCupPos[3] = {0.0};
	float cupZ = 0.0;

	int area = checkArea();
	if(area == 3){
		//カップの位置をロボットから見た相対的なものに
		relativeCupPos[0] = G_CAP_X - initRobotPos.cartesianX();
	  relativeCupPos[1] = G_CAP_Y - initRobotPos.cartesianY();
	  relativeCupPos[2] = (G_CAP_Z + 0.079) - initRobotPos.cartesianZ();//カップの縁の高さ

		cupZ = relativeCupPos[2];

		//カップのxをつかい、シャトルの位置を予想
		float x;
		x = (1.0-sqrtf(1.0-4.0*geinAir*getPointX(cupZ)))/(2.0*geinAir);
	  shuttlePoint[0] = x;
	  shuttlePoint[1] = getPointY();
	  shuttlePoint[2] = cupZ;

		printf("cup:\t%f\t%f\n",shuttlePoint[0]-relativeCupPos[0],shuttlePoint[1]-relativeCupPos[1]);

	  rotationZ(shuttlePoint,-1.0 * initRobotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける

		//描画するために角度と位置変更
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr buf(new pcl::PointCloud<pcl::PointXYZRGBA>);
	  float campusPoint[3] = {0.0};
	  for(int i = 0;i<3;i++){campusPoint[i] = shuttlePoint[i];}
	  moveCloud(campusPoint,initRobotPos.cartesianX(),initRobotPos.cartesianY(),initRobotPos.cartesianZ());
	  addSphereCloud(buf,campusPoint[0],campusPoint[1],campusPoint[2],0,0,255);

	  mergeCloud(cloud,buf,cloud);

		//カップの中にあればtrue,違うのであればfalse
	  if(relativeCupPos[0] - (0.6 - SHUTTLE_D/2.0) <= shuttlePoint[0]){
			if(relativeCupPos[0] + (0.6 - SHUTTLE_D/2.0) >= shuttlePoint[0]){
				if(sqrt(pow(0.6 - SHUTTLE_D/2.0,2)-pow(shuttlePoint[0] - relativeCupPos[0],2)) + relativeCupPos[1] >= shuttlePoint[1]){
					if(-1.0 * sqrt(pow(0.6 - SHUTTLE_D/2.0,2)-pow(shuttlePoint[0] - relativeCupPos[0],2)) + relativeCupPos[1] <= shuttlePoint[1]){
						return 1;
					}
				}
			}
		}
		return 0;
	}else{
		return 2;
	}
}

//シャトルを見つける前の動作を軽くするためのフィルター類
void Orbit::filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passThroughCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxelGridConCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr radiusOutlierRemovalCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	passThroughContainer(cloud,passThroughCloud);
	approximateVoxelGridContainer(passThroughCloud,voxelGridConCloud);
	//radiusOutlierRemovalContainer(voxelGridConCloud,radiusOutlierRemovalCloud);

	*cloud_filtered = *voxelGridConCloud;
}

//擬似的な空気抵抗を返す
float Orbit::getGeinAir(){return geinAir;}
//擬似的な空気抵抗を使った時のxの減少値
float Orbit::getGeinAir(float pointX){return geinAir * pointX * pointX;}
//z=ax^2+bx+cのzを求めるバージョン
float Orbit::getPointX(float pointZ){return (-lsmXZ.x[1] + sqrtf((lsmXZ.x[1]*lsmXZ.x[1])-4.0*lsmXZ.x[2]*(lsmXZ.x[0]-pointZ)))/(2.0*lsmXZ.x[2]);}//ロボットの座標系
//avePointYを返す
float Orbit::getPointY(){return avePointY;}
//z=ax^2+bx+c
float Orbit::getPointZ(float pointX){return lsmXZ.x[2] * pointX * pointX + lsmXZ.x[1] * pointX + lsmXZ.x[0];}//ロボットの座標系

//リングを通ったか確認 0:リングに入っていない 1:リングに入った 2:エリアに入っていない 本番用
int Orbit::passCheck(float* Y,float* Z){
	int area = checkArea();
	if(lsmXZ.x[2] >= 0)return 2;
	if(area != 0){
		float shuttlePoint[3] = {0.0};
	  float relativeRingPos[3] = {0.0};
	  float ringX = 0.0;

	  //リングの位置をロボットから見た相対的なものに
		if(area == 1 || area == 2){
			relativeRingPos[0] = N_RING_X - initRobotPos.cartesianX();
		  relativeRingPos[1] = N_RING_Y - initRobotPos.cartesianY();
		  relativeRingPos[2] = N_RING_Z - initRobotPos.cartesianZ();
		}else if(area == 3){
			relativeRingPos[0] = G_RING_X - initRobotPos.cartesianX();
		  relativeRingPos[1] = G_RING_Y - initRobotPos.cartesianY();
		  relativeRingPos[2] = G_RING_Z - initRobotPos.cartesianZ();
		}
	  ringX = relativeRingPos[0] / cos(-1.0 * initRobotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける

	  //リングのxをつかい、シャトルの位置を予想
	  shuttlePoint[0] = ringX;
	  shuttlePoint[1] = getPointY();
	  shuttlePoint[2] = getPointZ(ringX);

	  rotationZ(shuttlePoint,-1.0 * initRobotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける

	  *Y = shuttlePoint[1] - relativeRingPos[1];
		*Z = shuttlePoint[2] - relativeRingPos[2];

	  //輪の中にあればtrue,違うのであればfalse
	  if(relativeRingPos[1] - (0.4 - SHUTTLE_D/2.0) <= shuttlePoint[1]){
			if(relativeRingPos[1] + (0.4 - SHUTTLE_D/2.0)>= shuttlePoint[1]){
				if(sqrt(pow(0.4 - SHUTTLE_D/2.0,2)-pow(shuttlePoint[1] - relativeRingPos[1],2)) + relativeRingPos[2] >= shuttlePoint[2]){
					if(-1.0 * sqrt(pow(0.4 - SHUTTLE_D/2.0,2)-pow(shuttlePoint[1] - relativeRingPos[1],2)) + relativeRingPos[2] <= shuttlePoint[2]){
						return 1;
					}
				}
			}
		}
		return 0;
	}else{
		return 2;
	}
}

//リングを通ったか確認 0:リングに入っていない 1:リングに入った 2:エリアに入っていない 表示用
int Orbit::passCheck(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float* Y,float* Z){
	int area = checkArea();
	if(lsmXZ.x[2] >= 0)return 2;
	if(area != 0){
		float shuttlePoint[3] = {0.0};
	  float relativeRingPos[3] = {0.0};
	  float ringX = 0.0;

	  //リングの位置をロボットから見た相対的なものに
		if(area == 1 || area == 2){
			relativeRingPos[0] = N_RING_X - initRobotPos.cartesianX();
		  relativeRingPos[1] = N_RING_Y - initRobotPos.cartesianY();
		  relativeRingPos[2] = N_RING_Z - initRobotPos.cartesianZ();
		}else if(area == 3){
			relativeRingPos[0] = G_RING_X - initRobotPos.cartesianX();
		  relativeRingPos[1] = G_RING_Y - initRobotPos.cartesianY();
		  relativeRingPos[2] = G_RING_Z - initRobotPos.cartesianZ();
		}
	  ringX = relativeRingPos[0] / cos(-1.0 * initRobotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける

	  //リングのxをつかい、シャトルの位置を予想
	  shuttlePoint[0] = ringX;
	  shuttlePoint[1] = getPointY();
	  shuttlePoint[2] = getPointZ(ringX);

	  rotationZ(shuttlePoint,-1.0 * initRobotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける

	  *Y = shuttlePoint[1] - relativeRingPos[1];
		*Z = shuttlePoint[2] - relativeRingPos[2];

		//描画するために角度と位置変更
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr buf(new pcl::PointCloud<pcl::PointXYZRGBA>);
	  float campusPoint[3] = {0.0};
	  for(int i = 0;i<3;i++){campusPoint[i] = shuttlePoint[i];}
	  moveCloud(campusPoint,initRobotPos.cartesianX(),initRobotPos.cartesianY(),initRobotPos.cartesianZ());
	  addSphereCloud(buf,campusPoint[0],campusPoint[1],campusPoint[2],0,0,255);

		if(area == 1 || area == 2){addSphereCloud(buf,N_RING_X,N_RING_Y,N_RING_Z,255,0,0);}
		else if(area == 3){addSphereCloud(buf,G_RING_X,G_RING_Y,G_RING_Z,255,255,0);}

	  mergeCloud(cloud,buf,cloud);

	  //輪の中にあればtrue,違うのであればfalse
	  if(relativeRingPos[1] - (0.4 - SHUTTLE_D/2.0) <= shuttlePoint[1]){
			if(relativeRingPos[1] + (0.4 - SHUTTLE_D/2.0)>= shuttlePoint[1]){
				if(sqrt(pow(0.4 - SHUTTLE_D/2.0,2)-pow(shuttlePoint[1] - relativeRingPos[1],2)) + relativeRingPos[2] >= shuttlePoint[2]){
					if(-1.0 * sqrt(pow(0.4 - SHUTTLE_D/2.0,2)-pow(shuttlePoint[1] - relativeRingPos[1],2)) + relativeRingPos[2] <= shuttlePoint[2]){
						return 1;
					}
				}
			}
		}
		return 0;
	}else{
		return 2;
	}
}

//ノイズ除去の目的でリングのある位置の点群を消す 相対的な座標用 z軸回転前
void Orbit::ringCut(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr buff(new pcl::PointCloud<pcl::PointXYZRGBA>);

	rotationZ(cloud,-1.0 * robotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける
  moveCloud(cloud,robotPos.cartesianX(),robotPos.cartesianY(),robotPos.cartesianZ());


	Coord<float,float> GT_N;
	GT_N.cartesianX(N_RING_X - 0.3);
	GT_N.cartesianY(N_RING_Y - 0.6);
	GT_N.cartesianZ(N_RING_Z - 2.5);

	Coord<float,float> LT_N;
	LT_N.cartesianX(N_RING_X + 0.3);
	LT_N.cartesianY(N_RING_Y + 0.6);
	LT_N.cartesianZ(N_RING_Z + 0.8);

	Coord<float,float> GT_G;
	GT_G.cartesianX(G_RING_X - 0.3);
	GT_G.cartesianY(G_RING_Y - 0.6);
	GT_G.cartesianZ(G_RING_Z - 3.5);

	Coord<float,float> LT_G;
	LT_G.cartesianX(G_RING_X + 0.3);
	LT_G.cartesianY(G_RING_Y + 0.6);
	LT_G.cartesianZ(G_RING_Z + 0.6);

	pointsCut(cloud,buff,GT_N,LT_N,false);
	pointsCut(buff,cloud_filtered,GT_G,LT_G,false);

	moveCloud(cloud_filtered,-1.0 * robotPos.cartesianX(),-1.0 * robotPos.cartesianY(),-1.0 * robotPos.cartesianZ());
	rotationZ(cloud_filtered,robotPos.angleZ());

}

//ノイズ除去の目的でリングのある位置の点群を消す 絶対的な座標用 z軸回転後
void Orbit::ringCutNotMove(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr buff(new pcl::PointCloud<pcl::PointXYZRGBA>);

	Coord<float,float> GT_N;
	GT_N.cartesianX(N_RING_X - 0.3);
	GT_N.cartesianY(N_RING_Y - 0.6);
	GT_N.cartesianZ(N_RING_Z - 2.5);

	Coord<float,float> LT_N;
	LT_N.cartesianX(N_RING_X + 0.3);
	LT_N.cartesianY(N_RING_Y + 0.6);
	LT_N.cartesianZ(N_RING_Z + 0.8);

	Coord<float,float> GT_G;
	GT_G.cartesianX(G_RING_X - 0.3);
	GT_G.cartesianY(G_RING_Y - 0.6);
	GT_G.cartesianZ(G_RING_Z - 3.5);

	Coord<float,float> LT_G;
	LT_G.cartesianX(G_RING_X + 0.3);
	LT_G.cartesianY(G_RING_Y + 0.6);
	LT_G.cartesianZ(G_RING_Z + 0.6);

	pointsCut(cloud,buff,GT_N,LT_N,false);
	pointsCut(buff,cloud_filtered,GT_G,LT_G,false);

}

//ロボットから見たカメラの位置を保存する
void Orbit::setCameraSetupPos(Coord<float,float> &coord){this->cameraSetupPos = coord;}
//geinAirを更新する
void Orbit::setGeinAir(float value){this->geinAir = value;}
//ロボットの現在位置を保存する
void Orbit::setRobotXYZ(Coord<float,float> &coord){this->robotPos = coord;}
//ロボットの射出位置を保存する
void Orbit::setInitRobotXYZ(Coord<float,float> &coord){this->initRobotPos = coord;}

//ロボットが射出位置にいない時ようにズレを補正する
void Orbit::shiftCorrection(float* value)
{
	rotationZ(value,(-1.0 * robotPos.angleZ()) - (-1.0 * initRobotPos.angleZ()));
	moveCloud(value,robotPos.cartesianX()-initRobotPos.cartesianX(),robotPos.cartesianY()-initRobotPos.cartesianY(),robotPos.cartesianZ()-initRobotPos.cartesianZ());
}

//シャトルコックを見つけ座標を保存する 本番用
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

			//ズレ補正
			shiftCorrection(xyz_centroid_buf);
      //rotationZ(xyz_centroid_buf,(-1.0 * robotPos.angleZ()) - (-1.0 * initRobotPos.angleZ()));
			//moveCloud(xyz_centroid_buf,robotPos.cartesianX()-initRobotPos.cartesianX(),robotPos.cartesianY()-initRobotPos.cartesianY(),robotPos.cartesianZ()-initRobotPos.cartesianZ());

      //行ったこと:座標系をロボットに合わる、カメラのy軸の回転を補正、ロボットが回転することによるズレを補正、ロボットが移動することによるズレを補正
      //行っていないこと:カメラのz軸の回転を補正、カメラの原点をロボットの位置に移動
      addShuttlePoint(xyz_centroid_buf[0],xyz_centroid_buf[1],xyz_centroid_buf[2]);
      //printf("%3.5f %3.5f %3.5f\n",xyz_centroid_buf[0],xyz_centroid_buf[1],xyz_centroid_buf[2]);
    }
	}
}

//シャトルコックを見つけ座標を保存する 表示用
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

			//ズレ補正
			shiftCorrection(xyz_centroid_buf);
      //rotationZ(xyz_centroid_buf,(-1.0 * robotPos.angleZ()) - (-1.0 * initRobotPos.angleZ()));
			//moveCloud(xyz_centroid_buf,robotPos.cartesianX()-initRobotPos.cartesianX(),robotPos.cartesianY()-initRobotPos.cartesianY(),robotPos.cartesianZ()-initRobotPos.cartesianZ());

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
