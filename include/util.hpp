#pragma once
#include "util.h"
#include <complex>
/*******************************************
 * utilplus ver1.1 2015/2/15
 * This is a program made for mathematically calculation.
 * This is the first kitrp's microcomputer program that uses classes!
 * [Dependency]
 * util 1.1+
 *
 * [Note]
 *
 * [Author]
 * Tomoki Nagatani
 *
 * [Change history]
 * ver1.1 2016/ 2/ 15 Add PhysicalObject,Change,UpEdge,DownEdge,quadraticEquationSolver,cubicEquationSolver. RingBuffer fatal bug fix.RingBuffer add class support.
 * ver1.0 2015/ 2/ 6 Add revolution. Add operator overload for Coord. Add sign.Dependency update to util 1.1+.
 * ver0.2 2015/ 1/16 Addition of add functions.
 * ver0.1 2015/ 1/11 The first version.Constuctor ,read and write members available.
 *
 ******************************************/

template <typename lType,typename aType=lType>
class Coord{
protected:
    lType cartesianXData;
    lType cartesianYData;
    lType cartesianZData;
    aType angleXData;
    aType angleYData;
    aType angleZData;

public:
    Coord(){
        cartesianXData=0;
        cartesianYData=0;
        cartesianZData=0;
        angleXData=0;
        angleYData=0;
        angleZData=0;
    };
    Coord(Coord const &coord){
        cartesianXData=coord.cartesianXData;
        cartesianYData=coord.cartesianYData;
        cartesianZData=coord.cartesianZData;
        angleXData=coord.angleXData;
        angleYData=coord.angleYData;
        angleZData=coord.angleZData;
    }
    Coord(lType cx,lType cy,lType cz){
        cartesianXData=cx;
        cartesianYData=cy;
        cartesianZData=cz;
        angleXData=0;
        angleYData=0;
        angleZData=0;
    }
    Coord(lType cx,lType cy,lType cz,aType ex,aType ey,aType ez){
        cartesianXData=cx;
        cartesianYData=cy;
        cartesianZData=cz;
        angleXData=ex;
        angleYData=ey;
        angleZData=ez;
    }

    void cartesian(lType cx,lType cy,lType cz){
        cartesianXData=cx;
        cartesianYData=cy;
        cartesianZData=cz;
    }
    lType cartesianX(){
        return cartesianXData;
    }
    lType cartesianY(){
        return cartesianYData;
    }
    lType cartesianZ(){
        return cartesianZData;
    }
    void cartesianX(lType x){
        cartesianXData=x;
    }
    void cartesianY(lType y){
        cartesianYData=y;
    }
    void cartesianZ(lType z){
        cartesianZData=z;
    }
    void cartesianXAdd(lType x){
        cartesianXData=x;
    }
    void cartesianYAdd(lType y){
        cartesianYData=y;
    }
    void cartesianZAdd(lType z){
        cartesianZData=z;
    }

    void polar(lType radius,aType theta,aType phi){
        lType xy=0;
        if(theta==0){
            cartesianXData=0;
            cartesianYData=0;
            cartesianZData=radius;
            return;
        }
        cartesianZData=radius*cos(theta);
        xy=radius*sin(theta);
        cartesianXData=xy*cos(phi);
        cartesianYData=xy*sin(phi);
    }
    lType polarRadius(){
        lType radius;
        //    float theta;
        //    float phi;
        radius=sqrt(pow(cartesianXData,2)+pow(cartesianYData,2)+pow(cartesianZData,2));
        //    phi=atan2(cartesianYData,cartesianXData);
        //    if(radius==0) theta=0;
        //    else theta=acos(cartesianZData/radius);
        return radius;
    }
    aType polarTheta(){
        lType radius;
        aType theta;
        //float phi;
        radius=sqrt(pow(cartesianXData,2)+pow(cartesianYData,2)+pow(cartesianZData,2));
        //phi=atan2(cartesianYData,cartesianXData);
        if(radius==0) theta=0;
        else theta=acos(cartesianZData/radius);
        return theta;
    }
    aType polarPhi(){
        //	float radius;
        //	float theta;
        aType phi;
        //    float radius;
        //float theta;
        //radius=sqrt(pow(cartesianXData,2)+pow(cartesianYData,2)+pow(cartesianZData,2));
        phi=atan2(cartesianYData,cartesianXData);
        //if(radius==0) theta=0;
        //else theta=acos(cartesianZData/radius);
        return phi;
    }
    void polarRadius(lType radiusData){
        lType radius;
        aType theta;
        aType phi;
        radius=sqrt(pow(cartesianXData,2)+pow(cartesianYData,2)+pow(cartesianZData,2));
        phi=atan2(cartesianYData,cartesianXData);
        if(radius==0) theta=0;
        else theta=acos(cartesianZData/radius);
        radius=radiusData;
        Coord::polar(radius,theta,phi);
    }
    void polarTheta(aType thetaData){
        lType radius;
        aType theta;
        aType phi;
        radius=sqrt(pow(cartesianXData,2)+pow(cartesianYData,2)+pow(cartesianZData,2));
        phi=atan2(cartesianYData,cartesianXData);
        if(radius==0) theta=0;
        else theta=acos(cartesianZData/radius);
        theta=thetaData;
        Coord::polar(radius,theta,phi);
    }
    void polarPhi(aType phiData){
        lType radius;
        aType theta;
        aType phi;
        radius=sqrt(pow(cartesianXData,2)+pow(cartesianYData,2)+pow(cartesianZData,2));
        phi=atan2(cartesianYData,cartesianXData);
        if(radius==0) theta=0;
        else theta=acos(cartesianZData/radius);
        phi=phiData;
        Coord::polar(radius,theta,phi);
    }
    void polarRadiusAdd(lType radius){
        polarRadius(polarRadius()+radius);
    }
    void polarThetaAdd(aType theta){
        polarTheta(polarTheta()+theta);
    }
    void polarPhiAdd(aType phiArg){
        polarPhi(polarPhi()+phiArg);
    }

    void angle(aType x,aType y,aType z){
        angleXData=x;
        angleYData=y;
        angleZData=z;
    }
    aType angleX(){
        return angleXData;
    }
    aType angleY(){
        return angleYData;
    }
    aType angleZ(){
        return angleZData;
    }
    void angleX(aType x){
        angleXData=x;
    }
    void angleY(aType y){
        angleYData=y;
    }
    void angleZ(aType z){
        angleZData=z;
    }
    void angleXAdd(aType x){
        angleXData+=x;
    }
    void angleYAdd(aType y){
        angleYData+=y;
    }
    void angleZAdd(aType z){
        angleZData+=z;
    }

    lType x(){
        return cartesianX();
    }
    lType y(){
        return cartesianY();
    }
    lType z(){
        return cartesianZ();
    }
    lType radius(){
        return polarRadius();
    }
    aType theta(){
        return polarTheta();
    }
    aType phi(){
        return polarPhi();
    }

    aType ax(){
        return angleX();
    }
    aType ay(){
        return angleY();
    }
    aType az(){
        return angleZ();
    }

    aType roll(){
        return angleX();
    }
    aType pitch(){
        return angleX();
    }
    aType yaw(){
        return angleX();
    }

    void x(lType value){
        cartesianX(value);
    }
    void y(lType value){
        cartesianY(value);
    }
    void z(lType value){
        cartesianZ(value);
    }
    void radius(lType value){
        polarRadius(value);
    }
    void theta(aType value){
        polarTheta(value);
    }
    void phi(aType value){
        polarPhi(value);
    }

    void ax(aType value){
        angleX(value);
    }
    void ay(aType value){
        angleY(value);
    }
    void az(aType value){
        angleZ(value);
    }

    void revolutionXY(aType angle){
        lType cartesianXSub = cartesianXData;
        lType cartesianYSub = cartesianYData;
        angleZData = angleZData + angle;
        //	int a = cos(angle);
        //	std::cout << std::endl << " angle : " << a << std::endl;
        cartesianXData = cartesianXSub * cos(angle) + (-1 * cartesianYSub * sin(angle));
        cartesianYData = cartesianXSub * sin(angle) + cartesianYSub * cos(angle);
    }
    void revolutionYZ(aType angle){
        lType cartesianYSub = cartesianYData;
        lType cartesianZSub = cartesianZData;
        angleXData = angleXData + angle;
        cartesianYData = cartesianYSub * cos(angle) - cartesianZSub * sin(angle);
        cartesianZData = cartesianYSub * sin(angle) + cartesianZSub * cos(angle);
    }
    void revolutionZX(aType angle){
        lType cartesianXSub = cartesianXData;
        lType cartesianZSub = cartesianZData;
        angleYData = angleYData + angle;
        cartesianZData = cartesianZSub * cos(angle) - cartesianXSub * sin(angle);
        cartesianXData = cartesianZSub * sin(angle) + cartesianXSub * cos(angle);
    }

    void revolutionX(aType angle){
        lType cartesianXSub = cartesianXData;
        lType cartesianYSub = cartesianYData;
        lType cartesianZSub = cartesianZData;
        angleXData = angleXData + angle;
        cartesianXData = cartesianXSub;
        cartesianYData = cartesianYSub * cos(angle) - cartesianZSub * sin(angle);
        cartesianZData = cartesianYSub * sin(angle) + cartesianZSub * cos(angle);
    }
    void revolutionY(aType angle){
        lType cartesianXSub = cartesianXData;
        lType cartesianYSub = cartesianYData;
        lType cartesianZSub = cartesianZData;
        angleYData = angleYData + angle;
        cartesianXData = cartesianXSub * cos(angle) + cartesianZSub * sin(angle);
        cartesianYData = cartesianYSub;
        cartesianZData = (-1 * cartesianYSub * sin(angle)) + cartesianZSub * cos(angle);
    }
    void revolutionZ(aType angle){
        lType cartesianXSub = cartesianXData;
        lType cartesianYSub = cartesianYData;
        lType cartesianZSub = cartesianZData;
        angleZData = angleZData + angle;
        cartesianXData = cartesianXSub * cos(angle) - cartesianYSub * sin(angle);
        cartesianYData = cartesianXSub * sin(angle) + cartesianYSub * cos(angle);
        cartesianZData = cartesianZSub;
    }

    void revolutionXYPosition(aType angle){
        lType cartesianXSub = cartesianXData;
        lType cartesianYSub = cartesianYData;
        //angleZData = angleZData + angle;
        //	int a = cos(angle);
        //	std::cout << std::endl << " angle : " << a << std::endl;
        cartesianXData = cartesianXSub * cos(angle) + (-1 * cartesianYSub * sin(angle));
        cartesianYData = cartesianXSub * sin(angle) + cartesianYSub * cos(angle);
    }
    void revolutionYZPosition(aType angle){
        lType cartesianYSub = cartesianYData;
        lType cartesianZSub = cartesianZData;
        //angleXData = angleXData + angle;
        cartesianYData = cartesianYSub * cos(angle) - cartesianZSub * sin(angle);
        cartesianZData = cartesianYSub * sin(angle) + cartesianZSub * cos(angle);
    }
    void revolutionZXPosition(aType angle){
        lType cartesianXSub = cartesianXData;
        lType cartesianZSub = cartesianZData;
        //angleYData = angleYData + angle;
        cartesianZData = cartesianZSub * cos(angle) - cartesianXSub * sin(angle);
        cartesianXData = cartesianZSub * sin(angle) + cartesianXSub * cos(angle);
    }

    void revolutionXPosition(aType angle){
        lType cartesianXSub = cartesianXData;
        lType cartesianYSub = cartesianYData;
        lType cartesianZSub = cartesianZData;
        //angleXData = angleXData + angle;
        cartesianXData = cartesianXSub;
        cartesianYData = cartesianYSub * cos(angle) - cartesianZSub * sin(angle);
        cartesianZData = cartesianYSub * sin(angle) + cartesianZSub * cos(angle);
    }
    void revolutionYPosition(aType angle){
        lType cartesianXSub = cartesianXData;
        lType cartesianYSub = cartesianYData;
        lType cartesianZSub = cartesianZData;
        //angleYData = angleYData + angle;
        cartesianXData = cartesianXSub * cos(angle) + cartesianZSub * sin(angle);
        cartesianYData = cartesianYSub;
        cartesianZData = (-1 * cartesianYSub * sin(angle)) + cartesianZSub * cos(angle);
    }
    void revolutionZPosition(aType angle){
        lType cartesianXSub = cartesianXData;
        lType cartesianYSub = cartesianYData;
        lType cartesianZSub = cartesianZData;
        //angleZData = angleZData + angle;
        cartesianXData = cartesianXSub * cos(angle) - cartesianYSub * sin(angle);
        cartesianYData = cartesianXSub * sin(angle) + cartesianYSub * cos(angle);
        cartesianZData = cartesianZSub;
    }

    void revolutionEuler(aType a, aType b, aType c){
        lType cartesianXSub = cartesianXData;
        lType cartesianYSub = cartesianYData;
        lType cartesianZSub = cartesianZData;
        cartesianXData = cartesianXSub * (cos(a) * cos(c) - sin(a) * cos(b) * sin(c)) + cartesianYSub * (cos(a) * sin(c) + sin(a) * cos(b) * cos(c)) + cartesianZSub * (sin(a) * sin(b));
        cartesianYData = cartesianXSub * (-1 * sin(a) * cos(c) - cos(a) * cos(b) * sin(c)) + cartesianYData * (-1 * sin(a) * sin(c) + cos(a) * cos(b) * cos(c)) + cartesianZData * (cos(a) * sin(b));
        cartesianZData = cartesianXSub * (sin(b) * sin(c)) + cartesianYSub * (-1 * sin(b) * cos(c)) + cartesianZSub * cos(b);

    }
    void revolutionEulerinverse(aType a, aType b, aType c){
        lType cartesianXSub = cartesianXData;
        lType cartesianYSub = cartesianYData;
        lType cartesianZSub = cartesianZData;
        cartesianXData = cartesianXSub * (cos(a) * cos(c) - sin(a) * cos(b) * sin(c)) + cartesianYSub * (-1 * sin(a) * cos(c) + -1 * cos(a) * cos(b) * sin(c)) + cartesianZSub * (sin(b) * sin(c));
        cartesianYData = cartesianXSub * (cos(a) * sin(c) + sin(a) * cos(b) * cos(c)) + cartesianYSub * (-1 * sin(a) * sin(c) + cos(a) * cos(b) * cos(c)) + cartesianZSub * (-1 * sin(b) * cos(c));
        cartesianZData = cartesianXSub * (sin(a) * sin(b)) + cartesianYSub * (cos(a) * sin(b)) + cartesianZSub * cos(b);

    }

    Coord <lType,aType>operator+(Coord<lType,aType> &coord){
        return Coord<lType,aType> (x()+coord.x(),y()+coord.y(),z()+coord.z(),ax()+coord.ax(),ay()+coord.ay(),az()+coord.az());
    }
    Coord <lType,aType>operator-(Coord<lType,aType> &coord){
        return Coord<lType,aType> (x()-coord.x(),y()-coord.y(),z()-coord.z(),ax()-coord.ax(),ay()-coord.ay(),az()-coord.az());
    }
    Coord<lType,aType> operator*(aType value){
        return Coord<lType,aType> (x()*value,y()*value,z()*value,ax()*value,ay()*value,az()*value);
    }
    Coord<lType,aType> operator/(aType value){
        return Coord<lType,aType> (x()/value,y()/value,z()/value,ax()/value,ay()/value,az()/value);
    }
    void operator+=(Coord<lType,aType> &coord){
        cartesian(x()+coord.x(),y()+coord.y(),z()+coord.z());
        angle(angleX()+coord.angleX(),angleY()+coord.angleY(),angleZ()+coord.angleZ());
    }
    void operator-=(Coord<lType,aType> &coord){
        cartesian(x()-coord.x(),y()-coord.y(),z()-coord.z());
        angle(angleX()-coord.angleX(),angleY()-coord.angleY(),angleZ()-coord.angleZ());
    }
    void operator*=(aType value){
        cartesian(x()*value,y()*value,z()*value);
        angle(angleX()*value,angleY()*value,angleZ()*value);
    }
    void operator/=(aType value){
        cartesian(x()/value,y()/value,z()/value);
        angle(angleX()/value,angleY()/value,angleZ()/value);
    }
    bool operator==(Coord &coord){
        return   ((x()==coord.x())
                ||(y()==coord.y())
                ||(z()==coord.z())
                ||(ax()==coord.ax())
                ||(ax()==coord.ax())
                ||(ax()==coord.ax()));
    }
/*    Coord <lType,aType>operator=(Coord<lType,aType> &coord){

    	return *this;
    }*/
};


template <>
class Coord<float,float>{
protected:
    float cartesianXData;
    float cartesianYData;
    float cartesianZData;
    float angleXData;
    float angleYData;
    float angleZData;

public:
    Coord(){
        cartesianXData=0;
        cartesianYData=0;
        cartesianZData=0;
        angleXData=0;
        angleYData=0;
        angleZData=0;
    };
    Coord(Coord const &coord){
        cartesianXData=coord.cartesianXData;
        cartesianYData=coord.cartesianYData;
        cartesianZData=coord.cartesianZData;
        angleXData=coord.angleXData;
        angleYData=coord.angleYData;
        angleZData=coord.angleZData;
    }
    Coord(float cx,float cy,float cz){
        cartesianXData=cx;
        cartesianYData=cy;
        cartesianZData=cz;
        angleXData=0;
        angleYData=0;
        angleZData=0;
    }
    Coord(float cx,float cy,float cz,float ex,float ey,float ez){
        cartesianXData=cx;
        cartesianYData=cy;
        cartesianZData=cz;
        angleXData=ex;
        angleYData=ey;
        angleZData=ez;
    }

    void cartesian(float cx,float cy,float cz){
        cartesianXData=cx;
        cartesianYData=cy;
        cartesianZData=cz;
    }
    float cartesianX(){
        return cartesianXData;
    }
    float cartesianY(){
        return cartesianYData;
    }
    float cartesianZ(){
        return cartesianZData;
    }
    void cartesianX(float x){
        cartesianXData=x;
    }
    void cartesianY(float y){
        cartesianYData=y;
    }
    void cartesianZ(float z){
        cartesianZData=z;
    }
    void cartesianXAdd(float x){
        cartesianXData=x;
    }
    void cartesianYAdd(float y){
        cartesianYData=y;
    }
    void cartesianZAdd(float z){
        cartesianZData=z;
    }

    void polar(float radius,float theta,float phi){
        float xy=0;
        if(theta==0){
            cartesianXData=0;
            cartesianYData=0;
            cartesianZData=radius;
            return;
        }
        cartesianZData=radius*cosf(theta);
        xy=radius*sinf(theta);
        cartesianXData=xy*cosf(phi);
        cartesianYData=xy*sinf(phi);
    }
    float polarRadius(){
        float radius;
        //    float theta;
        //    float phi;
        radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
        //    phi=atan2f(cartesianYData,cartesianXData);
        //    if(radius==0) theta=0;
        //    else theta=acosf(cartesianZData/radius);
        return radius;
    }
    float polarTheta(){
        float radius;
        float theta;
        //float phi;
        radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
        //phi=atan2f(cartesianYData,cartesianXData);
        if(radius==0) theta=0;
        else theta=acosf(cartesianZData/radius);
        return theta;
    }
    float polarPhi(){
        //	float radius;
        //	float theta;
        float phi;
        //    float radius;
        //float theta;
        //radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
        phi=atan2f(cartesianYData,cartesianXData);
        //if(radius==0) theta=0;
        //else theta=acosf(cartesianZData/radius);
        return phi;
    }
    void polarRadius(float radiusData){
        float radius;
        float theta;
        float phi;
        radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
        phi=atan2f(cartesianYData,cartesianXData);
        if(radius==0) theta=0;
        else theta=acosf(cartesianZData/radius);
        radius=radiusData;
        Coord::polar(radius,theta,phi);
    }
    void polarTheta(float thetaData){
        float radius;
        float theta;
        float phi;
        radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
        phi=atan2f(cartesianYData,cartesianXData);
        if(radius==0) theta=0;
        else theta=acosf(cartesianZData/radius);
        theta=thetaData;
        Coord::polar(radius,theta,phi);
    }
    void polarPhi(float phiData){
        float radius;
        float theta;
        float phi;
        radius=sqrtf(powf(cartesianXData,2)+powf(cartesianYData,2)+powf(cartesianZData,2));
        phi=atan2f(cartesianYData,cartesianXData);
        if(radius==0) theta=0;
        else theta=acosf(cartesianZData/radius);
        phi=phiData;
        Coord::polar(radius,theta,phi);
    }
    void polarRadiusAdd(float radius){
        polarRadius(polarRadius()+radius);
    }
    void polarThetaAdd(float theta){
        polarTheta(polarTheta()+theta);
    }
    void polarPhiAdd(float phiArg){
        polarPhi(polarPhi()+phiArg);
    }

    void angle(float x,float y,float z){
        angleXData=x;
        angleYData=y;
        angleZData=z;
    }
    float angleX(){
        return angleXData;
    }
    float angleY(){
        return angleYData;
    }
    float angleZ(){
        return angleZData;
    }
    void angleX(float x){
        angleXData=x;
    }
    void angleY(float y){
        angleYData=y;
    }
    void angleZ(float z){
        angleZData=z;
    }
    void angleXAdd(float x){
        angleXData+=x;
    }
    void angleYAdd(float y){
        angleYData+=y;
    }
    void angleZAdd(float z){
        angleZData+=z;
    }

    float x(){
        return cartesianX();
    }
    float y(){
        return cartesianY();
    }
    float z(){
        return cartesianZ();
    }
    float radius(){
        return polarRadius();
    }
    float theta(){
        return polarTheta();
    }
    float phi(){
        return polarPhi();
    }

    float ax(){
        return angleX();
    }
    float ay(){
        return angleY();
    }
    float az(){
        return angleZ();
    }

    float roll(){
        return angleX();
    }
    float pitch(){
        return angleX();
    }
    float yaw(){
        return angleX();
    }

    void x(float value){
        cartesianX(value);
    }
    void y(float value){
        cartesianY(value);
    }
    void z(float value){
        cartesianZ(value);
    }
    void radius(float value){
        polarRadius(value);
    }
    void theta(float value){
        polarTheta(value);
    }
    void phi(float value){
        polarPhi(value);
    }

    void ax(float value){
        angleX(value);
    }
    void ay(float value){
        angleY(value);
    }
    void az(float value){
        angleZ(value);
    }

    void revolutionXY(float angle){
        float cartesianXSub = cartesianXData;
        float cartesianYSub = cartesianYData;
        angleZData = angleZData + angle;
        //	int a = cosf(angle);
        //	std::cout << std::endl << " angle : " << a << std::endl;
        cartesianXData = cartesianXSub * cosf(angle) + (-1 * cartesianYSub * sinf(angle));
        cartesianYData = cartesianXSub * sinf(angle) + cartesianYSub * cosf(angle);
    }
    void revolutionYZ(float angle){
        float cartesianYSub = cartesianYData;
        float cartesianZSub = cartesianZData;
        angleXData = angleXData + angle;
        cartesianYData = cartesianYSub * cosf(angle) - cartesianZSub * sinf(angle);
        cartesianZData = cartesianYSub * sinf(angle) + cartesianZSub * cosf(angle);
    }
    void revolutionZX(float angle){
        float cartesianXSub = cartesianXData;
        float cartesianZSub = cartesianZData;
        angleYData = angleYData + angle;
        cartesianZData = cartesianZSub * cosf(angle) - cartesianXSub * sinf(angle);
        cartesianXData = cartesianZSub * sinf(angle) + cartesianXSub * cosf(angle);
    }

    void revolutionX(float angle){
        float cartesianXSub = cartesianXData;
        float cartesianYSub = cartesianYData;
        float cartesianZSub = cartesianZData;
        angleXData = angleXData + angle;
        cartesianXData = cartesianXSub;
        cartesianYData = cartesianYSub * cosf(angle) - cartesianZSub * sinf(angle);
        cartesianZData = cartesianYSub * sinf(angle) + cartesianZSub * cosf(angle);
    }
    void revolutionY(float angle){
        float cartesianXSub = cartesianXData;
        float cartesianYSub = cartesianYData;
        float cartesianZSub = cartesianZData;
        angleYData = angleYData + angle;
        cartesianXData = cartesianXSub * cosf(angle) + cartesianZSub * sinf(angle);
        cartesianYData = cartesianYSub;
        cartesianZData = (-1 * cartesianYSub * sinf(angle)) + cartesianZSub * cosf(angle);
    }
    void revolutionZ(float angle){
        float cartesianXSub = cartesianXData;
        float cartesianYSub = cartesianYData;
        float cartesianZSub = cartesianZData;
        angleZData = angleZData + angle;
        cartesianXData = cartesianXSub * cosf(angle) - cartesianYSub * sinf(angle);
        cartesianYData = cartesianXSub * sinf(angle) + cartesianYSub * cosf(angle);
        cartesianZData = cartesianZSub;
    }

    void revolutionXYPosition(float angle){
        float cartesianXSub = cartesianXData;
        float cartesianYSub = cartesianYData;
        //angleZData = angleZData + angle;
        //	int a = cosf(angle);
        //	std::cout << std::endl << " angle : " << a << std::endl;
        cartesianXData = cartesianXSub * cosf(angle) + (-1 * cartesianYSub * sinf(angle));
        cartesianYData = cartesianXSub * sinf(angle) + cartesianYSub * cosf(angle);
    }
    void revolutionYZPosition(float angle){
        float cartesianYSub = cartesianYData;
        float cartesianZSub = cartesianZData;
        //angleXData = angleXData + angle;
        cartesianYData = cartesianYSub * cosf(angle) - cartesianZSub * sinf(angle);
        cartesianZData = cartesianYSub * sinf(angle) + cartesianZSub * cosf(angle);
    }
    void revolutionZXPosition(float angle){
        float cartesianXSub = cartesianXData;
        float cartesianZSub = cartesianZData;
        //angleYData = angleYData + angle;
        cartesianZData = cartesianZSub * cosf(angle) - cartesianXSub * sinf(angle);
        cartesianXData = cartesianZSub * sinf(angle) + cartesianXSub * cosf(angle);
    }

    void revolutionXPosition(float angle){
        float cartesianXSub = cartesianXData;
        float cartesianYSub = cartesianYData;
        float cartesianZSub = cartesianZData;
        //angleXData = angleXData + angle;
        cartesianXData = cartesianXSub;
        cartesianYData = cartesianYSub * cosf(angle) - cartesianZSub * sinf(angle);
        cartesianZData = cartesianYSub * sinf(angle) + cartesianZSub * cosf(angle);
    }
    void revolutionYPosition(float angle){
        float cartesianXSub = cartesianXData;
        float cartesianYSub = cartesianYData;
        float cartesianZSub = cartesianZData;
        //angleYData = angleYData + angle;
        cartesianXData = cartesianXSub * cosf(angle) + cartesianZSub * sinf(angle);
        cartesianYData = cartesianYSub;
        cartesianZData = (-1 * cartesianYSub * sinf(angle)) + cartesianZSub * cosf(angle);
    }
    void revolutionZPosition(float angle){
        float cartesianXSub = cartesianXData;
        float cartesianYSub = cartesianYData;
        float cartesianZSub = cartesianZData;
        //angleZData = angleZData + angle;
        cartesianXData = cartesianXSub * cosf(angle) - cartesianYSub * sinf(angle);
        cartesianYData = cartesianXSub * sinf(angle) + cartesianYSub * cosf(angle);
        cartesianZData = cartesianZSub;
    }

    void revolutionEuler(float a, float b, float c){
        float cartesianXSub = cartesianXData;
        float cartesianYSub = cartesianYData;
        float cartesianZSub = cartesianZData;
        cartesianXData = cartesianXSub * (cos(a) * cosf(c) - sinf(a) * cosf(b) * sinf(c)) + cartesianYSub * (cos(a) * sinf(c) + sinf(a) * cosf(b) * cosf(c)) + cartesianZSub * (sin(a) * sinf(b));
        cartesianYData = cartesianXSub * (-1 * sinf(a) * cosf(c) - cosf(a) * cosf(b) * sinf(c)) + cartesianYData * (-1 * sinf(a) * sinf(c) + cosf(a) * cosf(b) * cosf(c)) + cartesianZData * (cos(a) * sinf(b));
        cartesianZData = cartesianXSub * (sin(b) * sinf(c)) + cartesianYSub * (-1 * sinf(b) * cosf(c)) + cartesianZSub * cosf(b);

    }
    void revolutionEulerinverse(float a, float b, float c){
        float cartesianXSub = cartesianXData;
        float cartesianYSub = cartesianYData;
        float cartesianZSub = cartesianZData;
        cartesianXData = cartesianXSub * (cos(a) * cosf(c) - sinf(a) * cosf(b) * sinf(c)) + cartesianYSub * (-1 * sinf(a) * cosf(c) + -1 * cosf(a) * cosf(b) * sinf(c)) + cartesianZSub * (sin(b) * sinf(c));
        cartesianYData = cartesianXSub * (cos(a) * sinf(c) + sinf(a) * cosf(b) * cosf(c)) + cartesianYSub * (-1 * sinf(a) * sinf(c) + cosf(a) * cosf(b) * cosf(c)) + cartesianZSub * (-1 * sinf(b) * cosf(c));
        cartesianZData = cartesianXSub * (sin(a) * sinf(b)) + cartesianYSub * (cos(a) * sinf(b)) + cartesianZSub * cosf(b);

    }

    Coord <float,float>operator+(Coord<float,float> &coord){
        return Coord<float,float> (x()+coord.x(),y()+coord.y(),z()+coord.z(),ax()+coord.ax(),ay()+coord.ay(),az()+coord.az());
    }
    Coord <float,float>operator-(Coord<float,float> &coord){
        return Coord<float,float> (x()-coord.x(),y()-coord.y(),z()-coord.z(),ax()-coord.ax(),ay()-coord.ay(),az()-coord.az());
    }
    Coord<float,float> operator*(float value){
        return Coord<float,float> (x()*value,y()*value,z()*value,ax()*value,ay()*value,az()*value);
    }
    Coord<float,float> operator/(float value){
        return Coord<float,float> (x()/value,y()/value,z()/value,ax()/value,ay()/value,az()/value);
    }
    void operator+=(Coord<float,float> &coord){
        cartesian(x()+coord.x(),y()+coord.y(),z()+coord.z());
        angle(angleX()+coord.angleX(),angleY()+coord.angleY(),angleZ()+coord.angleZ());
    }
    void operator-=(Coord<float,float> &coord){
        cartesian(x()-coord.x(),y()-coord.y(),z()-coord.z());
        angle(angleX()-coord.angleX(),angleY()-coord.angleY(),angleZ()-coord.angleZ());
    }
    void operator*=(float value){
        cartesian(x()*value,y()*value,z()*value);
        angle(angleX()*value,angleY()*value,angleZ()*value);
    }
    void operator/=(float value){
        cartesian(x()/value,y()/value,z()/value);
        angle(angleX()/value,angleY()/value,angleZ()/value);
    }
    bool operator==(Coord<float,float> &coord){
        return   ((x()==coord.x())
                  ||(y()==coord.y())
                  ||(z()==coord.z())
                  ||(ax()==coord.ax())
                  ||(ax()==coord.ax())
                  ||(ax()==coord.ax()));
    }
};

typedef Coord<float> Coordf;
typedef Coord<double> Coordd;

template <typename lType,typename aType=lType>
class PhysicalObject{
public:
    Coord<lType,aType> position;
    Coord<lType,aType> velocity;
    Coord<lType,aType> acceleration;
    Coord<lType,aType> jerk;
    void gainTime(float seconds){velocity=velocity+acceleration*seconds;position=position+velocity*seconds;};
};


template<typename T=char,int bufSize=256> class RingBuffer{
private:
    T data[bufSize+2];
    int readPointer;
    int writePointer;

public:
    RingBuffer(){
        readPointer=0;
        writePointer=0;
    };
    int write(T value){
        int next=(writePointer + 1) % (bufSize+1);
        if(readPointer==next)return 1;
        data[writePointer]=value;
        writePointer = next;
        return 0;
    }
    T& read(){
        if(readPointer==writePointer) return data[readPointer];
        int oldReadPointer=readPointer;
        readPointer = (readPointer + 1) % (bufSize+1);
        return data[oldReadPointer];
    }
    T& peek(){
        if(readPointer==writePointer) return data[readPointer];
        return data[readPointer];
    }
    T& peek(int pointer){
        if(pointer>=length()||pointer<0) return data[readPointer];
        int tReadPointer = (readPointer + pointer) % (bufSize+1);
        return data[tReadPointer];
    }
    void clear(){ readPointer = 0; writePointer = 0; };
    void remove(){
        if(isEmpty())return;
        if(writePointer>0)writePointer--;
        else writePointer+=bufSize;
    };
    int isEmpty(){return  (readPointer==writePointer)?(1):(0);}
    int isFull(){return (readPointer==((writePointer + 1) % (bufSize+1)))?(1):(0);}
    int size(){return bufSize;};
    int length(){
        if(writePointer>=readPointer) return writePointer-readPointer;
        else                          return bufSize-(readPointer-writePointer-1);
    }
};

#include <stdint.h>
template<typename T,int bufSize> class Recorder{
public:
    void write(T value,int64_t time){
        //Record force the first data.
        if(records.isEmpty()){
            Record record(value,time);
            records.write(record);
            return;
        }

        //Ignore same values.
        if(value==records.peek(records.length()-1).value){
            return;
        }

        //Make space to record a new data when the buffer is full.
        if(records.isFull()){
            records.read();
        }

        //Record
        Record record(value,time);
        records.write(record);
    }

    T& read(int64_t time){
        for(int i=records.length()-1;i>=0;i--){
            if(records.peek(i).time<=time){
                return records.peek(i).value;
            }
        }
        return records.peek(records.length()-1).value;
    }

    T smoothRead(int64_t time){
        for(int i=records.length()-1;i>=0;i--){
            if(records.peek(i).time<=time){
                if(records.length()-1<=i){
                    //std::cout << "smoothRead end records" << std::endl;
                    return records.peek(records.length()-1).value;
                }

                return (records.peek(i).value*(records.peek(i+1).time-time)
                        +records.peek(i+1).value*(time-records.peek(i).time))
                /(records.peek(i+1).time-records.peek(i).time);
            }
        }
        //std::cout << "smoothRead invailed time" << std::endl;
        return records.peek(0).value;
    }
private:
    class Record{
    public:
        Record(){
        }
        Record(T value,int64_t time){
            this->value=value;
            this->time=time;
        }
        T value;
        int64_t time;
    };
    RingBuffer<Record,bufSize> records;
};

class Change{
public:
    int oldValue;
    int operator()(int value){
        if(oldValue!=value){
            oldValue=value;
            return 1;
        }else{
            oldValue=value;
            return 0;
        }
    };
};
class UpEdge{
public:
    bool oldValue;
    int operator()(bool value){
        if(oldValue==false&&value==true){
            oldValue=value;
            return 1;
        }else{
            oldValue=value;
            return 0;
        }
    };
};
class DownEdge{
public:
    bool oldValue;
    int operator()(bool value){
        if(oldValue==true&&value==false){
            oldValue=value;
            return 1;
        }else{
            oldValue=value;
            return 0;
        }
    };
};

class Toggle{
public:
	int returnValue;
	UpEdge sw;
	Toggle(){returnValue=0;sw(true);}
	int operator()(bool value){
		if(sw(value)){
			returnValue=!returnValue;
		}
		return returnValue;
	};
	void operator=(bool value){
		returnValue=value;
	}
};

//a*x^2+b*x^1+c=0
int quadraticEquationSolver(float &x1,float &x1i,float &x2,float &x2i,float a,float b,float c);
int quadraticEquationSolver(std::complex<float> &x1,std::complex<float> &x2,float a,float b,float c);

//a*x^3+b*x^2+c*x^1+d=0
int cubicEquationSolver(std::complex<float> &x1,std::complex<float> &x2,std::complex<float> &x3
                        ,float a,float b,float c,float d);
int cubicEquationSolver(float &x1r,float &x1i,float &x2r,float &x2i,float &x3r,float &x3i
                        ,float a,float b,float c,float d);
