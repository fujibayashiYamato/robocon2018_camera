#pragma once
/*******************************************
 * util ver1.0 2015/1/11
 * This is a program made for mathematically calculation.
 *
 * [Dependency]
 * none
 *
 * [Note]
 * Coordinate functions will not support well in the future.
 * To operate coordinate, using "utilplus" is recommended.
 *
 * [Author]
 * Tomoki Nagatani
 *
 * [Change history]
 * ver1.1 2015/ 3/ 4 Add converter for int,unsigned short,short
 * ver1.1 2015/ 2/ 7 Add weak to sign, for utilplus1.0.
 * ver1.0 2015/ 1/11 C++ include capable.
 ******************************************/

#ifdef __cplusplus

#include "util.h"
#include "math.h"
#include <stdio.h>

 extern "C" {
#endif
float dtor(float x);
float rtod(float x);
__attribute__ ((weak)) int sign(int val);
int fsign(float val);
float constrain(float x,float a,float b);
float area(float value,float shita,float ue);
float max(float a,float b);
float min(float a,float b);
float abs_max(float a,float b);
float abs_min(float a,float b);
float floatlimit(float mae,float val,float ato);

int uchar4_to_int(unsigned char *value);
void int_to_uchar4(unsigned char *value,int int_value);

unsigned short uchar2_to_ushort(unsigned char *value);
void ushort_to_uchar2(unsigned char *value,unsigned short short_value);

#ifdef __cplusplus
}

float dtor(float x){
	return (2*M_PI*x/360);	//x/360*2*PI
}

float rtod(float x){
	return (x*360/(2*M_PI));	//x/(2*PI)*360
}

int sign(int val)
{
	if(val>=0) return 1;
	else return -1;
}

int fsign(float val)
{
	if(val>=0.0) return 1;
	else return -1;
}

float floatlimit(float mae,float val,float ato)
{
	if(mae>=val) return mae;
	if(ato<=val) return ato;
	return val;
}

float constrain(float x,float a,float b){
	if(x<a) return a;
	else if(b<x) return b;
	else return x;
}

float area(float value,float shita,float ue)
{
	int i=0,limit=100;
	if(shita>=ue) return 0;
	for(i=0;value<shita&&i<limit;i++) {value+=(ue-shita);}
	for(i=0;ue<value&&i<limit;i++) {value-=(ue-shita);}
	return value;
}

float max(float a,float b){
	if(a>b) return a;
	else return b;
}

float min(float a,float b){
	if(a<b) return a;
	else return b;
}

float abs_max(float a,float b){
    if(fabs(a)>fabs(b)) return a;
    else return b;
}

float abs_min(float a,float b){
    if(fabs(a)<fabs(b)) return a;
    else return b;
}

typedef union{
    int int_value;
    unsigned char char4_value[4];
}Int_char4;

int uchar4_to_int(unsigned char *value){
    Int_char4 tmp;
    tmp.char4_value[0]=value[0];
    tmp.char4_value[1]=value[1];
    tmp.char4_value[2]=value[2];
    tmp.char4_value[3]=value[3];
    return tmp.int_value;
}

void int_to_uchar4(unsigned char *value,int int_value){
    Int_char4 tmp;
    tmp.int_value=int_value;
    value[0]=tmp.char4_value[0];
    value[1]=tmp.char4_value[1];
    value[2]=tmp.char4_value[2];
    value[3]=tmp.char4_value[3];
}

typedef union{
    unsigned short short_value;
    unsigned char char2_value[2];
}UShort_char2;


unsigned short uchar2_to_ushort(unsigned char *value){
    UShort_char2 tmp;
    tmp.char2_value[0]=value[0];
    tmp.char2_value[1]=value[1];
    return tmp.short_value;
}

void ushort_to_uchar2(unsigned char *value,unsigned short short_value){
    UShort_char2 tmp;
    tmp.short_value=short_value;
    value[0]=tmp.char2_value[0];
    value[1]=tmp.char2_value[1];
}

#endif
