
#ifndef  GrassSolar_H
#define  GrassSolar_H

#ifdef _WIN32 || WIN32
#include<Windows.h>
#endif

#include <math.h>
#include <vector>
#include <osg/Vec3>
#include <osg/Matrix>
#include <map>
#include <osgUtil/IntersectionVisitor>
#include "osg/LineSegment"
#include <osgUtil/LineSegmentIntersector>
#include "ModelLoader.h"

//structure of solar radiation result
struct SolarRadiation
{
		float global;//global component
		float beam;//beam component
		float diffuse;//diffuse component
		float reflected;//reflected component
		std::string shadowMasks;
public:
		SolarRadiation()
		{
				global = -9999;
				beam = -9999;
				diffuse = -9999;
				reflected = -9999;
				shadowMasks = "";
		}
		void Zero()
		{
				global = 0;
				beam = 0;
				diffuse = 0;
				reflected = 0;
				shadowMasks = "";
		}

		SolarRadiation operator+(const SolarRadiation& rad)
		{
				SolarRadiation newrad;
				newrad.beam = rad.beam + this->beam;
				newrad.global = rad.global + this->global;
				newrad.diffuse = rad.diffuse + this->diffuse;
				newrad.reflected = rad.reflected + this->reflected;
				return newrad;
		}
		//SolarRadiation operator*(const SolarRadiation& rad,const float& val)
		//{
		//	SolarRadiation newrad;
		//	newrad.beam=rad.beam*val;
		//	newrad.global=rad.global*val;
		//	newrad.diffuse=rad.diffuse*val;
		//	newrad.reflected=rad.reflected*val;
		//	return newrad;
		//}
		SolarRadiation operator*(const SolarRadiation& rad)
		{
				SolarRadiation newrad;
				newrad.beam = rad.beam*this->beam;
				newrad.global = rad.global*this->global;
				newrad.diffuse = rad.diffuse*this->diffuse;
				newrad.reflected = rad.reflected*this->reflected;
				return newrad;
		}
		SolarRadiation operator*(const float& val)
		{
				SolarRadiation newrad;
				newrad.beam = beam * val;
				newrad.global = global * val;
				newrad.diffuse = diffuse * val;
				newrad.reflected = reflected * val;
				return newrad;
		}

		SolarRadiation operator/(const float& val)
		{
				SolarRadiation newrad;
				newrad.beam = beam / val;
				newrad.global = global / val;
				newrad.diffuse = diffuse / val;
				newrad.reflected = reflected / val;
				return newrad;
		}

		//SolarRadiation operator/(const SolarRadiation& rad)
		//{
		//	SolarRadiation newrad;
		//	newrad.beam=rad.beam/this->beam;
		//	newrad.global=rad.global/this->global;
		//	newrad.diffuse=rad.diffuse/this->diffuse;
		//	newrad.reflected=rad.reflected/this->reflected;
		//	return newrad;
		//}
		//SolarRadiation operator/(const SolarRadiation& rad,const float& val)
		//{
		//	SolarRadiation newrad;
		//	newrad.beam=rad.beam/val;
		//	newrad.global=rad.global/val;
		//	newrad.diffuse=rad.diffuse/val;
		//	newrad.reflected=rad.reflected/val;
		//	return newrad;
		//}

};


//structure of solar radiation result
typedef struct/* __align__(16)*/
{
public:
		bool tien;
		double day;
		double linke;
		double length, c, declin, step;
		double elev, slope, aspect;
		double lum_C11, lum_C13, lum_C22, lum_C31, lum_C33, lum_Lx, lum_Ly, lum_Lz;
		double lum_p, sunrise_time, sunset_time, h0, tanh0, A0, angle;
		double longitude, latitude, lum_time, declination;
		double sinlat, coslat, sindecl, cosdecl;
		double longit_l, latid_l, cos_u, cos_v, sin_u, sin_v;
		double sin_phi_l, tan_lam_l, lum_C31_l, lum_C33_l;
		double beam_e, diff_e, refl_e, bh, dh, rr, insol_t;
		double coslatsq;
		bool* shadowInfo;
		SolarRadiation* parts;
}TempVariables;

struct Stats
{
		float fmin;
		float fmax;
		float fmean;
		float std;
		float sum;
		float validnum;
		Stats()
		{

				fmin = FLT_MAX;
				fmax = -FLT_MAX;
				fmean = 0;
				std = 0;
				sum = 0;
				validnum = 0;
		}
};

//structure of solar vector
typedef struct
{
		float time;
		float azimuth;//azimuth angle
		float alt; //elevation angle
		//float x,y,z; //position
}SunVector;

struct SolarTime
{
		int hour;
		int minute;
		int second;
		SolarTime()
		{
				hour = 6;
				minute = 0;
				second = 0;
		}
		SolarTime(int h, int m, int s)
				:hour(h), minute(m), second(s)
		{
		}
		double toDecimalHour()
		{
				return hour + minute / 60.0 + second / 3600.0;
		}

};


//parameters for r.sun calculation
struct SolarParam
{
		float linke;//turbidity factor
		float bsky;//scale factor for the beam component
		float dsky;//scale factor for the diffuse component
		float lon;//longitude
		float lat;//latitude
		float elev;//elevation
		float slope;//slope in degrees
		float aspect;//aspect in degrees
		float time_step;//time resolution in hours
		int day;//range from 1 to 366
		bool* shadowInfo;//an array of shadow masks corresponding to the number of solar vectors in a day
		bool isShadowed;//a single shadow mask will be used if 'shadowInfo' is null
		bool isInstantaneous;//apply instantaneous calculation mode
		bool isSingleDay;
		bool useLatitudeOverride;
		bool useElevationOverride;
		int startDay;
		int endDay;
		SolarTime time;//decimal time 
		SolarParam()
		{
				shadowInfo = NULL;
				isShadowed = false;
				isInstantaneous = false;//time-integrated calculation mode as default
				elev = 0;
				slope = 0;
				aspect = 0;
				linke = 3.0;
				bsky = 1;
				dsky = 1;
				time_step = 1;
				day = 1;
				startDay = 1;
				endDay = 1;
				isSingleDay = true;
				useLatitudeOverride = true;
				useElevationOverride = true;
		}

};



class GrassSolar
{
public:
		GrassSolar() {};
		~GrassSolar() {};

	 static	double calculateAspect(osg::Vec3d normal);
		static double calculateSlope(osg::Vec3d normal);
		static osg::Vec3d solarAngle2Vector(double alt, double azimuth);
		static std::vector<osg::Vec3d> sunVector2LightDir(std::vector<SunVector>& sunvectors);

		void com_par(TempVariables& tmpval);
		void com_par_const(TempVariables& tmpval);
		double lumcline2(TempVariables& tmpval);
		void joules2(TempVariables& tmpval, const bool& isInstaneous, const double& assignedTime);
		double com_sol_const(double no_of_day);
		double com_declin(double);
		double brad(double sh, TempVariables& tmpval);
		//double drad(double sh,TempVariables& tmpval);
		double drad_isotropic(double sh, TempVariables& tmpval);
		// This function converts decimal degrees to radians
		double deg2rad2(double deg);
		//  This function converts radians to decimal degrees
		double rad2deg2(double rad);

		bool COLLECT_SUN_VECTOR = false;

		std::vector<SunVector> SunVectors;
		std::vector<SolarRadiation> ClearSkyRads;
		SolarRadiation PreviousSolarRad;
		//perform actual calculation
		SolarRadiation calculateSolarRadiation(SolarParam& solar_param);
		//obtain the array of solar vectors for a specified day
		std::vector<SunVector> getSunVectors(SolarParam& sparam);
		SolarRadiation calculateSolarRadiation(SolarParam& solar_param, osg::Node* sceneNode, osg::Vec3d pos);
};
#endif