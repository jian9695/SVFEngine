#pragma once
#ifdef _WIN32 || WIN32 
#include <Windows.h>
#endif

#include <string>
#include <math.h>
#include <fstream>
#include <osg/Vec3d>

//structure of solar radiation result
struct SolarRadiation
{
	float global;//global component
	float beam;//beam component
	float diffuse;//diffuse component
	float reflected;//reflected component
	float svf;//sky view factor
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
		newrad.beam = rad.beam * this->beam;
		newrad.global = rad.global * this->global;
		newrad.diffuse = rad.diffuse * this->diffuse;
		newrad.reflected = rad.reflected * this->reflected;
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
struct SunVector
{
	float time;
	float azimuth;//azimuth angle
	float alt; //elevation angle
	//float x,y,z; //position
	SunVector() {}
	SunVector(float azimuthAngle, float altAngle) { azimuth = azimuthAngle; alt = altAngle; }
};

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
		lon = -9999;
		isSingleDay = true;
		useLatitudeOverride = true;
		useElevationOverride = true;
	}

};

struct SolarRadiationPoint : public SolarParam, public SolarRadiation
{
	osg::Vec3d pos;
	SolarRadiationPoint() {}
	SolarRadiationPoint(const osg::Vec3d position, const SolarParam& param, const SolarRadiation& rad)
	{
		pos = position;
		*((SolarParam*)this) = param;
		*((SolarRadiation*)this) = rad;
	}
};

enum ValType
{
	d = 0,
	f = 1,
	vec = 2,
	s = 3,
	i = 4
};

struct OuputVariable
{
public:
	double _d;
	float _f;
	osg::Vec3d _vec;
	std::string _s;
	long _i;
	ValType _type;
	OuputVariable(double val)
	{
		_d = val;
		_type = ValType::d;
	}

	OuputVariable(float val)
	{
		_f = val;
		_type = ValType::f;
	}

	OuputVariable(const osg::Vec3d& val)
	{
		_vec = val;
		_type = ValType::vec;
	}

	OuputVariable(const std::string& val)
	{
		_s = val;
		_type = ValType::s;
	}

	OuputVariable(int val)
	{
		_i = val;
		_type = ValType::i;
	}

	OuputVariable(long val)
	{
		_i = val;
		_type = ValType::i;
	}

	void Out(std::ofstream& ofs)
	{
		if (_type == ValType::d)
		{
			ofs << _d;
		}
		else if (_type == ValType::f)
		{
			ofs << _f;
		}
		else if (_type == ValType::s)
		{
			ofs << "\"" << _s << "\"";
		}
		else if (_type == ValType::i)
		{
			ofs << _i;
		}
		else if (_type == ValType::vec)
		{
			ofs << "\"" << "[" << _vec.x() << "," << _vec.y() << "," << _vec.z() << "\"" << "]";
		}
	}
};

