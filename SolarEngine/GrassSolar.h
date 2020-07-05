#pragma once

#ifdef _WIN32 || WIN32
#include<Windows.h>
#endif

#include "TypeDef.h"
#include "Utils.h"
#include <osg/Node>

class RayCasterBase
{
public:
	virtual bool isShadowed(const double& alt, const double& azimuth, const osg::Vec3d& pos) { return false; };
};

class GrassSolar
{
public:
		GrassSolar() {};
		~GrassSolar() {};
		std::vector<SunVector> getSunVectors(SolarParam& sparam);
		SolarRadiation calculateSolarRadiation(SolarParam& solar_param);
		SolarRadiation calculateSolarRadiation(SolarParam& solar_param, osg::Node* sceneNode, osg::Vec3d pos);
		std::tuple<SolarRadiationPoint, SolarRadiationPoint> calculateSolarRadiation(SolarParam solarParam, const osg::Vec3d& pos, RayCasterBase* rayCaster);
private:
	unsigned int m_curTimeStep;
	bool m_COLLECT_SUN_VECTOR = false;
	std::vector<SunVector> m_sunVectors;

	//structure for holding temporary values
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
		double latitude, lum_time, declination;
		double sinlat, coslat, sindecl, cosdecl;
		double longit_l, latid_l, cos_u, cos_v, sin_u, sin_v;
		double sin_phi_l, tan_lam_l, lum_C31_l, lum_C33_l;
		double beam_e, diff_e, refl_e, bh, dh, rr, insol_t;
		double coslatsq;
		bool* shadowInfo;
		SolarRadiation* parts;
	}TempVariables;

private:
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
};