#include "Utils.h"
#include "GrassSolar.h"
#include <sstream>

//Angle between vector (x,y) and the positive Y axis (0,1) with origin at (0,0)
double Utils::calAzimuthAngle(double x, double y)
{
	double x2 = 0.0;
	double y2 = 1.0;
	double dot = x * x2 + y * y2;      //# dot product
	double det = x * y2 - y * x2;      //# determinant
	double angle = osg::RadiansToDegrees(atan2(det, dot));  //# atan2(y, x) or atan2(sin, cos)
	if (angle < 0)
		angle += 360;
	return angle;
}

//The aspect categories represent the number degrees of east and they increase counterclockwise: 90deg is North, 180 is West, 270 is South 360 is East.
//The aspect value 0 is used to indicate undefined aspect in flat areas with slope=0. 
//
//The slope output raster map contains slope values, 
//stated in degrees of inclination from the horizontal if format=degrees option (the default) is chosen, 
//and in percent rise if format=percent option is chosen. Category and color table files are generated. 
//***********N90

//*******W180    E360 

//***********S270

//***********N0

//*******W270    E90 

//***********S180
//tmpval.elev = 0;
//if (tmpval.aspect != 0.) {
//	if (tmpval.aspect < 90.)
//		tmpval.aspect = 90. - tmpval.aspect;
//	else
//		tmpval.aspect = 450. - tmpval.aspect;
//}

double Utils::calculateAspect(osg::Vec3d normal)
{
	double aspect = 0;
	if (normal.x() == 0 && normal.y() == 0)
		return aspect;
	osg::Vec3d normalxy = normal;
	normalxy.normalize();
	normalxy.z() = 0;
	normalxy.normalize();
	double cosa = acos(normalxy * osg::Vec3d(1, 0, 0));
	aspect = osg::RadiansToDegrees(cosa);
	if (normal.y() < 0)
		aspect = 360 - aspect;
	return aspect;
}

double Utils::calculateSlope(osg::Vec3d normal)
{
	double cosa = acos(normal * osg::Vec3d(0, 0, 1));
	double slope = osg::RadiansToDegrees(cosa);
	return slope;
}

osg::Vec3d Utils::solarAngle2Vector(double alt, double azimuth)
{
	osg::Vec3d lightDir;
	lightDir.z() = cos(osg::DegreesToRadians(90.0 - alt));
	double projectedLenghOnXY = cos(osg::DegreesToRadians(alt));
	lightDir.y() = projectedLenghOnXY * cos(osg::DegreesToRadians(azimuth));
	//lightDir.x() = sqrt((projectedLenghOnXY * projectedLenghOnXY) - lightDir.y() * lightDir.y());
	lightDir.x() = projectedLenghOnXY * cos(osg::DegreesToRadians(90 - azimuth));
	lightDir.normalize();
	return lightDir;
}

std::vector<osg::Vec3d> Utils::sunVector2LightDir(std::vector<SunVector>& sunvectors)
{
	std::vector<osg::Vec3d> vsuns;
	for (int i = 0; i < sunvectors.size(); i++)
	{
		vsuns.push_back(Utils::solarAngle2Vector(sunvectors[i].alt, sunvectors[i].azimuth));
	}
	return vsuns;
}

double Utils::calSVF(osg::Image* img, bool applyLambert)
{
	unsigned int skypixels = 0;
	unsigned int nonskypixels = 0;
	unsigned char* data = img->data();
	unsigned int numpixels = img->s() * img->t();
	unsigned int ncols = img->s();
	unsigned int nrows = img->t();
	if (ncols != nrows)
		return 0;
	double resol = 1.0 / nrows;
	double totalarea = 0;
	double skyarea = 0;
	double y = resol * 0.5 - 0.5;
	for (unsigned int row = 0; row < nrows; row++)
	{
		y += resol;
		double x = resol * 0.5 - 0.5;
		for (unsigned int col = 0; col < ncols; col++)
		{
			unsigned char a = data[3];
			if (a == 0) {
				x += resol;
				data += 4;
				continue;//outside
			}
			double zenithD = sqrt(x * x + y * y) * 90.0;//in degrees
			if (zenithD <= 0.000000001)
				zenithD = 0.000000001;
			double zenithR = zenithD * 3.1415926 / 180.0;
			double wproj = sin(zenithR) / (zenithD / 90);//weight for equal-areal projection
			if (applyLambert)
			{
				wproj = wproj * cos(zenithR);
			}
			totalarea += wproj;
			if (a < 130)
			{
				skypixels++;
				skyarea += wproj;
			}
			else
			{
				nonskypixels++;
			}
			x += resol;
			data += 4;
		}

	}
	double svf = skyarea / totalarea;
	return svf;
}

SolarRadiation Utils::calSolar(osg::Image* img, SolarParam* solarParam, osg::Vec3d pos, osg::Vec3d normal, osg::Node* sceneNode)
{
	SolarRadiation annualRad;
	annualRad.Zero();
	std::vector<SolarRadiation> dailyRads;
	SolarParam param = *solarParam;
	param.slope = Utils::calculateSlope(normal);
	param.aspect = Utils::calculateAspect(normal);

	int startDay = param.startDay;
	int endDay = param.endDay;
	if (endDay > startDay)
		endDay = startDay;
	int lastSteps = 0;
	GrassSolar rsun;
	for (int day = startDay; day <= endDay; day++)
	{
		param.startDay = day;
		//std::vector<bool> shadowMasks;
		//std::vector<bool> shadowMasksRayCaster;
		std::vector<SunVector> sunVecs = rsun.getSunVectors(param);
		//shadowMasks.resize(sunVecs.size());
		if (lastSteps != sunVecs.size())
		{
			if (param.shadowInfo)
				delete[] param.shadowInfo;
			param.shadowInfo = new bool[sunVecs.size()];
		}
		for (int n = 0; n < sunVecs.size(); n++)
		{
			SunVector sunVec = sunVecs[n];
			//osg::Vec3d sundir = GrassSolar::solarAngle2Vector(sunVec.alt, sunVec.azimuth);
			//osg::Vec3d start = pos;
			//osg::Vec3d end = pos + sundir * 10000000;
			//osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(start, end);
			//osgUtil::IntersectionVisitor intersectVisitor(intersector.get());
			//sceneNode->accept(intersectVisitor);
			//if (intersector->containsIntersections())
			//{
			//	shadowMasksRayCaster.push_back(true);
			//}
			//else
			//{
			//	shadowMasksRayCaster.push_back(false);
			//}
			double radius = (90.0 - sunVec.alt) / 90.0 * 0.5;
			//double radius = sin(osg::DegreesToRadians(90.0 - sunVec.alt)) * 0.5;
			double theta = sunVec.azimuth - 90;
			if (theta < 0)
				theta += 360;
			theta = osg::DegreesToRadians(theta);
			double x = radius * cos(theta);
			double y = radius * sin(theta);
			x += 0.5;
			y += 0.5;
			osg::Vec4 color = img->getColor(osg::Vec2(x, y));
			param.shadowInfo[n] = color.a() > 0.7 ? true : false;
			//shadowMasks[n] = param.shadowInfo[n];
		}

		/*for (int n = 0; n < sunVecs.size(); n++)
		{
			printf("%d", shadowMasks[n] ? 1 : 0);
			if(n == sunVecs.size() - 1)
				printf("\n");
			else
				printf(",");
		}

		for (int n = 0; n < sunVecs.size(); n++)
		{
			printf("%d", shadowMasksRayCaster[n] ? 1 : 0);
			if (n == sunVecs.size() - 1)
				printf("\n");
			else
				printf(",");
		}*/

		param.day = day;
		SolarRadiation dailyRad = rsun.calculateSolarRadiation(param);
		dailyRads.push_back(dailyRad);
		annualRad = annualRad + dailyRad;
	}
	if (param.shadowInfo)
		delete[] param.shadowInfo;
	return annualRad;
}

std::string Utils::value2String(float value, int precision)
{
	std::stringstream buf;
	buf.precision(precision);
	buf << std::fixed << value;
	return buf.str();
}
