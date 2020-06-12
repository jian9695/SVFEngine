#include "TypeDef.h"


SolarRadiationPoint::SolarRadiationPoint(const osg::Vec3d position, const SolarParam& param, const SolarRadiation& rad)
{
	m_id = 0;
	m_pos = position;
	*((SolarParam*)this) = param;
	*((SolarRadiation*)this) = rad;
}

std::string SolarRadiationPoint::toString()
{
	std::stringstream ofs;
	std::vector<std::pair<std::string, OuputVariable>> outputVariables;
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Global [kWh/m2]:", OuputVariable(m_global / 1000)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Beam [kWh/m2]:", OuputVariable(m_beam / 1000)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Diffuse [kWh/m2]:", OuputVariable(m_diffuse / 1000)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Reflected [kWh/m2]:", OuputVariable(m_reflected / 1000)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("SVF:", OuputVariable(m_svf)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Slope:", OuputVariable(m_slope)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Aspect:", OuputVariable(m_aspect)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Start day :", OuputVariable(m_startDay)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("End day:", OuputVariable(m_endDay)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Time step:", OuputVariable(m_time_step)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Linke:", OuputVariable(m_linke)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Lat:", OuputVariable(m_lat)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Lon:", OuputVariable(m_lon)));
	outputVariables.push_back(std::pair<std::string, OuputVariable>("Elevation:", OuputVariable(m_elev)));
	//outputVariables.push_back(std::pair<std::string, OuputVariable>("3D Pos:", OuputVariable(m_pos)));
	for (int v = 0; v < outputVariables.size(); v++)
	{
		ofs << outputVariables[v].first << " ";

		if (outputVariables[v].second.m_type != ValType::vec)
		{
			outputVariables[v].second.out(ofs);
		}
		else
		{
			ofs << "[" << outputVariables[v].second.m_vec.x() << "," << outputVariables[v].second.m_vec.y() << "," << outputVariables[v].second.m_vec.z() << "]";
		}

		if (v != outputVariables.size())
		{
			ofs << "\n";
		}
	}
	return ofs.str();
}