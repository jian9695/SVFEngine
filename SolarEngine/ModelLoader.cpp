#include "ModelLoader.h"
#include <osgDB/ReadFile>
#include <vector>
#include <sstream>
#include <qdir.h>
#include <qfile.h>
#include "osg/ComputeBoundsVisitor"
#include "osgUtil/SmoothingVisitor"
#include "osgDB/writeFile"
#include "osgDB/readFile"
#include "ShapeFile.h"
#include "GrassSolar.h"
#include "GDAL_DS.h"

ModelLoader::ModelLoader()
{
}


ModelLoader::~ModelLoader()
{

}

std::string getBaseName(std::string path)
{
		char last = path[path.length() - 1];
		if (last == '/' || last == '\\')
		{
				path = path.substr(0, path.length() - 1);
		}
		return QFileInfo(path.data()).baseName().toLocal8Bit().data();
}

std::vector<std::string> findSubdirs(std::string dir)
{
		std::vector<std::string> subdirs;
		QDir rootdir(dir.data());
		rootdir.setFilter(QDir::Dirs | QDir::Hidden | QDir::NoSymLinks | QDir::NoDotAndDotDot);
		rootdir.setSorting(QDir::Name);
		std::vector<std::string> files;
		QFileInfoList list = rootdir.entryInfoList();
		for (int i = 0; i < list.size(); ++i) {
				QFileInfo fileInfo = list.at(i);
				std::string dir = (fileInfo.absoluteFilePath() + "/").toLocal8Bit().data();
				subdirs.push_back(dir);
		}
		return subdirs;
}

std::vector<std::string> findFiles(std::string indir, std::string match)
{
		std::vector<std::string> files;
		QDir input_dir(indir.data());
		input_dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks | QDir::NoDotAndDotDot);
		input_dir.setSorting(QDir::Name);
		indir = (input_dir.absolutePath() + "/").toLocal8Bit().data();
		QFileInfoList list = input_dir.entryInfoList();
		for (int i = 0; i < list.size(); ++i) {
				QFileInfo fileInfo = list.at(i);
				std::string input_file = (input_dir.absolutePath() + "/" + fileInfo.fileName()).toLocal8Bit().data();
				if (/*!fileInfo.fileName().contains(match.data(),Qt::CaseInsensitive) || */match != "" && !fileInfo.fileName().endsWith(match.data(), Qt::CaseInsensitive))
						continue;
				files.push_back(fileInfo.absoluteFilePath().toLocal8Bit().data());
		}
		return files;
}

std::vector<std::string> findLeafTileFiles(const std::vector<std::string>& tileFiles, const std::string& masterTileName)
{
		std::vector<int> levels;
		int maxLevel = -1;
		for (size_t i = 0; i < tileFiles.size(); i++)
		{
				std::string tilename = QFileInfo(tileFiles[i].data()).baseName().toLocal8Bit().data();
				if (tilename.length() - masterTileName.length() < 3)
				{
						levels.push_back(-1);
						continue;
				}

				tilename = tilename.substr(masterTileName.length() + 2, tilename.size() - masterTileName.length() - 2);
				std::string levelStr = "";
				for (size_t j = 0; j < tilename.length(); j++)
				{
						if (!isdigit(tilename[j]))
								break;
						levelStr += tilename[j];
				}

				int level = -1;
				std::stringstream ss;
				ss << levelStr;
				ss >> level;
				levels.push_back(level);
				if (maxLevel < level)
						maxLevel = level;
		}

		std::vector<std::string> leafTiles;
		for (size_t i = 0; i < tileFiles.size(); i++)
		{
				if (levels[i] == maxLevel)
				{
						leafTiles.push_back(tileFiles[i]);
				}
		}
		return leafTiles;
}

std::vector<std::string> findMasterTiles(std::string indir)
{
		std::vector<std::string> files;
		std::vector<std::string> dirs = findSubdirs(indir);
		for (size_t i = 0; i < dirs.size(); i++)
		{
				if (dirs[i].find("Tile_") == std::string::npos)
						continue;
				std::string masterTileName = getBaseName(dirs[i]);
				std::string masterTilePath = dirs[i] + masterTileName + ".osgb";
				files.push_back(masterTilePath);
		}
		return files;
}

OGRPolygon* toOGRPolygon(OGRLayer* layer, const OGREnvelope& bb)
{

		OGRPolygon *poPolygon = (OGRPolygon*)OGRGeometryFactory::createGeometry(wkbPolygon);
		OGRLinearRing  *linearRing = (OGRLinearRing  *)OGRGeometryFactory::createGeometry(wkbLinearRing);
		linearRing->addPoint(bb.MinX, bb.MinY);
		linearRing->addPoint(bb.MinX, bb.MaxY);
		linearRing->addPoint(bb.MaxX, bb.MaxY);
		linearRing->addPoint(bb.MaxX, bb.MinY);
		linearRing->addPoint(bb.MinX, bb.MinY);

		poPolygon->addRing(linearRing);//also crashed
		return poPolygon;
}

OGRPolygon* toOGRPolygon(OGRLayer* layer, const osg::BoundingBoxd& bb)
{
		OGREnvelope ogrBB;
		ogrBB.MaxX = bb.xMax();
		ogrBB.MaxY = bb.yMax();
		ogrBB.MinX = bb.xMin();
		ogrBB.MinY = bb.yMin();
		return toOGRPolygon(layer, ogrBB);
}

osg::Node* loadModels(std::string file)
{
	return osgDB::readNodeFile(file);
}

osg::Node* loadModels(std::vector<std::string> files)
{
	//if (files.size() == 1)
	//{
	//	osg::Node* nd = osgDB::readNodeFile(files[0]);
	//	return nd;
	//}

	osg::Group* scene = new osg::Group;
	for (int i = 0; i<files.size(); i++)
	{
		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(files[i]);
		if (!node || !node.valid())
			continue;
		node->setName(files[i]);
		scene->addChild(node.get());
	}
	//scene->setInitialBound(bb);
	//scene->setCenter(bb.center());
	return scene;
}

osg::Node* ModelLoader::LoadModel(std::string path, bool& isIntegratedMesh)
{
		isIntegratedMesh = false;

		if(QFileInfo(path.data()).exists())
				return osgDB::readNodeFile(path);

		if (QDir(path.data()).exists())
		{
				osg::Node* node = Load3DTiles(path);
				if (node)
						isIntegratedMesh = true;
				return node;
		}

		return nullptr;
}

osg::Node* ModelLoader::Load3DTiles(std::string indir)
{
		std::vector<std::string> files = findMasterTiles(indir);
	 return loadModels(files);
}

osg::Node* ModelLoader::Load3DTiles(std::string indir, osg::BoundingBox mask, bool intersects)
{
		double area = (mask.xMax() - mask.xMin()) * (mask.yMax() - mask.yMin());
		std::vector<std::string> files;
		std::vector<std::string> dirs = findSubdirs(indir);
		for (size_t i = 0; i < dirs.size(); i++)
		{
				if (dirs[i].find("Tile_") == std::string::npos)
						continue;
				std::string masterTileName = getBaseName(dirs[i]);
				std::string masterTilePath = dirs[i] + masterTileName + ".osgb";

				osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(masterTilePath);
				osg::ComputeBoundsVisitor cbs;
				node->accept(cbs);
				osg::BoundingBox masterTileBB = cbs.getBoundingBox();
				bool intersectsBB = masterTileBB.intersects(mask);
				osg::BoundingBox intersection = masterTileBB.intersect(mask);
				if ((intersection.xMax() - intersection.xMin()) * (intersection.yMax() - intersection.yMin()) < area * 0.1)
				{
						intersectsBB = false;
				}
				if (intersectsBB != intersects)
						continue;
				files.push_back(masterTilePath);
		}
		return loadModels(files);
}

osg::Node* ModelLoader::Load3DTiles(std::string indir, std::vector<std::string> maskTiles, bool include)
{

		std::set<std::string> tileset;
		for (size_t i = 0; i < maskTiles.size(); i++)
		{
				tileset.insert(maskTiles[i]);
		}

		std::vector<std::string> files;
		std::vector<std::string> dirs = findSubdirs(indir);
		for (size_t i = 0; i < dirs.size(); i++)
		{
				if (dirs[i].find("Tile_") == std::string::npos)
						continue;
				std::string masterTileName = getBaseName(dirs[i]);
				std::string masterTilePath = dirs[i] + masterTileName + ".osgb";
				bool found = false;
				if (tileset.find(masterTileName) != tileset.end())
				{
						found = true;
				}
				if (found != include)
						continue;
				files.push_back(masterTilePath);
		}
		return loadModels(files);
}

void ModelLoader::CopyLeafTiles(std::string indir, std::string outdir, osg::BoundingBox bb)
{
		bb.zMin() = -10000;
		bb.zMax() = 100000;
		QDir(outdir.data()).mkpath(".");
		QString qoutdir = QDir(outdir.data()).absolutePath() + "/";
		std::vector<std::string> subdirs = findSubdirs(indir);
		for (size_t i = 0; i < subdirs.size(); i++)
		{
				auto subdir = subdirs[i];
				if (subdir.find("Tile_") == std::string::npos)
						continue;

				std::string masterTileName = getBaseName(subdir);
				std::string masterTilePath = subdir + masterTileName + ".osgb";
				osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(masterTilePath);
				osg::ComputeBoundsVisitor cbs;
				node->accept(cbs);
				osg::BoundingBox masterTileBB = cbs.getBoundingBox();
				if (!masterTileBB.intersects(bb))
						continue;

				std::vector<std::string> allTiles = findFiles(subdir, ".osgb");
				std::vector<std::string> leafTiles = findLeafTileFiles(allTiles, masterTileName);
				for (size_t i = 0; i < leafTiles.size(); i++)
				{
						auto leafTileFile = leafTiles[i];
						osg::ComputeBoundsVisitor tileCBS;
						node = osgDB::readNodeFile(leafTileFile);
						node->accept(tileCBS);
						if (!tileCBS.getBoundingBox().intersects(bb))
								continue;
						QFile::copy(leafTileFile.data(), qoutdir + QFileInfo(leafTileFile.data()).fileName());
						printf("%s\n", QFileInfo(leafTileFile.data()).fileName().toLocal8Bit().data());
				}
		}
}

void ModelLoader::CopyLeafTiles(std::string indir, std::vector<std::string> tilenames, std::string outfile)
{
		osg::ref_ptr<osg::Group> group = new osg::Group;
		for (size_t i = 0; i < tilenames.size(); i++)
		{
				auto tilename = tilenames[i];
				std::string subdir = indir + tilename + "/";
				if (subdir.find("Tile_") == std::string::npos)
						continue;

				//std::string masterTileName = getBaseName(subdir);
				//std::string masterTilePath = subdir + masterTileName + ".osgb";
			//	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(masterTilePath);
				std::vector<std::string> allTiles = findFiles(subdir, ".osgb");
				std::vector<std::string> leafTiles = findLeafTileFiles(allTiles, tilename);
				for (size_t i = 0; i < leafTiles.size(); i++)
				{
						auto leafTileFile = leafTiles[i];
						osg::ComputeBoundsVisitor tileCBS;
						osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(leafTileFile);
						osgDB::writeNodeFile(*node, "ljm.osg");
						node = osgDB::readNodeFile("ljm.osg");
						group->addChild(node.get());
						//QFile::copy(leafTileFile.data(), qoutdir + QFileInfo(leafTileFile.data()).fileName());
						printf("%s\n", QFileInfo(leafTileFile.data()).fileName().toLocal8Bit().data());
				}
		}
		osgUtil::SmoothingVisitor smooth;
		group->accept(smooth);
		osgDB::writeNodeFile(*group, outfile);
}

osg::BoundingBoxd ModelLoader::CalBound(std::string indir, const std::vector<std::string>& tiles)
{
		osg::BoundingBoxd bb;
		bb.init();
		for (size_t i = 0; i < tiles.size(); i++)
		{
				std::string masterTilePath = indir + tiles[i] + "/" + tiles[i] + ".osgb";
				osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(masterTilePath);
				osg::ComputeBoundsVisitor cbs;
				node->accept(cbs);
				osg::BoundingBoxd tileBB = cbs.getBoundingBox();
				bb.expandBy(tileBB);
		}
		return bb;
}

void getBlockStats(std::string filename, BBWrapper bb)
{
		int startCol, startRow, startIndex, endCol, endRow, endIndex;
		GDAL_DS<float>* dsm = new GDAL_DS<float>();
		dsm->open(filename, GA_Update);
		std::tie(startCol, startRow, startIndex) = dsm->getCellIndex(bb.xMin(), bb.yMax());
		std::tie(endCol, endRow, endIndex) = dsm->getCellIndex(bb.xMax(), bb.yMin());
		if (startCol < 0)
				startCol = 0;
		if (startRow < 0)
				startRow = 0;
		if (endCol > dsm->ncols - 1)
				endCol = dsm->ncols - 1;
		if (endRow > dsm->nrows - 1)
				endRow = dsm->nrows - 1;

		double blockXSize = endCol - startCol + 1;
		double blockYSize = endRow - startRow + 1;
		float* blockBuf = new float[blockXSize * blockYSize];
		dsm->m_dataset->GetRasterBand(1)->RasterIO(GF_Read, startCol, startRow, blockXSize, blockYSize, blockBuf, blockXSize, blockYSize, dsm->getType(), 0, 0);
		float min = 1000000;
		float max = -min;
		float* pValue = blockBuf;
		double nodata = dsm->getNoData(1);
		for (size_t i = 0; i < blockXSize * blockYSize; i++)
		{
				float val = *pValue;
				if (val == nodata)
				{
						pValue++;
						continue;
				}
		   
				if (min > val)
						min = val;
				if (max < val)
						max = val;

				pValue++;
		}
		printf("min=%f,max=%f\n", min, max);
		delete dsm;
}

void ModelLoader::Test()
{
		AABBox aabb(osg::Vec3d(-0.5, -0.5, 0.0), osg::Vec3d(0.5, 0.5, 0.0));
		Ray ray(osg::Vec3d(0.0, 0.0, 0.0), osg::Vec3d(1.0, 1.0, 0.0));
		osg::Vec3d dir(osg::Vec3d(0.5, 0.5, 0.0) - osg::Vec3d(1.0, 0.5, 0.0));
		dir.normalize();
		ray = Ray(osg::Vec3d(1.0, 0.5, 0.0), dir);
		double dist;
		bool intersects = aabb.intersect(ray, dist);
		intersects = aabb.intersect(Ray(osg::Vec3d(1.0, 0.5, 5.0), osg::Vec3d(5.0, 5.0, 5.0)));
		std::vector<std::string> tilenames;
		tilenames.push_back("Tile_-002_-016");
		tilenames.push_back("Tile_-002_-017");
		tilenames.push_back("Tile_-003_-016");
		tilenames.push_back("Tile_-003_-017");
		BBWrapper bb = CalBound("E:/Data/weihai/Data/", tilenames);
		
		double bbScale = 4;
		BBWrapper expandedBB(bb.center().x() - bb.xhalfsize() * bbScale, bb.center().y() - bb.yhalfsize() * bbScale, bb.zMin(),
				bb.center().x() + bb.xhalfsize() * bbScale, bb.center().y() + bb.yhalfsize() * bbScale, bb.zMax());

		//getBlockStats("E:/Code/Weihai_DSM_025.tif", expandedBB);
		//getBlockStats("E:/Code/Weihai_DSM_050.tif", expandedBB);
		//getBlockStats("E:/Code/Weihai_DSM_075.tif", expandedBB);
		//getBlockStats("E:/Code/Weihai_DSM_1m.tif", expandedBB);
		int startCol, startRow, startIndex, endCol, endRow, endIndex;
		GDAL_DS<float>* dsm = new GDAL_DS<float>();
		dsm->open("E:/Code/Weihai_DSM_025.tif", GA_Update);
		std::tie(startCol, startRow, startIndex) = dsm->getCellIndex(expandedBB.xMin(), expandedBB.yMax());
		std::tie(endCol, endRow, endIndex) = dsm->getCellIndex(expandedBB.xMax(), expandedBB.yMin());
		if (startCol < 0)
				startCol = 0;
		if (startRow < 0)
				startRow = 0;
		if (endCol > dsm->ncols - 1)
				endCol = dsm->ncols - 1;
		if (endRow > dsm->nrows - 1)
				endRow = dsm->nrows - 1;

		//dsm->downSampleMaximum("E:/Code/Weihai_DSM_075.tif", 3);
	 double blockXSize = endCol - startCol + 1;
		double blockYSize = endRow - startRow + 1;
		float* blockBuf = new float[blockXSize * blockYSize];
		dsm->m_dataset->GetRasterBand(1)->RasterIO(GF_Read, startCol, startRow, blockXSize, blockYSize, blockBuf, blockXSize, blockYSize, dsm->getType(), 0, 0);
		double sum = 0;
		double count = 0;
		float* pValue = blockBuf;
		double nodata = dsm->getNoData(1);
		for (size_t i = 0; i < blockXSize * blockYSize; i++)
		{
				float val = *pValue;
				if (val == nodata)
				{
						pValue++;
						continue;
				}
				sum += val;
				count += 1;
				pValue++;
		}

		printf("%f\n", sum / count);
}

void ModelLoader::TileBoundary2Shapefile(std::string indir, std::string outfile)
{
		ShapeFile tileMap;
		tileMap.create(outfile);
		OGRFeatureDefn *poFDefn = tileMap.poLayer->GetLayerDefn();
		int tileNameIndex = tileMap.getOrCreateField("Name", OGRFieldType::OFTString);
		int tileMinXIndex = tileMap.getOrCreateField("MinX", OGRFieldType::OFTReal);
		int tileMaxXIndex = tileMap.getOrCreateField("MaxX", OGRFieldType::OFTReal);
		int tileMinYIndex = tileMap.getOrCreateField("MinY", OGRFieldType::OFTReal);
		int tileMaxYIndex = tileMap.getOrCreateField("MaxY", OGRFieldType::OFTReal);
		int tileFilePathIndex = tileMap.getOrCreateField("Path", OGRFieldType::OFTString);

		std::vector<std::string> files;
		std::vector<std::string> dirs = findSubdirs(indir);
		for (size_t i = 0; i < dirs.size(); i++)
		{
				if (dirs[i].find("Tile_") == std::string::npos)
						continue;
				std::string masterTileName = getBaseName(dirs[i]);
				std::string masterTilePath = dirs[i] + masterTileName + ".osgb";
				osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(masterTilePath);
				osg::ComputeBoundsVisitor cbs;
				node->accept(cbs);
				osg::BoundingBoxd masterTileBB = cbs.getBoundingBox();
				if (!masterTileBB.valid())
						continue;
				if(masterTileBB.xMin() < -100000000 || masterTileBB.xMax() > 100000000 || masterTileBB.yMin() < -100000000 || masterTileBB.yMax() > 100000000)
						continue;
				OGRFeature* poFeaPolygon = OGRFeature::CreateFeature(tileMap.poLayer->GetLayerDefn());
				poFeaPolygon->SetField(tileNameIndex, masterTileName.data());
				poFeaPolygon->SetField(tileMinXIndex, masterTileBB.xMin());
				poFeaPolygon->SetField(tileMaxXIndex, masterTileBB.xMax());
				poFeaPolygon->SetField(tileMinYIndex, masterTileBB.yMin());
				poFeaPolygon->SetField(tileMaxYIndex, masterTileBB.yMax());
				poFeaPolygon->SetField(tileFilePathIndex, masterTilePath.data());
				OGRPolygon *poPolygon = toOGRPolygon(tileMap.poLayer, masterTileBB);
				poFeaPolygon->SetGeometry(poPolygon);
				tileMap.poLayer->CreateFeature(poFeaPolygon);
				OGRFeature::DestroyFeature(poFeaPolygon);
		}
		tileMap.close();
}