#pragma once
#include "gdal_priv.h"
#include "gdal_rat.h"
#include "ogrsf_frmts.h"
#include <osg/Image>
#include <osgDB/WriteFile>
#include <osg/Vec3>
#include <osg/Vec2>
#include <osg/Vec2d>
#include <osg/Vec3d>
#include <math.h>
#include <cmath>
#include "TypeDef.h"

class GDAL_DSInfo
{
public:
	OGREnvelope bound;
	size_t nrows;
	size_t ncols;
	size_t slice;
	int numbands;

//adfGeoTransform[0] /* top left x */
//adfGeoTransform[1] /* w-e pixel resolution */
//adfGeoTransform[2] /* 0 */
//adfGeoTransform[3] /* top left y */
//adfGeoTransform[4] /* 0 */
//adfGeoTransform[5] /* n-s pixel resolution (negative value) */
	double adfGeoTransform[6];
	double nodata;
	std::string filename;
	std::string pszFormat;
	std::string projection;
	GDAL_DSInfo()
	{
			pszFormat = "GTiff";
	}
};

template <class T>
class GDAL_DS : public GDAL_DSInfo
{
public:
		GDAL_DS()
		{
				m_dataset = nullptr;
				m_cache = nullptr;
				pszFormat = "GTiff";
		}

		~GDAL_DS()
		{
				close();
		}

		bool isInside(double x, double y)
		{
				if (x >= bound.MinX && x <= bound.MaxX && y >= bound.MinY && y <= bound.MaxY)
						return true;
				return false;
		}

		bool open(std::string fname, GDALAccess mode = GA_ReadOnly)
		{
				close();
				filename = fname;
				m_dataset = (GDALDataset*)GDALOpen(filename.data(), mode);
				if (!m_dataset)
						return false;
				m_dataset->GetGeoTransform(adfGeoTransform);
				ncols = m_dataset->GetRasterXSize();
				nrows = m_dataset->GetRasterYSize();
				slice = nrows * ncols;
				numbands = m_dataset->GetRasterCount();
				projection = m_dataset->GetProjectionRef();
				setGeoTransform(adfGeoTransform);
		}

		void create(std::string fname)
		{
				close();
				filename = fname;
				char **papszOptions = NULL;
				GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat.data());
				poDriver->Delete(filename.data());
				m_dataset = poDriver->Create(filename.data(), ncols, nrows, numbands, getType(), papszOptions);
				m_dataset->SetGeoTransform(adfGeoTransform);
				m_dataset->SetProjection(projection.data());
		}

		void setGeoTransform(double* transform)
		{
				memcpy(adfGeoTransform, transform, sizeof(double) * 6);
				bound.MinX = adfGeoTransform[0];
				bound.MaxY = adfGeoTransform[3];
				bound.MaxX = adfGeoTransform[0] + adfGeoTransform[1] * ncols;
				bound.MinY = adfGeoTransform[3] + adfGeoTransform[5] * nrows;
				if (m_dataset)
						m_dataset->SetGeoTransform(transform);
		}

		T* readData(int bandidx)
		{
				if (!m_dataset)
						return NULL;
				T* data = new T[slice];
				GDALDataType gdt = getType();
				m_dataset->GetRasterBand(bandidx)->RasterIO(GF_Read, 0, 0, ncols, nrows, data, ncols, nrows, gdt, 0, 0);
				nodata = getNoData(bandidx);
				m_nodata = nodata;
				return data;
		}

		void readData(int bandidx, T* data)
		{
				if (!m_dataset)
						return;
				GDALDataType gdt = getType();
				m_dataset->GetRasterBand(bandidx)->RasterIO(GF_Read, 0, 0, ncols, nrows, data, ncols, nrows, gdt, 0, 0);
		}

		void writeData(int bandidx, T* data, double nodata)
		{
				GDALRasterBand *pBand = m_dataset->GetRasterBand(bandidx);
				pBand->RasterIO(GF_Write, 0, 0, ncols, nrows, data, ncols, nrows, getType(), 0, 0);
				pBand->SetNoDataValue(nodata);
		}

		double sum(int bandindex)
		{
				T* data = readData(bandindex);
				T nodata = (T)getNoData(bandindex);
				T* pdata = data;
				double sum = 0;
				for (size_t i = 0; i < slice; i++)
				{
						if (*pdata != nodata)
						{
								sum += *pdata;
						}
						pdata++;
				}
				delete[] data;
				return sum;
		}

		void multiply(T factor, int bandindex)
		{
				T* data = readData(bandindex);
				T nodata = (T)getNoData(bandindex);
				T* pdata = data;
				double sum = 0;
				for (size_t i = 0; i < slice; i++)
				{
						if (*pdata != nodata)
						{
								*pdata = *pdata * factor;
						}
						pdata++;
				}
				writeData(bandindex, data, nodata);
				delete[] data;
		}

		double getNoData(int bandidx)
		{
				return m_dataset->GetRasterBand(bandidx)->GetNoDataValue();
		}

		void setNoData(int bandidx, double nodata)
		{
				m_dataset->GetRasterBand(bandidx)->SetNoDataValue(nodata);
		}

		void close()
		{
				if (m_dataset)
						GDALClose(m_dataset);
				if (m_cache)
						delete[] m_cache;
				m_dataset = nullptr;
				m_cache = nullptr;
		}

		GDALDataType getType()
		{
				if (typeid(T) == typeid(unsigned char) || typeid(T) == typeid(char))
						return GDT_Byte;
				if (typeid(T) == typeid(unsigned short))
						return GDT_UInt16;
				if (typeid(T) == typeid(short))
						return GDT_Int16;
				if (typeid(T) == typeid(unsigned int))
						return GDT_UInt32;
				if (typeid(T) == typeid(int))
						return GDT_Int32;
				if (typeid(T) == typeid(float))
						return GDT_Float32;
				if (typeid(T) == typeid(double))
						return GDT_Float64;
				return GDT_Unknown;
		}

		void setDSInfo(GDAL_DSInfo* info)
		{
				nrows = info->nrows;
				ncols = info->ncols;
				slice = info->slice;
				numbands = info->numbands;
				filename = info->filename;
				pszFormat = info->pszFormat;
				projection = info->projection;
				setGeoTransform(info->adfGeoTransform);
		}

		void crop(OGREnvelope bb, std::string outfile)
		{
				int startCol = getCol(bb.MinX);
				int startRow = getRow(bb.MaxY);
				int endCol = getCol(bb.MaxX);
				int endRow = getRow(bb.MinY);

				if (startCol < 0)
						startCol = 0;
				if (startRow < 0)
						startRow = 0;
				if (endCol > ncols - 1)
						endCol = ncols - 1;
				if (endRow > nrows - 1)
						endRow = nrows - 1;

				double blockXSize = endCol - startCol + 1;
				double blockYSize = endRow - startRow + 1;
				T* blockBuf = new T[blockXSize * blockYSize];
				m_dataset->GetRasterBand(1)->RasterIO(GF_Read, startCol, startRow, blockXSize, blockYSize, blockBuf, blockXSize, blockYSize, getType(), 0, 0);
				double nodata = getNoData(1);
				//destBB.MinX = bound.MinX + startCol * adfGeoTransform[1];
				//destBB.MaxY = bound.MaxY + startRow * adfGeoTransform[5];
				GDAL_DS* newdt = new GDAL_DS<T>();

				memcpy(newdt->adfGeoTransform, adfGeoTransform, sizeof(double) * 6);
				newdt->adfGeoTransform[0] = adfGeoTransform[0] + startCol * adfGeoTransform[1];
				newdt->adfGeoTransform[3] = adfGeoTransform[3] + startRow * adfGeoTransform[5];
				//newdt->bound = destBB;
				newdt->numbands = 1;
				newdt->ncols = blockXSize;
				newdt->nrows = blockYSize;
				newdt->projection = this->projection;
				newdt->pszFormat = this->pszFormat;
				newdt->slice = blockXSize * blockYSize;

				newdt->create(outfile);
				newdt->writeData(1, blockBuf, nodata);

				delete[] blockBuf;
				delete newdt;
		}

		void crop(int startCol, int startRow, int endCol, int endRow, std::string outfile)
		{
				if (startCol < 0)
						startCol = 0;
				if (startRow < 0)
						startRow = 0;
				if (endCol > ncols - 1)
						endCol = ncols - 1;
				if (endRow > nrows - 1)
						endRow = nrows - 1;

				double blockXSize = endCol - startCol + 1;
				double blockYSize = endRow - startRow + 1;
				T* blockBuf = new T[blockXSize * blockYSize];
				m_dataset->GetRasterBand(1)->RasterIO(GF_Read, startCol, startRow, blockXSize, blockYSize, blockBuf, blockXSize, blockYSize, getType(), 0, 0);
				double nodata = getNoData(1);
				//destBB.MinX = bound.MinX + startCol * adfGeoTransform[1];
				//destBB.MaxY = bound.MaxY + startRow * adfGeoTransform[5];
				GDAL_DS* newdt = new GDAL_DS<T>();

				memcpy(newdt->adfGeoTransform, adfGeoTransform, sizeof(double) * 6);
				newdt->adfGeoTransform[0] = adfGeoTransform[0] + startCol * adfGeoTransform[1];
				newdt->adfGeoTransform[3] = adfGeoTransform[3] + startCol * adfGeoTransform[5];
				//newdt->bound = destBB;
				newdt->numbands = 1;
				newdt->ncols = blockXSize;
				newdt->nrows = blockYSize;
				newdt->projection = this->projection;
				newdt->pszFormat = this->pszFormat;
				newdt->slice = blockXSize * blockYSize;

				newdt->create(outfile);
				newdt->writeData(1, blockBuf, nodata);

				delete[] blockBuf;
				delete newdt;
		}

		void crop(osg::BoundingBoxd bb, std::string outfile)
		{
				OGREnvelope envelop;
				envelop.MinX = bb.xMin();
				envelop.MaxX = bb.xMax();
				envelop.MinY = bb.yMin();
				envelop.MaxY = bb.yMax();
				crop(envelop, outfile);
		}

	//<col, row, index>
		std::tuple<int, int, int> getCellIndex(double x, double y)
		{
				if(x < bound.MinX || x > bound.MaxX || y < bound.MinY ||  y > bound.MaxY)
						return std::make_tuple(-1, -1, -1);
				int col = (int)((x - bound.MinX) / adfGeoTransform[1]);
				int row = (int)((y - bound.MaxY) / adfGeoTransform[5]);
				int index = row * ncols + col;
				return std::make_tuple(col, row, index);
		}

		int getCol(double x)
		{
				if (x < bound.MinX || x > bound.MaxX)
						return -1;
				int col = (int)((x - bound.MinX) / adfGeoTransform[1]);
				if (col < 0)
						col = 0;
				else if (col > ncols - 1)
						col = ncols - 1;
				return col;
		}

		int getRow(double y)
		{
				if (y < bound.MinY || y > bound.MaxY)
						return -1;
				int row = (int)((y - bound.MaxY) / adfGeoTransform[5]);
				if (row < 0)
						row = 0;
				else if (row > nrows - 1)
						row = nrows - 1;
				return row;
		}

		std::tuple<int, int, int> getCellIndexCached(double x, double y)
		{
				if (x < m_CacheBound.xMin() || x > m_CacheBound.xMax() || y < m_CacheBound.yMin() || y > m_CacheBound.yMax())
						return std::make_tuple(-1, -1, -1);
				int col = (int)((x - m_CacheBound.xMin()) / adfGeoTransform[1]);
				int row = (int)((y - m_CacheBound.yMax()) / adfGeoTransform[5]);
				int index = row * m_cacheCols + col;
				return std::make_tuple(col, row, index);
		}

		T getCellValueCached(double x, double y)
		{
			if (x < m_CacheBound.xMin() || x > m_CacheBound.xMax() || y < m_CacheBound.yMin() || y > m_CacheBound.yMax())
				return m_nodata;
			int col = (int)((x - m_CacheBound.xMin()) / adfGeoTransform[1]);
			int row = (int)((y - m_CacheBound.yMax()) / adfGeoTransform[5]);
			int index = row * m_cacheCols + col;
			return m_cache[index];
		}

		void downSampleMaximum(std::string outfile, int factor)
		{
				double resolX = adfGeoTransform[1] * factor;
				double resolY = adfGeoTransform[5] * factor;

				GDAL_DS* newdt = new GDAL_DS<T>();
				newdt->ncols = (int)(ceil((double)ncols / (double)factor));
				newdt->nrows = (int)(ceil((double)nrows / (double)factor));
				newdt->bound.MinX = bound.MinX;
				newdt->bound.MaxY = bound.MaxY;
				newdt->bound.MaxX = bound.MinX + newdt->ncols * resolX;
				newdt->bound.MinY = bound.MaxY + newdt->nrows * resolY;
				memcpy(newdt->adfGeoTransform, adfGeoTransform, sizeof(double) * 6);
				newdt->adfGeoTransform[1] = resolX;
				newdt->adfGeoTransform[5] = resolY;
				newdt->setGeoTransform(newdt->adfGeoTransform);
				newdt->numbands = 1;
				newdt->projection = this->projection;
				newdt->pszFormat = this->pszFormat;
				newdt->slice = newdt->ncols * newdt->nrows;

				int bufXSize = factor * 1000;
				int bufYSize = factor * 1000;
				int bufSize = bufXSize * bufYSize;
				T* srcBlockBuf = new T[bufSize];
				T* destBlockBuf = new T[1000 * 1000];
				T* destdata = new T[newdt->ncols * newdt->nrows];
				int yoffset = 0;
				double nodata = this->getNoData(1);
				T* pDestdata = destdata;
				for (int i = 0; i < newdt->slice; i++)
				{
						*pDestdata = nodata;
						pDestdata++;
				}
				while (true)
				{
						int xoffset = 0;
						int blockYSize = MIN(bufYSize, nrows - yoffset);
						int blockYSizeIO = blockYSize;
						if (blockYSize < bufYSize) 
						{
								blockYSize = (int)(ceil((double)blockYSize / (double)factor)) * factor;
						}
						while (true)
						{
								int blockXSize = MIN(bufXSize, ncols - xoffset);
								int blockXSizeIO = blockXSize;
								if (blockXSize < bufXSize)
								{
										blockXSize = (int)(ceil((double)blockXSize / (double)factor)) * factor;
								}
								T* srcBlockBufTemp = nullptr;
								T* pSrcBlockBuf = srcBlockBuf;
								if (blockXSizeIO != bufXSize || blockYSizeIO != bufYSize)
								{
										srcBlockBufTemp = new T[blockYSizeIO * blockXSizeIO];
										pSrcBlockBuf = srcBlockBufTemp;
										for (int i = 0; i < blockXSizeIO * blockYSizeIO; i++)
										{
												*pSrcBlockBuf = nodata;
												pSrcBlockBuf++;
										}
										pSrcBlockBuf = srcBlockBufTemp;
								}
								m_dataset->GetRasterBand(1)->RasterIO(GF_Read, xoffset, yoffset, blockXSizeIO, blockYSizeIO, pSrcBlockBuf, blockXSizeIO, blockYSizeIO, getType(), 0, 0);
	
								if (blockXSize != blockXSizeIO || blockYSize != blockYSizeIO)
								{
										T* srcBlockBufTemp2 = new T[blockYSize * blockXSize];
										for (int y = 0; y < blockYSize; y++)
										{
												for (int x = 0; x < blockXSize; x++)
												{
														int srcIndex = x + y * blockXSizeIO;
														int destIndex = x + y * blockXSize;
														if (x >= blockXSizeIO || y >= blockYSizeIO)
														{
																srcBlockBufTemp2[destIndex] = nodata;
														}
														else
														{
																srcBlockBufTemp2[destIndex] = pSrcBlockBuf[srcIndex];
														}
												}
										}
										delete[] pSrcBlockBuf;
										pSrcBlockBuf = srcBlockBufTemp2;
										srcBlockBufTemp = srcBlockBufTemp2;
								}

								for (int y = 0; y < blockYSize / factor; y++)
								{
										for (int x = 0; x < blockXSize / factor; x++)
										{
												float windowMax = -1000000;
												for (int windowy = 0; windowy < factor; windowy++)
												{
														for (int windowx = 0; windowx < factor; windowx++)
														{
																float val = pSrcBlockBuf[(y * factor + windowy) * blockXSize + (x * factor + windowx)];
																if (windowMax < val)
																		windowMax = val;
														}
												}
												if (windowMax <= -1000000)
														windowMax = nodata;
												destBlockBuf[y * 1000 + x] = windowMax;
										}
								}

								int curY = 0;
								for (int y = yoffset / factor; y < yoffset / factor + 1000; y++)
								{
										int curX = 0;
										for (int x = xoffset / factor; x < xoffset / factor + 1000; x++)
										{
												if (y < newdt->nrows && x < newdt->ncols)
												{
														destdata[y * newdt->ncols + x] = destBlockBuf[curY * 1000 + curX];
												}
												curX++;
										}
										curY++;
								}
								if (srcBlockBufTemp)
										delete[] srcBlockBufTemp;

								xoffset += bufXSize;
								if (xoffset >= ncols)
										break;
						}
						yoffset += bufYSize;
						if (yoffset >= nrows)
								break;
						printf("%d/%d\n", yoffset, nrows);
				}

				newdt->create(outfile);
				newdt->writeData(1, destdata, nodata);

				delete[] srcBlockBuf;
				delete[] destBlockBuf;
				delete[] destdata;
				delete newdt;
		}

		void downSampleMinimum(std::string outfile, int factor)
		{
				double resolX = adfGeoTransform[1] * factor;
				double resolY = adfGeoTransform[5] * factor;

				GDAL_DS* newdt = new GDAL_DS<T>();
				newdt->ncols = (int)(ceil((double)ncols / (double)factor));
				newdt->nrows = (int)(ceil((double)nrows / (double)factor));
				newdt->bound.MinX = bound.MinX;
				newdt->bound.MaxY = bound.MaxY;
				newdt->bound.MaxX = bound.MinX + newdt->ncols * resolX;
				newdt->bound.MinY = bound.MaxY + newdt->nrows * resolY;
				memcpy(newdt->adfGeoTransform, adfGeoTransform, sizeof(double) * 6);
				newdt->adfGeoTransform[1] = resolX;
				newdt->adfGeoTransform[5] = resolY;
				newdt->setGeoTransform(newdt->adfGeoTransform);
				newdt->numbands = 1;
				newdt->projection = this->projection;
				newdt->pszFormat = this->pszFormat;
				newdt->slice = newdt->ncols * newdt->nrows;

				int bufXSize = factor * 1000;
				int bufYSize = factor * 1000;
				int bufSize = bufXSize * bufYSize;
				T* srcBlockBuf = new T[bufSize];
				T* destBlockBuf = new T[1000 * 1000];
				T* destdata = new T[newdt->ncols * newdt->nrows];
				int yoffset = 0;
				double nodata = this->getNoData(1);
				T* pDestdata = destdata;
				for (int i = 0; i < newdt->slice; i++)
				{
						*pDestdata = nodata;
						pDestdata++;
				}
				while (true)
				{
						int xoffset = 0;
						int blockYSize = MIN(bufYSize, nrows - yoffset);
						int blockYSizeIO = blockYSize;
						if (blockYSize < bufYSize)
						{
								blockYSize = (int)(ceil((double)blockYSize / (double)factor)) * factor;
						}
						while (true)
						{
								int blockXSize = MIN(bufXSize, ncols - xoffset);
								int blockXSizeIO = blockXSize;
								if (blockXSize < bufXSize)
								{
										blockXSize = (int)(ceil((double)blockXSize / (double)factor)) * factor;
								}
								T* srcBlockBufTemp = nullptr;
								T* pSrcBlockBuf = srcBlockBuf;
								if (blockXSizeIO != bufXSize || blockYSizeIO != bufYSize)
								{
										srcBlockBufTemp = new T[blockYSizeIO * blockXSizeIO];
										pSrcBlockBuf = srcBlockBufTemp;
										for (int i = 0; i < blockXSizeIO * blockYSizeIO; i++)
										{
												*pSrcBlockBuf = nodata;
												pSrcBlockBuf++;
										}
										pSrcBlockBuf = srcBlockBufTemp;
								}
								m_dataset->GetRasterBand(1)->RasterIO(GF_Read, xoffset, yoffset, blockXSizeIO, blockYSizeIO, pSrcBlockBuf, blockXSizeIO, blockYSizeIO, getType(), 0, 0);

								if (blockXSize != blockXSizeIO || blockYSize != blockYSizeIO)
								{
										T* srcBlockBufTemp2 = new T[blockYSize * blockXSize];
										for (int y = 0; y < blockYSize; y++)
										{
												for (int x = 0; x < blockXSize; x++)
												{
														int srcIndex = x + y * blockXSizeIO;
														int destIndex = x + y * blockXSize;
														if (x >= blockXSizeIO || y >= blockYSizeIO)
														{
																srcBlockBufTemp2[destIndex] = nodata;
														}
														else
														{
																srcBlockBufTemp2[destIndex] = pSrcBlockBuf[srcIndex];
														}
												}
										}
										delete[] pSrcBlockBuf;
										pSrcBlockBuf = srcBlockBufTemp2;
										srcBlockBufTemp = srcBlockBufTemp2;
								}

								for (int y = 0; y < blockYSize / factor; y++)
								{
										for (int x = 0; x < blockXSize / factor; x++)
										{
												float windowMin = 1000000000;
												for (int windowy = 0; windowy < factor; windowy++)
												{
														for (int windowx = 0; windowx < factor; windowx++)
														{
																float val = pSrcBlockBuf[(y * factor + windowy) * blockXSize + (x * factor + windowx)];
																if (val != nodata && windowMin > val)
																		windowMin = val;
														}
												}
												if (windowMin <= -100 || windowMin > 10000)
														windowMin = nodata;
												destBlockBuf[y * 1000 + x] = windowMin;
										}
								}

								int curY = 0;
								for (int y = yoffset / factor; y < yoffset / factor + 1000; y++)
								{
										int curX = 0;
										for (int x = xoffset / factor; x < xoffset / factor + 1000; x++)
										{
												if (y < newdt->nrows && x < newdt->ncols)
												{
														destdata[y * newdt->ncols + x] = destBlockBuf[curY * 1000 + curX];
												}
												curX++;
										}
										curY++;
								}
								if (srcBlockBufTemp)
										delete[] srcBlockBufTemp;

								xoffset += bufXSize;
								if (xoffset >= ncols)
										break;
						}
						yoffset += bufYSize;
						if (yoffset >= nrows)
								break;
						printf("%d/%d\n", yoffset, nrows);
				}

				newdt->create(outfile);
				newdt->writeData(1, destdata, nodata);

				delete[] srcBlockBuf;
				delete[] destBlockBuf;
				delete[] destdata;
				delete newdt;
		}

		void readCache(osg::BoundingBoxd bb)
		{
				if (bb.xMin() != FLT_MAX && bb.xMax() != -FLT_MAX)
				{
						m_CacheBound = bb;
						int startCol, startRow, startIndex, endCol, endRow, endIndex;
						std::tie(startCol, startRow, startIndex) = getCellIndex(bb.xMin(), bb.yMax());
						std::tie(endCol, endRow, endIndex) = getCellIndex(bb.xMax(), bb.yMin());
						if (startCol < 0)
								startCol = 0;
						if (startRow < 0)
								startRow = 0;
						if (endCol >ncols - 1)
								endCol = ncols - 1;
						if (endRow > nrows - 1)
								endRow = nrows - 1;

						double blockXSize = endCol - startCol + 1;
						double blockYSize = endRow - startRow + 1;
						float* blockBuf = new float[blockXSize * blockYSize];
				 	m_dataset->GetRasterBand(1)->RasterIO(GF_Read, startCol, startRow, blockXSize, blockYSize, blockBuf, blockXSize, blockYSize, getType(), 0, 0);
						m_cache = blockBuf;
						m_cacheRows = blockYSize;
						m_cacheCols = blockXSize;
				}
				else
				{
						m_CacheBound = osg::BoundingBoxd(bound.MinX, bound.MinY, 0, bound.MaxX, bound.MaxY, 1);
						m_cache = readData(1);
						m_cacheRows = nrows;
						m_cacheCols = ncols;
				}
				m_nodata = getNoData(1);
		}
public:
	//OGREnvelope bound;
	//int nrows;
	//int ncols;
	//int slice;
	//int numbands;
	//double adfGeoTransform[6];
	//std::string filename;
	GDALDataset* m_dataset;
	double m_nodata;
	osg::BoundingBoxd m_CacheBound;
	T* m_cache;
	int m_cacheRows;
	int m_cacheCols;
private:

};

//typedef enum {
//	/*! Unknown Orange unspecified type */          GDT_Unknown = 0,
//	/*! Eight bit unsigned integer */           GDT_Byte = 1,
//	/*! Sixteen bit unsigned integer */         GDT_UInt16 = 2,
//	/*! Sixteen bit signed integer */           GDT_Int16 = 3,
//	/*! Thirty two bit unsigned integer */      GDT_UInt32 = 4,
//	/*! Thirty two bit signed integer */        GDT_Int32 = 5,
//	/*! Thirty two bit floating point */        GDT_Float32 = 6,
//	/*! Sixty four bit floating point */        GDT_Float64 = 7,
//	/*! Complex Int16 */                        GDT_CInt16 = 8,
//	/*! Complex Int32 */                        GDT_CInt32 = 9,
//	/*! Complex Float32 */                      GDT_CFloat32 = 10,
//	/*! Complex Float64 */                      GDT_CFloat64 = 11,
//	GDT_TypeCount = 12          /* maximum type # + 1 */
//} GDALDataType;
template <class T>
class SVFComputer
{
public:
	GDAL_DS<T>* g_pDS;
	T* g_pData;
	osg::Vec4i skyColor;
	osg::Vec4i greenColor;
	osg::Vec4i outsideColor;
	SVFComputer(GDAL_DS<T>* ds) {
			g_pDS = ds;
			g_pData = ds->readData(1);
	}
	SVFComputer() {
		g_pDS = NULL;
		g_pData = NULL;
	}
	~SVFComputer()
	{
		delete[] g_pData;
	}
	void setData(GDAL_DS<T>* ds)
	{
		if (g_pData)
			delete[] g_pData;
		g_pDS = ds;
		g_pData = ds->readData(1);
	}
	double maxHorizon(osg::Vec3d eye, double azimuthAngle)
	{
		azimuthAngle = azimuthAngle * 3.1415926 / 180;
		osg::Vec2d dir(sin(azimuthAngle), cos(azimuthAngle));
		return maxHorizon(eye, dir);		
	}
	double maxHorizon(osg::Vec3d eye, osg::Vec2d dir)
	{
		osg::Vec2d curpos(eye.x(), eye.y());
		osg::Vec2d eyepos = curpos;
		double step = g_pDS->adfGeoTransform[1] * 0.5;
		OGREnvelope bb = g_pDS->bound;
		double resol = g_pDS->adfGeoTransform[1];
		double maxhorizon = -180;
		double dist = 0;
		double h = 0;
		while (true)
		{
			curpos = curpos + dir * step;
			dist += step;
			if (dist > 1)
			{
				int irow = (int)(bb.MaxY - curpos.y()) / resol;
				int icol = (int)(curpos.x() - bb.MinX) / resol;
				if (irow < 0 || irow > g_pDS->nrows - 1 || icol < 0 || icol > g_pDS->ncols - 1)
					break;
				double run = (eyepos - curpos).length();
				T height = g_pData[icol + irow*g_pDS->ncols];
				double rise = height - eye.z();
				double horizon = atan2(rise, run) * 180 / 3.1415926;
				if (horizon > maxhorizon)
				{
					maxhorizon = horizon;
					h = height;
				}
			}


		}
		if (maxhorizon < 0)
			maxhorizon = 0;
		if (maxhorizon > 90)
			maxhorizon = 90;
		printf("%f,%f,%f,%f\n", dir.x(), dir.y(), h, maxhorizon);
		return maxhorizon;
	}
	std::vector<osg::Vec2d> computeSkymap(osg::Vec3d eye, double numsteps)
	{
		std::vector<osg::Vec2d> coords;
		double step = 360.0 / numsteps;
		for (size_t i = 0; i < numsteps; i++)
		{
			double azimuth = step * i;
			double azimuthInRadians = azimuth * 3.1415926 / 180;
			osg::Vec2d dir(sin(azimuthInRadians), cos(azimuthInRadians));
			double maxhorizon = maxHorizon(eye, dir);
			double len = (90.0 - maxhorizon) / 90.0;
			osg::Vec2d pos(dir.x()*len, dir.y()*len);
			coords.push_back(pos);
		}
		return coords;
	}
	void drawSkymap(std::vector<osg::Vec2d> points, int imagesize, std::string outfile)
	{
		QImage* qImage = new QImage(imagesize, imagesize, QImage::Format::Format_ARGB32);
		//QImage* qImage2 = new QImage(imagesize, imagesize, QImage::Format::Format_ARGB32);
		qImage->fill(qRgba(0, 0, 0, 0));
		//qImage2->fill(qRgba(0, 0, 0, 0));
		QPainter painter(qImage);
		painter.begin(qImage);
		//painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
		//painter.setRenderHint(QPainter::Antialiasing); //make it look nicer
		QPointF center(imagesize*0.5, imagesize*0.5);
	
		//QTransform transform = QTransform().scale(imagesize*0.5, imagesize*0.5).translate(imagesize*0.5, imagesize*0.5);
		//QTransform transform = QTransform().scale(imagesize*0.5, imagesize*0.5)
		QTransform transform = QTransform().scale(imagesize*0.5, imagesize*0.5) * QTransform().translate(imagesize*0.5, imagesize*0.5);
		painter.setTransform(transform);
		painter.setBrush(QBrush(qRgba(255, 0, 0, 255)));
		painter.setPen(Qt::NoPen);
		painter.drawEllipse(QPointF(0, 0), 1, 1);

		QPainterPath qpath;
		QPolygonF qpolygon;
		for (size_t i = 0; i < points.size(); i++)
		{
			osg::Vec2d pt = points[i];
			qpolygon.push_back(QPointF(pt.x(), pt.y()));
		}
		qpolygon.push_back(QPointF(points[0].x(), points[0].y()));
		qpath.addPolygon(qpolygon);
		//painter.setPen(Qt::NoPen);
		QPen pen(qRgba(0, 0, 0, 0));
		pen.setWidth(0);
		painter.setPen(pen);
		painter.fillPath(qpath, QBrush(qRgba(0, 255, 0, 255)));
		painter.end();
		QString str = outfile.data();
		std::string fix = "png";
		if (str.endsWith("jpg"))
			fix = "JPG";
		else if (str.endsWith("bmp"))
			fix = "bmp";
		//osg::ref_ptr<osg::Image> image = new osg::Image;
		//image->allocateImage(qImage->width(), qImage->height(), 1, 4, GL_BGRA, GL_UNSIGNED_BYTE);
		//The pixel format is always RGBA to support transparency
		//memset(image->srcdata(), 0, qImage->byteCount());
		//image->setImage(qImage->width(), qImage->height(), 1,
		//	4,
		//	GL_RGBA, GL_UNSIGNED_BYTE, //Why not GL_RGBA - QGIS bug?
		//	srcdata,
		//	osg::Image::USE_NEW_DELETE, 1);
		//memset(srcdata, 0, qImage->byteCount());
		//image->flipVertical();
		//osgDB::writeImageFile(*image, outfile.srcdata());
		int elementSize = 4;
		uchar* data = new uchar[qImage->width() * qImage->height() * elementSize];
		uchar* pdata = qImage->bits();
		for (size_t i = 0; i < qImage->height(); i++)
		{
			for (size_t j = 0; j < qImage->width(); j++)
			{
				//BGRA
				if (pdata[2] == 255)
					pdata[3] = 255;
				else if (pdata[1] == 255)
					pdata[3] = 128;
				else
					pdata[3] = 0;
				pdata += elementSize;
			}
		}

		for (size_t i = 0; i < qImage->height(); i++)
		{
			int oriLine = qImage->height() - i - 1;
			memcpy(data + i * qImage->width() * elementSize, qImage->bits() + oriLine*qImage->width() * elementSize, qImage->width() * elementSize);
		}
		memcpy(qImage->bits(), data, qImage->width() * qImage->height() * elementSize);
		delete[] data;
		qImage->save(str, fix.data());
		delete qImage;
	}
	T getHeightAt(double x, double y)
	{
		double resol = g_pDS->adfGeoTransform[1];
		int irow = (int)(g_pDS->bound.MaxY - y) / resol;
		int icol = (int)(x - g_pDS->bound.MinX) / resol;
		if (irow < 0 || irow > g_pDS->nrows - 1 || icol < 0 || icol > g_pDS->ncols - 1)
			return g_pDS->getNoData(1);
		return g_pData[icol + irow*g_pDS->ncols];
	}
	//double calSVF(bool applyLambert = false)
	//{
	//	unsigned int skypixels = 0;
	//	unsigned int nonskypixels = 0;
	//	unsigned int ncols = g_pDS->ncols;
	//	unsigned int nrows = g_pDS->nrows;
	//	unsigned int numpixels = ncols * nrows;
	//	if (ncols != nrows)
	//		return 0;
	//	T* srcdata = g_pDS->readData(4);
	//	double resol = 1.0 / nrows;
	//	double totalarea = 0;
	//	double skyarea = 0;
	//	double y = resol * 0.5 - 0.5;
	//	int npixel = 0;
	//	for (unsigned int row = 0; row < nrows; row++)
	//	{
	//		y += resol;
	//		double x = resol * 0.5 - 0.5;
	//		for (unsigned int col = 0; col < ncols; col++)
	//		{
	//			T a = srcdata[npixel];

	//			//printf("%d,", (int)a);
	//			npixel++;
	//			if (outsideColor.a() && r == greenColor.r() && g == greenColor.g() && b == greenColor.b()) {
	//				x += resol;
	//				continue;//outside
	//			}
	//			double zenithD = sqrt(x*x + y*y) * 90.0;//in degrees
	//			if (zenithD <= 0.000000001)
	//				zenithD = 0.000000001;
	//			double zenithR = zenithD * 3.1415926 / 180.0;
	//			double wproj = sin(zenithR) / (zenithD / 90);//weight for equal-areal projection
	//			if (applyLambert)
	//			{
	//				wproj = wproj * cos(zenithR);
	//			}
	//			totalarea += wproj;
	//			if (a == 255)
	//			{
	//				nonskypixels++;
	//			}
	//			else
	//			{
	//				skypixels++;
	//				skyarea += wproj;
	//			}
	//			x += resol;
	//		}

	//	}
	//	double svf = skyarea / totalarea;
	//	delete[] srcdata;
	//	return svf;
	//}
	double calSVF(bool applyLambert = false)
	{
		unsigned int skypixels = 0;
		unsigned int nonskypixels = 0;
		unsigned int ncols = g_pDS->ncols;
		unsigned int nrows = g_pDS->nrows;
		unsigned int numpixels = ncols * nrows;
		if (ncols != nrows)
			return 0;
		T* data = g_pDS->readData(4);
		T* rdata = g_pDS->readData(1);
		T* gdata = g_pDS->readData(2);
		T* bdata = g_pDS->readData(3);

		double resol = 1.0 / nrows;
		double totalarea = 0;
		double skyarea = 0;
		double y = resol * 0.5 - 0.5;
		int npixel = 0;
		for (unsigned int row = 0; row < nrows; row++)
		{
			y += resol;
			double x = resol * 0.5 - 0.5;
			for (unsigned int col = 0; col < ncols; col++)
			{
				T a = data[npixel];
				T r = rdata[npixel];
				T g = gdata[npixel];
				T b = bdata[npixel];
				//printf("%d,", (int)a);
				npixel++;
				if (a == outsideColor.a() && r == outsideColor.r() && g == outsideColor.g() && b == outsideColor.b()) {
					x += resol;
					continue;//outside
				}
				double zenithD = sqrt(x*x + y*y) * 90.0;//in degrees
				if (zenithD <= 0.000000001)
					zenithD = 0.000000001;
				double zenithR = zenithD * 3.1415926 / 180.0;
				double wproj = sin(zenithR) / (zenithD / 90);//weight for equal-areal projection
				if (applyLambert)
				{
					wproj = wproj * cos(zenithR);
				}
				totalarea += wproj;
				if ( g > 120)
				{
					printf("");
				}
				if (a == skyColor.a() && r == skyColor.r() && g == skyColor.g() && b == skyColor.b())
				{
					skypixels++;
					skyarea += wproj;
				}
				else
				{
					nonskypixels++;
				}
				x += resol;
			}

		}
		double svf = skyarea / totalarea;
		delete[] data;
		delete[] rdata;
		delete[] gdata;
		delete[] bdata;
		return svf;
	}
	double calGreenery(bool applyLambert = false)
	{
		unsigned int skypixels = 0;
		unsigned int nonskypixels = 0;
		unsigned int ncols = g_pDS->ncols;
		unsigned int nrows = g_pDS->nrows;
		unsigned int numpixels = ncols * nrows;
		if (ncols != nrows)
			return 0;
		T* data = g_pDS->readData(4);
		T* rdata = g_pDS->readData(1);
		T* gdata = g_pDS->readData(2);
		T* bdata = g_pDS->readData(3);

		double resol = 1.0 / nrows;
		double totalarea = 0;
		double skyarea = 0;
		double y = resol * 0.5 - 0.5;
		int npixel = 0;
		for (unsigned int row = 0; row < nrows; row++)
		{
			y += resol;
			double x = resol * 0.5 - 0.5;
			for (unsigned int col = 0; col < ncols; col++)
			{
				T a = data[npixel];
				T r = rdata[npixel];
				T g = gdata[npixel];
				T b = bdata[npixel];
				//printf("%d,", (int)a);
				npixel++;
				if (a == outsideColor.a() && r == outsideColor.r() && g == outsideColor.g() && b == outsideColor.b()) {
					x += resol;
					continue;//outside
				}
				double zenithD = sqrt(x*x + y*y) * 90.0;//in degrees
				if (zenithD <= 0.000000001)
					zenithD = 0.000000001;
				double zenithR = zenithD * 3.1415926 / 180.0;
				double wproj = sin(zenithR) / (zenithD / 90);//weight for equal-areal projection
				if (applyLambert)
				{
					wproj = wproj * cos(zenithR);
				}
				totalarea += wproj;
				if (a == greenColor.a() && r == greenColor.r() && g == greenColor.g() && b == greenColor.b())
				{
					skypixels++;
					skyarea += wproj;
				}
				else
				{
					nonskypixels++;
				}
				x += resol;
			}

		}
		double svf = skyarea / totalarea;
		delete[] data;
		delete[] rdata;
		delete[] gdata;
		delete[] bdata;
		return svf;
	}

	//double maxHorizon(osg::Vec3d eye, osg::Vec2 dir, double z);
};
//vec3 spherical2Cartisian(double lon, double lat,double )
//{
//	double theta = lon * 0.0174533;
//	double phi =   lat* 0.0174533;
//	return vec3(cos(phi)*cos(theta), cos(phi)*sin(theta), sin(phi));
//}
