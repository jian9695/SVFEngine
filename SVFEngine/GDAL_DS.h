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
				m_dataset = nullptr;
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

public:
	GDALDataset* m_dataset;
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
