#include <iostream>
#include "shpline_io_shplib.h"
namespace utility
{


shpline_io_shplib::shpline_io_shplib() {

}

shpline_io_shplib::~shpline_io_shplib() {

}

bool shpline_io_shplib::readPolylineShp(const std::string& filename, std::vector<std::vector<Eigen::Vector3d>>& polylines) {
	bool in_flag = false;
    std::cout << "begin read shp: " << filename << std::endl;

	SHPHandle hSHP;
	DBFHandle hDBF;
	int nShapeType, nEntities;
	double adfMinBound[4], adfMaxBound[4];

	hSHP = SHPOpen(filename.c_str(), "r");
	if (hSHP == NULL)
	{
        std::cout << "Failed read: " << filename << std::endl;
		return in_flag;
	}

	char szDBF[MAX_PATH + 1];

	std::strcpy(szDBF, filename.c_str());
	szDBF[strlen(szDBF) - 3] = '\0';
	std::strcat(szDBF, "dbf");
	hDBF = DBFOpen(szDBF, "rb");
	if (!hDBF)
	{
        std::cout << "Failed read: " << filename << std::endl;
		SHPClose(hSHP);
		return in_flag;
	}

	SHPGetInfo(hSHP, &nEntities, &nShapeType, adfMinBound, adfMaxBound);

	SHPObject *psElem;
	double *padfX, *padfY, *padfZ;

	int nField;
	nField = DBFGetFieldCount(hDBF);
	polylines.resize(nEntities);

	for (int i = 0; i < nEntities; i++)
	{
		std::vector<Eigen::Vector3d>().swap(polylines[i]);

		psElem = SHPReadObject(hSHP, i);

		padfX = new double[psElem->nVertices];
		padfY = new double[psElem->nVertices];
		padfZ = new double[psElem->nVertices];

		//std::cout << "i: " << i << "vertex"  << "\n" << psElem->nVertices << std::endl;

		for (int j = 0; j < psElem->nVertices; j++)
		{
			padfX[j] = psElem->padfX[j];
			padfY[j] = psElem->padfY[j];
			padfZ[j] = psElem->padfZ[j];

			Eigen::Vector3d point{ padfX[j], padfY[j], padfZ[j] };
			polylines[i].push_back(point);
		}

		for (int j = 0; j < nField; j++)
		{
			DBFFieldType eType;
			int nWidth, nDecimals;
			char szTitle[20];

			eType = DBFGetFieldInfo(hDBF, j, szTitle, &nWidth, &nDecimals);
			switch (eType)
			{
			case FTString:
			{
				//std::string value = DBFReadStringAttribute(hDBF, i, j);
				break;
			}

			case FTInteger:
			{
				int value = DBFReadIntegerAttribute(hDBF, i, j);
				break;
			}
			case FTDouble:
			{
				double value = DBFReadDoubleAttribute(hDBF, i, j);
				break;
			}
			default:
				break;
			}
		}

		delete[] padfX;
		delete[] padfY;
		delete[] padfZ;
		SHPDestroyObject(psElem);
	}

	// �ر��ļ�
	SHPClose(hSHP);
	DBFClose(hDBF);
}

bool shpline_io_shplib::writePolylineShp(const std::string& filename,
                                const std::vector<std::vector<Eigen::Vector3d>>& plyls,
                                         const bool is_closed) {
	bool o_flag = false;

	// create filename.shp, filename.dbf
	SHPHandle hShp = SHPCreate(std::string(filename + ".shp").c_str(), SHPT_ARCZ);
	DBFHandle hDbf = DBFCreate(std::string(filename + ".dbf").c_str());

	int nlines = plyls.size();
	if (0 == nlines)
	{
		return o_flag;
	}
	SHPObject* shpObject;

	for (int i = 0; i < nlines; ++i)
	{
		// field index
		int field_idx = 0;

		int nVertices = plyls[i].size();
		int nVertices_plus = nVertices;
        if(is_closed){
            ++nVertices_plus;
        }
        double* padfX = new double[nVertices_plus];
        double* padfY = new double[nVertices_plus];
        double* padfZ = new double[nVertices_plus];
        double* padfm = new double[nVertices_plus];

        for (int j = 0; j < nVertices; ++j)
        {
            padfX[j] = plyls[i][j][0];
            padfY[j] = plyls[i][j][1];
            padfZ[j] = plyls[i][j][2];
            padfm[j] = i;
        }
        if(is_closed){
            padfX[nVertices] = plyls[i][0][0];
            padfY[nVertices] = plyls[i][0][1];
            padfZ[nVertices] = plyls[i][0][2];
            padfm[nVertices] = nVertices;
        }
		shpObject = SHPCreateObject(SHPT_ARCZ, -1, 0, NULL, NULL, nVertices_plus, padfX, padfY, padfZ, padfm);
		SHPWriteObject(hShp, -1, shpObject);
		SHPDestroyObject(shpObject);

		delete[] padfX;
		delete[] padfY;
		delete[] padfZ;
		delete[] padfm;
		padfX = NULL;
		padfY = NULL;
		padfZ = NULL;
		padfm = NULL;

		// create .dbf attributes
		DBFAddField(hDbf, "ID", FTInteger, 10, 0);
		DBFAddField(hDbf, "Name", FTString, 10, 0);
		DBFAddField(hDbf, "Length", FTDouble, 32, 0);

		// dbf record
		int record_idx = DBFGetRecordCount(hDbf);

		DBFWriteIntegerAttribute(hDbf, record_idx, field_idx++, 1001);
		DBFWriteStringAttribute(hDbf, record_idx, field_idx++, "polyline");
		DBFWriteDoubleAttribute(hDbf, record_idx, field_idx++, 10.15);
	}

	DBFClose(hDbf);
	SHPClose(hShp);
	o_flag = true;

	return o_flag;
}

bool shpline_io_shplib::writePolylineShpWithOffset(
        const std::string& filename,
        const std::vector<std::vector<Eigen::Vector3f>>& plyls,
        const Eigen::Vector3d offset,
        const bool is_closed) {
	bool o_flag = false;

	// create filename.shp, filename.dbf
	SHPHandle hShp = SHPCreate(std::string(filename + ".shp").c_str(), SHPT_ARCZ);
	DBFHandle hDbf = DBFCreate(std::string(filename + ".dbf").c_str());

	int nlines = plyls.size();
	if (0 == nlines)
	{
		return o_flag;
	}
	SHPObject* shpObject;

	for (int i = 0; i < nlines; ++i)
	{
		// field index
		int field_idx = 0;

		int nVertices = plyls[i].size();
        int nVertices_plus = nVertices;
        if(is_closed){
            ++nVertices_plus;
        }
        double* padfX = new double[nVertices_plus];
        double* padfY = new double[nVertices_plus];
        double* padfZ = new double[nVertices_plus];
        double* padfm = new double[nVertices_plus];

        for (int j = 0; j < nVertices; ++j)
        {
            padfX[j] = plyls[i][j][0];
            padfY[j] = plyls[i][j][1];
            padfZ[j] = plyls[i][j][2];
            padfm[j] = i;
        }
        if(is_closed){
            padfX[nVertices] = plyls[i][0][0];
            padfY[nVertices] = plyls[i][0][1];
            padfZ[nVertices] = plyls[i][0][2];
            padfm[nVertices] = nVertices;
        }

		shpObject = SHPCreateObject(SHPT_ARCZ, -1, 0, NULL, NULL, nVertices, padfX, padfY, padfZ, padfm);
		SHPWriteObject(hShp, -1, shpObject);
		SHPDestroyObject(shpObject);

		delete[] padfX;
		delete[] padfY;
		delete[] padfZ;
		delete[] padfm;
		padfX = NULL;
		padfY = NULL;
		padfZ = NULL;
		padfm = NULL;

		// create .dbf attributes
		DBFAddField(hDbf, "ID", FTInteger, 10, 0);
		DBFAddField(hDbf, "Name", FTString, 10, 0);
		DBFAddField(hDbf, "Length", FTDouble, 32, 0);

		// dbf record
		int record_idx = DBFGetRecordCount(hDbf);

		DBFWriteIntegerAttribute(hDbf, record_idx, field_idx++, 1001);
		DBFWriteStringAttribute(hDbf, record_idx, field_idx++, "polyline");
		DBFWriteDoubleAttribute(hDbf, record_idx, field_idx++, 10.15);
	}

	DBFClose(hDbf);
	SHPClose(hShp);
	o_flag = true;

	return o_flag;
}

} //namespace utility


