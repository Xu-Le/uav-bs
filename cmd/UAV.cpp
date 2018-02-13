//
// Copyright (C) 2017-2018 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "UAV.h"

extern int log_level;

extern Point *groundUsers;
extern double *rateTable;
extern int *servedTable;

double UAV::theta = 0.7407;    // 42.44 degree
double UAV::sensitivity = -94.0; // -94dBm, see Table 7.3.3-1 of 3GPP TS 36.521-1 V14.4.0 (2017-09)
double UAV::TNPSD = -174.0;    // thermal noise power spectrum density: -174dBm/Hz
double UAV::reqPercent = 0.25; // 25%
int UAV::minH = 100;
int UAV::maxH = 400;
int UAV::maxU[2] = { 0, 0 };
size_t UAV::bandNum = 2;
double UAV::totalB[2] = { 9000.0, 18000.0 };
double UAV::totalC[2] = { 0.0, 0.0 };
std::vector<int> UAV::vrbTable[2] = { std::vector<int>(18), std::vector<int>(26) }; // an extra space for handling overruns in UAV::serve()
std::vector<double> UAV::radiusTable;
std::vector<double> UAV::powerTable;
std::vector<std::vector<double> > UAV::snrTable;

UAV::UAV(int _band) : band(-1), type(-1), B(0.0)
{
	setBand(_band);
}

UAV::~UAV()
{
	for (itU = users.begin(); itU != users.end(); ++itU)
		servedTable[itU->first] = 0;
	users.clear();
}

bool UAV::served(int user)
{
	return users.find(user) != users.end();
}

int UAV::serve(int user)
{
	if (servedTable[user] == 1)
		return Status::ALREADY;
	if (math::dist(pos, groundUsers[user]) > R)
		return Status::OUT;
	double bwReq = 180.0 * vrbTable[type][0]; // 180kHz
	users.insert(std::pair<int, double>(user, bwReq));
	B += bwReq;
	servedTable[user] = 1;
	// debug_log("serve user [%d], now B is %f\n", user, B);
	return Status::OK;
}

int UAV::unserve(int user)
{
	if ((itU = users.find(user)) != users.end())
	{
		B -= itU->second;
		servedTable[user] = 0;
		// debug_log("unserve user [%d], now B is %f\n", user, B);
		users.erase(itU);
		return Status::OK;
	}
	return Status::NOTEXIST;
}

void UAV::adjust()
{
	std::vector<Point*> usersPos(users.size(), NULL);
	itU = users.begin();
	for (size_t k = 0; itU != users.end(); ++itU)
		usersPos[k++] = &groundUsers[itU->first];
	disc.initialize(usersPos.begin(), usersPos.end());
	disc.cover();
	setR(disc.r);
	pos.x = disc.O.x;
	pos.y = disc.O.y;
}

void UAV::check(std::vector<int>& erased)
{
	erased.clear();
	for (itU = users.begin(); itU != users.end();)
	{
		if (math::dist(pos, groundUsers[itU->first]) > R)
		{
			B -= itU->second;
			servedTable[itU->first] = 0;
			erased.push_back(itU->first);
			itU = users.erase(itU);
		}
		else
			++itU;
	}
}

void UAV::setH(int h)
{
	if (h < UAV::minH)
		h = UAV::minH;
	if (h > UAV::maxH)
		h = UAV::maxH;
	pos.z = h;
	R = radiusTable[h];
	P = powerTable[h];
}

void UAV::setR(double r, bool smaller)
{
	int newH = std::lower_bound(radiusTable.begin() + minH, radiusTable.end(), r) - radiusTable.begin();
	if (smaller && newH > minH)
		--newH;
	pos.z = newH;
	R = radiusTable[newH];
	P = powerTable[newH];
}

void UAV::setBand(int _band)
{
	int oldType = type;
	switch (_band)
	{
	case Band::_2_0:
	case Band::_2_1:
	case Band::_2_2:
	case Band::_2_3:
		type = 1;
		break;
	case Band::_4_1:
	case Band::_4_2:
	case Band::_4_3:
	case Band::_4_4:
		type = 0;
		break;
	default:
		error_log("Unknown operating band!\n");
		exit(EXIT_FAILURE);
	}
	if (band != -1 && type != oldType)
	{
		error_log("cannot change to different band type.\n");
		exit(EXIT_FAILURE);
	}
	band = _band;
}

ReadConfig::ReadConfig(const char *filename)
{
	fd = fopen(filename, "r");
	if (fd == NULL)
	{
		error_log("Fail to open file %s.\n", filename);
		exit(EXIT_FAILURE);
	}

	fgets(buf, 100, fd);
}

ReadConfig::~ReadConfig()
{
	fclose(fd);
}

void ReadConfig::parse(std::string& key, std::string& value)
{
	std::string line(buf);
	line.pop_back(); // drop '\n'
	for (size_t i = 0; i < line.size(); ++i)
	{
		if (line[i] == '=')
		{
			assert(i > 0 && i < line.size()-1);
			key = line.substr(0, i);
			value = line.substr(i+1);
			break;
		}
	}

	fgets(buf, 100, fd);
}

bool ReadConfig::hasNext()
{
	return buf[0] != '\n';
}

void configureUAV()
{
	{
		ReadConfig readConfig("UAV.conf");
		std::string key, value;
		while (readConfig.hasNext())
		{
			readConfig.parse(key, value);
			if (key == "minH")
				UAV::minH = atoi(value.c_str());
			else if (key == "maxH")
				UAV::maxH = atoi(value.c_str());
			else if (key == "bandNum")
				UAV::bandNum = atoi(value.c_str());
			else if (key == "reqPercent")
				UAV::reqPercent = atof(value.c_str());
		}
	}

	for (int k = 0; k < 16; ++k)
		UAV::vrbTable[0][k] = 3 * (k + 1);
	UAV::vrbTable[0][16] = 50;
	for (int k = 0; k < 25; ++k)
		UAV::vrbTable[1][k] = 4 * (k + 1);
	UAV::vrbTable[1][25] = UAV::vrbTable[0][17] = 2147483647; // handle overruns in UAV::serve()
	UAV::radiusTable.resize(UAV::maxH + 1);
	UAV::powerTable.resize(UAV::maxH + 1);
	UAV::snrTable.resize(UAV::maxH + 1);

	double cotTheta = 1.0 / tan(UAV::theta);
	double cscTheta = 1.0 / sin(UAV::theta);
	// PLmax = A / (1 + a*exp(-b*(theta-a))) + 20*log10(h/sin(theta)) + B
	double A = -19.0; // (yita_LoS, yita_NLoS): (1.0, 20.0)
	double f = 2e9, c = 3e8;
	double B = 20*log10(4*M_PI*f/c) + 20.0;
	double a = 0.0, b = 0.0; // urban environment
	// calculate parameters 'a' and 'b'
	{
		double alpha = 0.3, beta = 500.0, gamma = 15.0; // urban environment
		double Ca[4][4] = { {9.34e-1, 1.97e-2, -1.24e-4, 2.73e-7}, {2.3e-1, 2.44e-3, -3.34e-6, 0}, {-2.25e-3, 6.58e-6, 0, 0}, {1.86e-5, 0, 0, 0} };
		double Cb[4][4] = { {1.17, -5.79e-3, 1.73e-5, -2e-8}, {-7.56e-2, 1.81e-4, -2.02e-7, 0}, {1.98e-3, -1.65e-6, 0, 0}, {-1.78e-5, 0, 0, 0} };
		for (int j = 0; j <= 3; ++j)
		{
			for (int i = 0; i <= 3-j; ++i)
			{
				double powRes = pow(alpha*beta, i) * pow(gamma, j);
				a += powRes * Ca[j][i];
				b += powRes * Cb[j][i];
			}
		}
	}
	double expRes = exp(-b * (UAV::theta - a));
	double C = A / (1 + a*expRes);
	for (int h = UAV::minH; h <= UAV::maxH; ++h)
	{
		UAV::radiusTable[h] = h * cotTheta;
		UAV::powerTable[h] = C + 20*log10(h*cscTheta) + B + UAV::sensitivity;
	}
	if (log_level >= DEBUG_LEVEL)
		for (int h = UAV::minH; h <= UAV::maxH; ++h)
			printf("h: %d, R: %f, P: %fdBm <=> %fmW\n", h, UAV::radiusTable[h], UAV::powerTable[h], math::dBm2mW(UAV::powerTable[h]));
#if 0
	FILE *fd = fopen("PLmax.csv", "w");
	if (fd == NULL)
	{
		error_log("Fail to open PLmax.csv.\n");
		exit(EXIT_FAILURE);
	}

	for (int h = UAV::minH; h <= UAV::maxH; ++h)
		fprintf(fd, "%d,%f,%f\n", h, UAV::radiusTable[h], UAV::powerTable[h] - UAV::sensitivity);

	fclose(fd);
#endif
	for (int h = UAV::minH; h <= UAV::maxH; ++h)
	{
		int maxR = static_cast<int>(UAV::radiusTable[h]);
		UAV::snrTable[h].resize(maxR + 1);
		int theta = M_PI_2; // 90 degree, special case r == 0
		expRes = exp(-b * (theta - a));
		C = A / (1 + a*expRes);
		UAV::snrTable[h][0] = UAV::powerTable[h] - (C + 10*log10(h*h) + B);
		// printf("h: %d, r: 0, S: %fdB\n", h, UAV::snrTable[h][0]);
		for (int r = 1; r <= maxR; ++r)
		{
			theta = atan(h/r);
			expRes = exp(-b * (theta - a));
			C = A / (1 + a*expRes);
			UAV::snrTable[h][r] = UAV::powerTable[h] - (C + 10*log10(h*h + r*r) + B);
			// printf("h: %d, r: %d, S: %fdB\n", h, r, UAV::snrTable[h][r]);
		}
	}

	UAV::maxU[0] = 16.0 / UAV::reqPercent;
	UAV::maxU[1] = 25.0 / UAV::reqPercent;
	int averageD = static_cast<int>(UAV::radiusTable[UAV::maxH] / M_SQRT2);
	for (int type = 0; type < 2; ++type)
	{
		double thermalNoise = UAV::TNPSD + 10*log10(UAV::totalB[type]/25) + 30.0;
		double averageSNR = math::dBm2mW(UAV::snrTable[UAV::maxH][averageD] - thermalNoise);
		UAV::totalC[type] = UAV::totalB[type] * log(1 + averageSNR);
		info_log("total bandwidth %fkHz  <==>  total capacity: %fkbps\n", UAV::totalB[type], UAV::totalC[type]);
	}
}
