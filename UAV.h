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

#ifndef __UAV_H__
#define __UAV_H__

#include "Utils.h"

enum Band {
	_2_0 = 1910, ///< dummy band
	_2_1 = 1930, ///< 1930 - 1950
	_2_2 = 1950, ///< 1950 - 1970
	_2_3 = 1970, ///< 1970 - 1990
	_4_1 = 2110, ///< 2110 - 2120
	_4_2 = 2120, ///< 2120 - 2130
	_4_3 = 2130, ///< 2130 - 2140
	_4_4 = 2140  ///< 2140 - 2150
};

class UAV
{
public:
	explicit UAV(int _band);
	~UAV();

	enum Status {
		OK,
		OUT,
		FULL,
		ALREADY,
		NOTEXIST
	};

	bool served(int user);
	int serve(int user);
	int unserve(int user);

	void adjust();
	void check(std::vector<int>& erased);

	double getX() { return pos.x; }
	double getY() { return pos.y; }
	double getH() { return pos.z; }
	double getR() { return R; }
	double getP() { return P; }
	double getB() { return B; }
	double remainingB() { return totalB[type] - B; }
	double getTotalB() { return totalB[type]; }
	double getTotalC() { return totalC[type]; }
	int getMaxU() { return UAV::maxU[type]; }
	int getBand() { return band; }
	int getType() { return type; }
	int userNum() { return static_cast<int>(users.size()); }
	const Point& getPos() { return pos; }
	void getUserList(std::list<int>& userList);
	void setX(double x) { pos.x = x; }
	void setY(double y) { pos.y = y; }
	void setH(int h);
	void setR(double r, bool smaller = false);
	void setBand(int _band);
	void setPos(const Point& _pos) { pos.x = _pos.x; pos.y = _pos.y; }

private:
	int band;  ///< which band a UAV operates on.
	int type;  ///< band type of a UAV.
	Point pos; ///< 3D position.
	double R;  ///< coverage radius.
	double P;  ///< transmit power.
	double B;  ///< occupied bandwidth.
	std::map<int, double> users;  ///< users served by this UAV, map from user ID to its occupied bandwidth.
	std::map<int, double>::iterator itU; ///< iterator used to handle with container users.
	Disc disc; ///< min cover disc of users.

public:
	static double theta;  ///< optimal elevation angle depends only on the environment.
	static double sensitivity; ///< sensitivity of the receiver, measured in dBm.
	static double TNPSD;  ///< thermal noise power spectrum density, measured in dBm/Hz.
	static double reqPercent; ///< the max percent of user needed to be served simultaneously.
	static int minH; ///< min allowed height of UAV.
	static int maxH; ///< max allowed height of UAV.
	static int maxU[2]; ///< the max number of user can be served by a UAV.
	static size_t bandNum; ///< the total number of different frequency band.
	static double totalB[2]; ///< total available bandwidth of a UAV, measured in kHz.
	static double totalC[2]; ///< total estimated capacity of a UAV, measured in kbps.
	static std::vector<int> vrbTable[2];    ///< number of allocated VRB according to RBG size p, see Table 7.1.6.1-1 of 3GPP TS 36.213 V14.4.0 (2017-09).
	static std::vector<double> radiusTable; ///< map from UAV height to its coverage radius, measured in meter.
	static std::vector<double> powerTable;  ///< map from UAV height to its transmit power, measured in dBm.
	static std::vector<std::vector<double> > snrTable; ///< 1st dimension is UAV height, 2nd dimension is the distance between ground user and 2D projection of the UAV, map to the SNR of ground users, measured in dB.
};

class ReadConfig
{
public:
	ReadConfig(const char *filename);
	~ReadConfig();

	void parse(std::string& key, std::string& value);
	bool hasNext();

private:
	FILE *fd;
	char buf[100];
};

/** configure static members of class UAV. */
void configureUAV();

#endif /* __UAV_H__ */
