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

#ifndef __IFDBSP_H__
#define __IFDBSP_H__

#include "UAV.h"

class Solution
{
public:
	enum Granularity {
		DETECT = 50
	};

	Solution();

	void initialize(int _num);
	void result(const char *uavFile, const char *userFile);

	void deployOne(std::vector<size_t>& adjustIndices);

private:
	Solution(const Solution&);
	Solution& operator=(const Solution&);

	int __findMaxCanServePlace(double& maxX, double& maxY, double R);
	void __adjustUAV(size_t uavIdx, std::vector<size_t>& adjustIndices);
	void __alterUAV(size_t uavIdx);
	void __expandUAV(size_t uavIdx, size_t ignoreIdx, const Point& Q, double vartheta, double xi);
	bool __tangentUAV(size_t uavIdx, Point& newPos, std::vector<size_t> conficitedUAVs = std::vector<size_t>());

	bool __assignBand();
	void __unfeedServed(size_t uavIdx);
	void __feedUnserved(size_t uavIdx);
	void __handleBuckets(int user, bool serve);
	void __attainUnservedList(double uavX, double uavY, double uavR, std::list<int>& unservedList);

public:
	int numAvailableUAV;
	int totalServed;
	int beginH;
	int countX;
	int countY;
	int xBucketNum;
	int yBucketNum;
	int initialBucketNum;
	int outsideBucketNum;
	double minRadius;
	double maxRadius;
	double margin;
	const double oc1;
	const double oc2;
	const double oc3;
	std::list<int>::iterator itUS;
	std::vector<UAV> UAVs;
	std::vector<std::vector<int> > deployAllowed;
	std::vector<std::vector<std::list<int> > > buckets;
};

#endif /* __IFDBSP_H__ */
