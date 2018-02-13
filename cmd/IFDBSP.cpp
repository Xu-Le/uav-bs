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

#include "IFDBSP.h"

extern int log_level;

extern double gX;
extern double gY;
extern int numUser;
extern Point *groundUsers;
extern double *rateTable;
extern int *servedTable;

Solution::Solution() : totalServed(0), oc1(0.9), oc2(0.6), oc3(1.2)
{
	minRadius = UAV::radiusTable[UAV::minH];
	maxRadius = UAV::radiusTable[UAV::maxH];
	int averageDensity = numUser / static_cast<int>(gX/1000*gY/1000);
	if (averageDensity >= 400)
		beginH = UAV::minH;
	else if (averageDensity <= 100)
		beginH = UAV::maxH;
	else
		beginH = UAV::minH + UAV::maxH - averageDensity;

	int leftMargin = static_cast<int>(maxRadius/M_SQRT2) / DETECT * DETECT, downMargin = leftMargin;
	int rightMargin = static_cast<int>(gX - maxRadius/M_SQRT2) / DETECT * DETECT + DETECT;
	int upMargin = static_cast<int>(gY - maxRadius/M_SQRT2) / DETECT * DETECT + DETECT;
	countX = (rightMargin - leftMargin) / DETECT + 1;
	countY = (upMargin - downMargin) / DETECT + 1;
	margin = leftMargin;
	deployAllowed.resize(countX, std::vector<int>(countY, 1));
	info_log("margin: %f, countX: %d, countY: %d\n", margin, countX, countY);
	UAVs.reserve(32);

	xBucketNum = static_cast<int>(ceil(gX/DETECT));
	yBucketNum = static_cast<int>(ceil(gY/DETECT));
	std::vector<std::list<int> > yBuckets(yBucketNum);
	buckets.resize(xBucketNum, yBuckets);
	for (int i = 0; i < numUser; ++i)
	{
		int _xIdx = static_cast<int>(groundUsers[i].x) / DETECT;
		int _yIdx = static_cast<int>(groundUsers[i].y) / DETECT;
		buckets[_xIdx][_yIdx].push_back(i);
	}

	initialBucketNum = static_cast<int>(ceil((maxRadius + margin)/DETECT));
	outsideBucketNum = static_cast<int>(ceil((maxRadius - margin)/DETECT));
	info_log("initialBucketNum: %d, outsideBucketNum: %d\n", initialBucketNum, outsideBucketNum);
}

void Solution::deploy(int K, const char *statfile)
{
	Timer timer("\nSolution::deploy(): ");

	numAvailableUAV = K;
	batch = statfile != NULL ? 1 : 0;
	std::vector<int> numTotalServiced(K);
	for (int k = 0; k < K; ++k)
	{
		deployOne();
		numTotalServiced[k] = totalServed;
	}
	if (statfile != NULL)
	{
		double elapse = timer.elapsed();
		FILE *fd = fopen(statfile, "a");
		if (fd == NULL)
		{
			error_log("Fail to open file %s.\n", statfile);
			exit(EXIT_FAILURE);
		}
		for (size_t _k = 0; _k < numTotalServiced.size(); ++_k)
			fprintf(fd, "%d,", numTotalServiced[_k]);
		fprintf(fd, "%f\n", elapse);
		fclose(fd);
	}
}

void Solution::deployOne()
{
	uncond_log("\n=========================    deploy UAV %lu    =========================\n", UAVs.size()+1);
	Timer timer("Solution::deployOne(): ");

	UAVs.emplace_back(Band::_2_1);
	UAV &curUAV = UAVs.back();

	double deployProgress = static_cast<double>(UAVs.size()) / numAvailableUAV;
	int initialH = deployProgress*(UAV::maxH - beginH) + beginH;
	info_log("initialH: %d\n", initialH);
	curUAV.setH(initialH);

	double optimalX = 0.0, optimalY = 0.0;
	__findMaxCanServePlace(optimalX, optimalY, curUAV.getR());
	curUAV.setX(optimalX), curUAV.setY(optimalY);

	__feedUnserved(UAVs.size()-1);

	curUAV.adjust();
	totalServed += curUAV.userNum();
	info_log("UAV[%lu]: (%.2f,%.2f), R: %f, N: %d, total served num: %d\n\n", UAVs.size(), curUAV.getX(), curUAV.getY(), curUAV.getR(), curUAV.userNum(), totalServed);

	for (size_t idx = 0; idx < UAVs.size()-1; ++idx)
	{
		double distO2O = math::dist(curUAV.getPos(), UAVs[idx].getPos()), distRpR = curUAV.getR() + UAVs[idx].getR();
		if (distO2O < distRpR)
		{
			__adjustUAV(idx);
			info_log("UAV num: %lu, total served num: %d\n\n", UAVs.size(), totalServed);
			break;
		}
	}

	std::set<int> bandSet;
	for (int i = 0; i < countX; ++i)
	{
		for (int j = 0; j < countY; ++j)
		{
			Point coord(margin + i*DETECT, margin + j*DETECT);
			int numNearbyUAV = 0;
			bandSet.clear();
			for (size_t idx = 0; idx < UAVs.size(); ++idx)
			{
				double dist2Circle = math::dist(coord, UAVs[idx].getPos()) - UAVs[idx].getR();
				if (dist2Circle < 0)
					bandSet.insert(UAVs[idx].getBand());
				if (dist2Circle < maxRadius/UAV::maxH * beginH)
					++numNearbyUAV;
			}
			deployAllowed[i][j] = numNearbyUAV < 2 && bandSet.size() < UAV::bandNum ? 1 : 0;
		}
	}
}

int Solution::__findMaxCanServePlace(double &maxX, double &maxY, double R)
{
	int maxUnservedNearby = -1;
	int xBegin = 0, xEnd = initialBucketNum;
	for (int i = 0; i < countX; ++i)
	{
		int yBegin = 0, yEnd = initialBucketNum;
		for (int j = 0; j < countY; ++j)
		{
			if (deployAllowed[i][j] == 1)
			{
				Point coord(margin + i*DETECT, margin + j*DETECT);
				int unservedNearby = 0;
				for (int m = xBegin; m < xEnd; ++m)
					for (int n = yBegin; n < yEnd; ++n)
						for (itUS = buckets[m][n].begin(); itUS != buckets[m][n].end(); ++itUS)
							if (math::dist(groundUsers[*itUS], coord) < R)
								++unservedNearby;
				if (maxUnservedNearby < unservedNearby)
				{
					maxUnservedNearby = unservedNearby;
					maxX = coord.x, maxY = coord.y;
				}
			}
			if (j >= outsideBucketNum)
				++yBegin;
			if (j < yBucketNum - initialBucketNum)
				++yEnd;
		}
		if (i >= outsideBucketNum)
			++xBegin;
		if (i < xBucketNum - initialBucketNum)
			++xEnd;
	}
	// info_log("maxUnservedNearby: %d, position: (%.2f, %.2f)\n", maxUnservedNearby, maxX, maxY);
	return maxUnservedNearby;
}

void Solution::__adjustUAV(size_t uavIdx)
{
	UAV &curUAV = UAVs.back();

	int littleOverlapNum = 0;
	double maxOverlap = 100.0;
	std::vector<std::pair<double, size_t> > conflictedUAVs;
	std::set<int> conflictedBand;
	for (; uavIdx < UAVs.size()-1; ++uavIdx)
	{
		double distO2O = math::dist(curUAV.getPos(), UAVs[uavIdx].getPos()), distRpR = curUAV.getR() + UAVs[uavIdx].getR();
		if (distO2O < distRpR)
		{
			conflictedUAVs.push_back(std::pair<double, size_t>(distO2O/distRpR, uavIdx));
			conflictedBand.insert(UAVs[uavIdx].getBand());
			if (distO2O/distRpR >= 0.9)
				++littleOverlapNum;
			if (maxOverlap > distO2O/distRpR)
				maxOverlap = distO2O/distRpR;
		}
	}
	std::sort(conflictedUAVs.begin(), conflictedUAVs.end(), std::greater<std::pair<double, size_t> >());
	size_t conflictedBandNum = conflictedBand.size();
	info_log("conflicted UAV num: %lu, conflicted band num: %lu\n", conflictedUAVs.size(), conflictedBandNum);

	Point curPos(curUAV.getPos());
	double curR = curUAV.getR();
	int curServedNum = curUAV.userNum();

	if (maxOverlap >= oc1 || (conflictedBandNum == UAV::bandNum && UAV::bandNum > 1))
	{
		info_log("overlap is less than 20%%, maxOverlap: %f\n", maxOverlap);

		if (conflictedBandNum == UAV::bandNum)
		{
			if (UAV::bandNum > 1)
			{
				int bandToAvoid = Band::_2_1;
				conflictedBand.clear();
				int i = 0, j = 0, l = static_cast<int>(conflictedUAVs.size()) - 1;
				for (; l >= 0; --l)
				{
					bandToAvoid = UAVs[conflictedUAVs[l].second].getBand();
					conflictedBand.insert(bandToAvoid);
					if (conflictedBand.size() == conflictedBandNum)
						break;
				}
				while (j <= l)
				{
					if (UAVs[conflictedUAVs[j].second].getBand() == bandToAvoid)
						conflictedUAVs[i++] = conflictedUAVs[j++];
					else
						++j;
				}
				conflictedUAVs.erase(conflictedUAVs.begin()+i, conflictedUAVs.end());
				curUAV.setBand(bandToAvoid);
				info_log("conflictedBandNum == UAV::bandNum && UAV::bandNum > 1, choose band %d to avoid.\n", bandToAvoid);
			}
		}
		else if (littleOverlapNum >= 2)
			conflictedUAVs.erase(conflictedUAVs.begin()+littleOverlapNum, conflictedUAVs.end());

		std::vector<size_t> conflictedUAVs1;
		for (size_t k = 0; k < conflictedUAVs.size(); ++k)
			conflictedUAVs1.push_back(conflictedUAVs[k].second);

		UAV &oldUAV = UAVs[conflictedUAVs1[0]];
		double overlap = math::dist(oldUAV.getPos(), curUAV.getPos()) / (oldUAV.getR() + curUAV.getR());

		// initialize newPos under the assumption conflictedUAVs.size() == 1, if not, modify newPos
		Point newPos((curUAV.getX()-oldUAV.getX())/overlap + oldUAV.getX(), (curUAV.getY()-oldUAV.getY())/overlap + oldUAV.getY());
		bool resetNewPos = __tangentUAV(UAVs.size()-1, newPos, conflictedUAVs1);
		if (!resetNewPos)
		{
			newPos.x += curUAV.getX() >= oldUAV.getX() ? 1.0 : -1.0; // precision consideration
			newPos.y += curUAV.getY() >= oldUAV.getY() ? 1.0 : -1.0; // precision consideration
		}
		curUAV.setPos(newPos);

		__unfeedServed(UAVs.size()-1);
		__feedUnserved(UAVs.size()-1);

		totalServed += curUAV.userNum() - curServedNum;
		info_log("move [cur]: (%.2f,%.2f), R: %.2f, N: %d  ==>  (%.2f,%.2f), R: %.2f, N: %d\n", curPos.x, curPos.y, curR, curServedNum, curUAV.getX(), curUAV.getY(), curUAV.getR(), curUAV.userNum());
	}
	else if (maxOverlap >= oc2 || conflictedBandNum == UAV::bandNum)
	{
		info_log("overlap is less than 80%%, maxOverlap: %f\n", maxOverlap);

		if (conflictedBandNum == UAV::bandNum && UAV::bandNum == 1)
			info_log("conflictedBandNum == UAV::bandNum && UAV::bandNum == 1.\n");
		for (size_t k = 0; k < conflictedUAVs.size(); ++k)
			__alterUAV(conflictedUAVs[k].second);
	}

	bool assignmentOK = __assignBand();
	if (!assignmentOK)
	{
		error_log("assert assignmentOK == true failed.\n");
		exit(EXIT_FAILURE);
	}
}

void Solution::__alterUAV(size_t uavIdx)
{
	UAV &curUAV = UAVs.back();
	UAV &oldUAV = UAVs[uavIdx];
	double overlap = math::dist(oldUAV.getPos(), curUAV.getPos()) / (oldUAV.getR() + curUAV.getR());
	info_log("overlap: %f\n", overlap);
	if (overlap >= 1.0 || (overlap < oc2 && UAV::bandNum > 1))
		return;

	bool adjustOld = oldUAV.userNum() < oldUAV.getMaxU();
	double newR = (oldUAV.getR() + curUAV.getR()) / 2, deltaAngle = M_PI / 18.0;
	Point midOO((oldUAV.getX()+curUAV.getX())/2, (oldUAV.getY()+curUAV.getY())/2);
	for (size_t idx = 0; idx < UAVs.size()-1; ++idx)
	{
		if (idx == uavIdx)
			continue;
		double distO2O = math::dist(oldUAV.getPos(), UAVs[idx].getPos()), distRpR = oldUAV.getR() + UAVs[idx].getR();
		if (distO2O/distRpR < oc3 && oldUAV.getBand() == UAVs[idx].getBand())
			adjustOld = false;
		distO2O = math::dist(midOO, UAVs[idx].getPos());
		if (newR > (distO2O - UAVs[idx].getR())/2 && oldUAV.getBand() == UAVs[idx].getBand())
			newR = (distO2O - UAVs[idx].getR())/2;
	}

	Point oldPos(oldUAV.getPos()), curPos(curUAV.getPos());
	double oldR = oldUAV.getR(), curR = curUAV.getR();
	int oldServedNum = oldUAV.userNum(), curServedNum = curUAV.userNum();
	if (adjustOld) // adjust both oldUAV and curUAV
	{
		info_log("adjust both oldUAV and curUAV.\n");

		oldUAV.setX(-gX), oldUAV.setY(-gY);
		curUAV.setX(-gX), curUAV.setY(-gY);
		__unfeedServed(uavIdx);
		__unfeedServed(UAVs.size()-1);

		size_t maxUnservedNum = 0;
		double optimalAngle = 0.0;
		for (double angle = 0.0; angle < 3.1415926; angle += deltaAngle)
		{
			std::list<int> unservedList;
			__attainUnservedList(midOO.x + newR*cos(angle), midOO.y + newR*sin(angle), newR, unservedList);
			size_t unservedNum = unservedList.size();
			__attainUnservedList(midOO.x - newR*cos(angle), midOO.y - newR*sin(angle), newR, unservedList);
			unservedNum += unservedList.size();
			if (maxUnservedNum < unservedNum)
			{
				maxUnservedNum = unservedNum;
				optimalAngle = angle;
			}
		}
		info_log("newR: %f, maxUnservedNum: %lu, optimalAngle: %f\n", newR, maxUnservedNum, optimalAngle);

		double xi = -1.0;
		oldUAV.setX(midOO.x + newR*cos(optimalAngle)), oldUAV.setY(midOO.y + newR*sin(optimalAngle));
		curUAV.setX(midOO.x - newR*cos(optimalAngle)), curUAV.setY(midOO.y - newR*sin(optimalAngle));
		if (math::dist(oldPos, oldUAV.getPos()) > math::dist(oldPos, curUAV.getPos()))
		{
			oldUAV.setX(midOO.x - newR*cos(optimalAngle)), oldUAV.setY(midOO.y - newR*sin(optimalAngle));
			curUAV.setX(midOO.x + newR*cos(optimalAngle)), curUAV.setY(midOO.y + newR*sin(optimalAngle));
			xi = 1.0;
		}
		oldUAV.setR(newR, true), curUAV.setR(newR, true);

		__feedUnserved(uavIdx);
		__feedUnserved(UAVs.size()-1);

		if (oldUAV.userNum() == oldUAV.getMaxU())
			oldUAV.adjust();

		totalServed += oldUAV.userNum() - oldServedNum + curUAV.userNum() - curServedNum;
		info_log("move [old]: (%.2f,%.2f), R: %.2f, N: %d  ==>  (%.2f,%.2f), R: %.2f, N: %d", oldPos.x, oldPos.y, oldR, oldServedNum, oldUAV.getX(), oldUAV.getY(), oldUAV.getR(), oldUAV.userNum());
		info_log("move [cur]: (%.2f,%.2f), R: %.2f, N: %d  ==>  (%.2f,%.2f), R: %.2f, N: %d\n", curPos.x, curPos.y, curR, curServedNum, curUAV.getX(), curUAV.getY(), curUAV.getR(), curUAV.userNum());

		if (oldUAV.userNum() < oldUAV.getMaxU())
			__expandUAV(uavIdx, UAVs.size()-1, midOO, optimalAngle, -xi);
		if (curUAV.userNum() < curUAV.getMaxU())
			__expandUAV(UAVs.size()-1, uavIdx, midOO, optimalAngle, xi);
	}
	else // only adjust curUAV
	{
		info_log("only adjust curUAV.\n");

		curUAV.setX(-gX), curUAV.setY(-gY);
		__unfeedServed(UAVs.size()-1);

		int maxUnservedNum = 0;
		double optimalAngle = -2.0;
		Point newPos;
		oldR = oldUAV.getR(), curR = curUAV.getR(), newR = oldR + curR + 1.0; // precision consideration
		for (double angle = 0.0; angle < 6.2831852; angle += deltaAngle)
		{
			newPos.x = oldUAV.getX() + newR*cos(angle), newPos.y = oldUAV.getY() + newR*sin(angle);
			bool skip = false;
			for (size_t idx = 0; idx < UAVs.size()-1; ++idx)
			{
				if (idx != uavIdx && curUAV.getBand() == UAVs[idx].getBand() && math::dist(newPos, UAVs[idx].getPos()) < curR + UAVs[idx].getR())
				{
					skip = true;
					break;
				}
			}
			if (skip)
				continue;
			std::list<int> unservedList;
			__attainUnservedList(newPos.x, newPos.y, curR, unservedList);
			int unservedNum = static_cast<int>(unservedList.size());
			if (maxUnservedNum < unservedNum)
			{
				maxUnservedNum = unservedNum;
				optimalAngle = angle;
			}
		}
		if (optimalAngle > -1.0 && maxUnservedNum >= curServedNum/3)
		{
			info_log("maxUnservedNum: %d, optimalAngle: %f\n", maxUnservedNum, optimalAngle);
			newPos.x = oldUAV.getX() + newR*cos(optimalAngle);
			newPos.y = oldUAV.getY() + newR*sin(optimalAngle);
		}
		else
		{
			newPos = curPos;
			if (optimalAngle < -1.0)
				info_log("no valid angle can be found, undo rotation\n");
			else
				info_log("maxUnservedNum: %d is too small, undo rotation\n", maxUnservedNum);
		}
		__tangentUAV(UAVs.size()-1, newPos);

		curUAV.setPos(newPos);
		__feedUnserved(UAVs.size()-1);

		totalServed += curUAV.userNum() - curServedNum;
		info_log("move [cur]: (%.2f,%.2f), R: %.2f, N: %d  ==>  (%.2f,%.2f), R: %.2f, N: %d\n", curPos.x, curPos.y, curR, curServedNum, curUAV.getX(), curUAV.getY(), curUAV.getR(), curUAV.userNum());
	}
}

void Solution::__expandUAV(size_t uavIdx, size_t ignoreIdx, const Point& Q, double vartheta, double xi)
{
	UAV &uav = UAVs[uavIdx];

	Point _pos = uav.getPos();
	double _R = uav.getR(), newR = _R;
	int _servedNum = uav.userNum();

	size_t nearestIdx = 1000;
	double deltaR = 5.0, maxAllowedRadius = maxRadius;
	for (size_t idx = 0; idx < UAVs.size()-1; ++idx)
	{
		if (idx == uavIdx || idx == ignoreIdx)
			continue;
		double allowedRadius = math::dist(_pos, UAVs[idx].getPos()) - UAVs[idx].getR();
		if (maxAllowedRadius > allowedRadius && uav.getBand() == UAVs[idx].getBand())
		{
			maxAllowedRadius = allowedRadius;
			nearestIdx = idx;
		}
	}

	if ((newR = _R + deltaR) < maxAllowedRadius)
	{
		uav.setX(-gX), uav.setY(-gY);
		__unfeedServed(uavIdx);
	}
	while (newR < maxAllowedRadius)
	{
		uav.setX(Q.x + xi*newR*cos(vartheta)), uav.setY(Q.y + xi*newR*sin(vartheta));
		newR += deltaR;
		if (nearestIdx != 1000 && newR >= math::dist(uav.getPos(), UAVs[nearestIdx].getPos()) - UAVs[nearestIdx].getR())
			break;
		std::list<int> unservedList;
		__attainUnservedList(uav.getX(), uav.getY(), newR - deltaR, unservedList);
		if (static_cast<int>(unservedList.size()) >= uav.getMaxU())
			break;
	}
	uav.setR(newR - deltaR, true);

	__feedUnserved(uavIdx);

	totalServed += uav.userNum() - _servedNum;
	info_log("expand: (%.2f,%.2f), R: %.2f, N: %d  ==>  (%.2f,%.2f), R: %.2f, N: %d\n", _pos.x, _pos.y, _R, _servedNum, uav.getX(), uav.getY(), uav.getR(), uav.userNum());
}

bool Solution::__tangentUAV(size_t uavIdx, Point& newPos, std::vector<size_t> conflictedUAVs)
{
	UAV &uav = UAVs[uavIdx];

	bool tangentExecuted = false;
	if (conflictedUAVs.size() < 2)
	{
		std::vector<std::pair<double, size_t> > sortConflictedUAVs;
		for (size_t idx = 0; idx < UAVs.size()-1; ++idx)
		{
			if (idx == uavIdx || uav.getBand() != UAVs[idx].getBand())
				continue;
			double overlap = math::dist(newPos, UAVs[idx].getPos()) / (uav.getR() + UAVs[idx].getR());
			if (overlap < oc3)
				sortConflictedUAVs.push_back(std::pair<double, size_t>(overlap, idx));
		}
		if (sortConflictedUAVs.size() >= 3)
			std::sort(sortConflictedUAVs.begin(), sortConflictedUAVs.end());
		conflictedUAVs.clear();
		for (size_t k = 0; k < sortConflictedUAVs.size(); ++k)
			conflictedUAVs.push_back(sortConflictedUAVs[k].second);
	}
	else if (conflictedUAVs.size() >= 3)
		std::reverse(conflictedUAVs.begin(), conflictedUAVs.end());
	if (conflictedUAVs.size() > 3)
		conflictedUAVs.erase(conflictedUAVs.begin()+3, conflictedUAVs.end());
	if (conflictedUAVs.size() == 3)
	{
		info_log("three tangent circles.\n");

		size_t cIdx0 = conflictedUAVs[0], cIdx1 = conflictedUAVs[1], cIdx2 = conflictedUAVs[2];
		Point A = UAVs[cIdx0].getPos(), B = UAVs[cIdx1].getPos(), C = UAVs[cIdx2].getPos();
		double rA = UAVs[cIdx0].getR(), rB = UAVs[cIdx1].getR(), rC = UAVs[cIdx2].getR();
		double kAB = math::equal0(B.x - A.x) ? 1e10 : (B.y - A.y) / (B.x - A.x);
		double kAC = math::equal0(C.x - A.x) ? 1e10 : (C.y - A.y) / (C.x - A.x);
		Point curPos = newPos;
		if (!math::equal0(kAB - kAC) && math::dist(A, B) >= rA + rB && math::dist(B, C) >= rB + rC && math::dist(C, A) >= rC + rA)
		{
			newPos = math::circleCircleCircle(A, B, C, rA, rB, rC);
			double newR = math::dist(newPos, A) - rA;
			if (newR >= minRadius && newR <= maxRadius && math::dist(curPos, newPos) < maxRadius)
			{
				double maxReduceR = -1.0;
				for (size_t idx = 0; idx < UAVs.size()-1; ++idx)
				{
					if (idx == cIdx0 || idx == cIdx1 || idx == cIdx2)
						continue;
					double distO2O = math::dist(newPos, UAVs[idx].getPos()), distRpR = newR + UAVs[idx].getR();
					if (maxReduceR < distRpR - distO2O && uav.getBand() == UAVs[idx].getBand())
						maxReduceR = distRpR - distO2O;
				}
				info_log("maxReduceR: %f\n", maxReduceR);
				if (maxReduceR > 0)
					newR -= maxReduceR;
				uav.setR(newR, true);
				info_log("newPos: (%f,%f), newR: %f\n", newPos.x, newPos.y, newR);
				tangentExecuted = true;
			}
			else
			{
				if (newR < minRadius)
				{
					info_log("new radius is too small.\n");
					return tangentExecuted;
				}
				else if (newR > maxRadius)
					info_log("new radius is too large.\n");
				else
					info_log("moving distance is too large.\n");
			}
		}
		else if (math::equal0(kAB - kAC))
			info_log("point A, B, C on one line.\n");
		else
			info_log("there exists intersection between circles.\n");
		if (!tangentExecuted)
		{
			conflictedUAVs.pop_back();
			newPos = curPos;
		}
	}
	if (!tangentExecuted && conflictedUAVs.size() == 2)
	{
		info_log("two tangent circles.\n");

		size_t cIdx0 = conflictedUAVs[0], cIdx1 = conflictedUAVs[1];
		Point A = UAVs[cIdx0].getPos(), B = UAVs[cIdx1].getPos();
		double AB = math::dist(A, B), rA = UAVs[cIdx0].getR(), rB = UAVs[cIdx1].getR();
		if (uav.getR() < (AB - rA - rB)/2.0)
			uav.setR((AB - rA - rB)/2.0);
		newPos = math::circleCircle(A, B, newPos, rA, rB, uav.getR());

		size_t nearestIdx = 1000;
		double maxReduceR = -1.0;
		for (size_t idx = 0; idx < UAVs.size()-1; ++idx)
		{
			if (idx == cIdx0 || idx == cIdx1)
				continue;
			double distO2O = math::dist(newPos, UAVs[idx].getPos()), distRpR = uav.getR() + UAVs[idx].getR();
			if (maxReduceR < distRpR - distO2O && uav.getBand() == UAVs[idx].getBand())
			{
				maxReduceR = distRpR - distO2O;
				nearestIdx = idx;
			}
		}
		if (maxReduceR > 0)
		{
			info_log("three tangent circles [again].");
			Point C = UAVs[nearestIdx].getPos();
			double rC = UAVs[nearestIdx].getR();
			newPos = math::circleCircleCircle(A, B, C, rA, rB, rC);
			double newR = math::dist(newPos, A) - rA;
			uav.setR(newR, true);
		}

		tangentExecuted = true;
	}
	return tangentExecuted;
}

bool Solution::__assignBand()
{
	UAV &curUAV = UAVs.back();

	std::map<int, bool> bandUsed;
	bandUsed.insert(std::pair<int, bool>(Band::_2_1, false));
	if (UAV::bandNum > 1)
		bandUsed.insert(std::pair<int, bool>(Band::_2_2, false));
	if (UAV::bandNum > 2)
		bandUsed.insert(std::pair<int, bool>(Band::_2_3, false));
	for (size_t idx = 0; idx < UAVs.size()-1; ++idx)
		if (math::dist(curUAV.getPos(), UAVs[idx].getPos()) < curUAV.getR() + UAVs[idx].getR() - 1.0)
			bandUsed[UAVs[idx].getBand()] = true;

	bool conflicted = false;
	if (UAV::bandNum > 1 && bandUsed[Band::_2_1])
	{
		if (UAV::bandNum > 2 && bandUsed[Band::_2_2])
		{
			curUAV.setBand(Band::_2_3);
			conflicted = bandUsed[Band::_2_3];
		}
		else
		{
			curUAV.setBand(Band::_2_2);
			conflicted = bandUsed[Band::_2_2];
		}
	}
	else
	{
		curUAV.setBand(Band::_2_1);
		conflicted = bandUsed[Band::_2_1];
	}
	return !conflicted;
}

void Solution::__unfeedServed(size_t uavIdx)
{
	assert(uavIdx < UAVs.size());

	UAV &uav = UAVs[uavIdx];

	std::vector<int> erased;
	uav.check(erased);
	for (size_t k = 0; k < erased.size(); ++k)
		__handleBuckets(erased[k], false);
}

void Solution::__feedUnserved(size_t uavIdx)
{
	assert(uavIdx < UAVs.size());

	UAV &uav = UAVs[uavIdx];
	Point coord(uav.getPos());

	std::list<int> unservedList;
	__attainUnservedList(uav.getX(), uav.getY(), uav.getR(), unservedList);
	if (!unservedList.empty())
		unservedList.sort([&coord](const int& lhs, const int& rhs) {
			return math::dist(groundUsers[lhs], coord) < math::dist(groundUsers[rhs], coord);
		});
	while (!unservedList.empty() && uav.userNum() < uav.getMaxU())
	{
		int curUsU = unservedList.front(); // alias
		unservedList.pop_front();
		uav.serve(curUsU); // must be UAV::Status::OK
		__handleBuckets(curUsU, true);
	}
}

void Solution::__handleBuckets(int user, bool serve)
{
	assert(user < numUser);

	int _xIdx = static_cast<int>(groundUsers[user].x) / DETECT;
	int _yIdx = static_cast<int>(groundUsers[user].y) / DETECT;
	if (serve)
	{
		itUS = std::find(buckets[_xIdx][_yIdx].begin(), buckets[_xIdx][_yIdx].end(), user);
		assert(itUS != buckets[_xIdx][_yIdx].end());
		buckets[_xIdx][_yIdx].erase(itUS);
	}
	else
		buckets[_xIdx][_yIdx].push_back(user);
}

void Solution::__attainUnservedList(double uavX, double uavY, double uavR, std::list<int>& unservedList)
{
	Point coord(uavX, uavY);
	int xIdx = static_cast<int>((coord.x - margin)/DETECT), yIdx = static_cast<int>((coord.y - margin)/DETECT);
	int xBegin = std::max(xIdx - outsideBucketNum, 0), xEnd = std::min(xIdx + initialBucketNum, xBucketNum);
	int yBegin = std::max(yIdx - outsideBucketNum, 0), yEnd = std::min(yIdx + initialBucketNum, yBucketNum);
	unservedList.clear();
	for (int m = xBegin; m < xEnd; ++m)
		for (int n = yBegin; n < yEnd; ++n)
			for (itUS = buckets[m][n].begin(); itUS != buckets[m][n].end(); ++itUS)
				if (servedTable[*itUS] == 0 && math::dist(groundUsers[*itUS], coord) < uavR)
					unservedList.push_back(*itUS);
}

void Solution::result(const char *uavFile, const char *userFile)
{
	FILE *fd = fopen(uavFile, "w");
	if (fd == NULL)
	{
		error_log("Fail to open %s.\n", uavFile);
		exit(EXIT_FAILURE);
	}

	for (size_t i = 0; i < UAVs.size(); ++i)
		fprintf(fd, "%.2f,%.2f,%.2f,%.2f,%.2f,%f,%d,%d\n", UAVs[i].getX(), UAVs[i].getY(), UAVs[i].getH(),
				UAVs[i].getR(), UAVs[i].getP(), UAVs[i].getB(), UAVs[i].getBand(), UAVs[i].userNum());

	fclose(fd);

	fd = fopen(userFile, "w");
	if (fd == NULL)
	{
		error_log("Fail to open %s.\n", userFile);
		exit(EXIT_FAILURE);
	}

	for (int i = 0; i < numUser; ++i)
		if (servedTable[i] == 1)
			fprintf(fd, "%.2f,%.2f\n", groundUsers[i].x, groundUsers[i].y);

	fclose(fd);
}
