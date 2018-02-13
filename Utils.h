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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <list>
#include <stack>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <utility>

#include <QDebug>

class Point
{
public:
	Point() : x(0.0), y(0.0), z(0.0) {}
	Point(double X, double Y) : x(X), y(Y), z(0.0) {}

	double x;
	double y;
	double z;
};

namespace math {

bool equal0(double n);

double mW2dBm(double mW);

double dBm2mW(double dBm);

double dist(const Point& A, const Point& B);

double dist2(const Point& A, const Point& B);

double dist3D(const Point& A, const Point& B);

double dotProduct(const Point& A, const Point& B);

double crossProduct(const Point& A, const Point& B);

double circumcircle(const Point& A, const Point& B, const Point& C, Point& O);

Point circleCircle(const Point& A, const Point& B, const Point& C, double rA, double rB, double rC);

Point pointPointCircle(const Point& A, const Point& B, const Point& I, double rI);

Point pointCircleCircle(const Point& A, const Point& B, const Point& C, double rA, double rB);

Point circleCircleCircle(const Point& A, const Point& B, const Point& C, double rA, double rB, double rC);

}

class Disc
{
public:
	template <class _InputIter>
	void initialize(_InputIter _first, _InputIter _last);

	bool add(Point *user);
	bool remove(Point *user);
	void cover();
	size_t size() { return points.size(); }

private:
	void _minDiscWithPoint(const Point& P, int n);
	void _minDiscWith2Points(const Point& P, const Point& Q, int n);
	void __segmentMidpoint(const Point& P, const Point& Q);

public:
	Point O;
	double r;

private:
	double r2; // r^2, for efficiency
	std::vector<Point*> points;
};

template <class _InputIter>
void Disc::initialize(_InputIter _first, _InputIter _last)
{
	points.assign(_first, _last);
}

class ConvexHull
{
public:
	template <class _InputIter>
	void initialize(_InputIter _first, _InputIter _last);

	void grahamScan();
	size_t size() { return points.size(); }

private:
	void _sortPoints();

private:
	std::vector<Point*> points;
	std::vector<Point*> S;
};

template <class _InputIter>
void ConvexHull::initialize(_InputIter _first, _InputIter _last)
{
	points.assign(_first, _last);
}

int parseInput(const char *filename);

bool vectorFind(std::vector<int>& vec, const int key);

void vectorRemove(std::vector<int>& vec, const int key);

#endif /* __UTILS_H__ */
