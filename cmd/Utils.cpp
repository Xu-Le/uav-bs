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

#include "Utils.h"

extern int log_level;

extern int numUser;
extern double gX;
extern double gY;
extern Point *groundUsers;
extern double *rateTable;
extern int *servedTable;

namespace math {

bool equal0(double n)
{
	return n > -1e-8 && n < 1e-8;
}

double mW2dBm(double mW)
{
	return 10.0 * log10(mW);
}

double dBm2mW(double dBm)
{
	return pow(10.0, dBm/10.0);
}

double dist(const Point& A, const Point& B)
{
	return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
}

double dist2(const Point& A, const Point& B)
{
	return (A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y);
}

double dist3D(const Point& A, const Point& B)
{
	return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y) + (A.z - B.z)*(A.z - B.z));
}

double dotProduct(const Point& A, const Point& B)
{
	return A.x * B.x + B.x * B.y;
}

double crossProduct(const Point& A, const Point& B)
{
	return A.x * B.y - B.x * A.y;
}

/**
 * L1: a*x + b*y = c, passing through (A.x, A.y),(B.x, B.y), its midpoint (xm, ym), its perpendicular bisector a1*x + b1*y = c1;
 * a1 = -b = B.x - A.x, b1 = a = B.y - A.y, c1 = a*ym - b*xm = ((B.x*B.x - A.x*A.x) + (B.y*B.y - A.y*A.y)) / 2;
 * L2's perpendicular bisector a2*x + b2*y = c2;
 * a2 = C.x - A.x, b2 = C.y - A.y, c2 = ((C.x*C.x - A.x*A.x) + (C.y*C.y - A.y*A.y)) / 2;
 * Let delta = a1*b2 - a2*b1:
 * if delta == 0  ==>  L1 // L2;
 * else  ==>  O.x = (b2*c1 - b1*c2) / delta, O.y = (a1*c2 - a2*c1) / delta.
 */
double circumcircle(const Point& A, const Point& B, const Point& C, Point& O)
{
	double a1 = B.x - A.x, b1 = B.y - A.y, c1 = ((B.x*B.x - A.x*A.x) + (B.y*B.y - A.y*A.y)) / 2;
	double a2 = C.x - A.x, b2 = C.y - A.y, c2 = ((C.x*C.x - A.x*A.x) + (C.y*C.y - A.y*A.y)) / 2;
	double delta = a1*b2 - a2*b1;
	O.x = (b2*c1 - b1*c2) / delta;
	O.y = (a1*c2 - a2*c1) / delta;
	double R2 = dist2(O, A);
	return R2;
}

Point circleCircle(const Point& A, const Point& B, const Point &C, double rA, double rB, double rC)
{
	double a = rB + rC, b = rC + rA, c = math::dist(A, B);
	double cosA = (b*b + c*c - a*a) / (2*b*c), sinA = sqrt(1 - cosA*cosA);
	double AD = b * cosA, CD = b * sinA;
	info_log("cosA: %f, sinA: %f, AD: %f, CD: %f\n", cosA, sinA, AD, CD);
	Point D((B.x-A.x)*AD/c + A.x, (B.y-A.y)*AD/c + A.y);
	double k = equal0(B.y - A.y) ? 1e10 : (A.x - B.x) / (B.y - A.y), de = sqrt(1 + k*k);
	Point C1(D.x + CD/de, D.y + CD*k/de), C2(D.x - CD/de, D.y - CD*k/de);
	info_log("A: (%f,%f), B: (%f,%f), C: (%f,%f), D: (%f,%f), C1:(%f,%f), C2: (%f,%f)\n", A.x, A.y, B.x, B.y, C.x, C.y, D.x, D.y, C1.x, C1.y, C2.x, C2.y);
	return dist(C, C1) < dist(C, C2) ? C1 : C2;
}

Point pointPointCircle(const Point& A, const Point& B, const Point& I, double rI)
{
	Point D((A.x + B.x)/2, (A.y + B.y)/2), P, O;
	info_log("A: (%f,%f), B: (%f,%f), D: (%f,%f)\n", A.x, A.y, B.x, B.y, D.x, D.y);
	if (equal0(A.y - B.y))
	{
		if (equal0(D.x - I.x))
		{
			double AD = dist(A, B)/2, DI = D.y > I.y ? D.y - I.y : I.y - D.y;
			DI -= rI;
			double R = (AD*AD + DI*DI)/2/DI, DO = DI - R;
			O.x = D.x, O.y = D.y > I.y ? D.y - DO : D.y + DO;
			return O;
		}
		// find intersect point P of midnormal of line AB and midnormal of line AI
		double k2 = (A.x - I.x) / (I.y - A.y), b2 = A.y - k2*A.x;
		P.x = D.x;
		P.y = k2*P.x + b2;
	}
	else
	{
		double k = (A.x - B.x) / (B.y - A.y), b = D.y - k*D.x;
		if (equal0((D.y - I.y)/(D.x - I.x) - k))
		{
			double AD = dist(A, B)/2, DI = dist(D, I), de = sqrt(1 + k*k);
			DI -= rI;
			double R = (AD*AD + DI*DI)/2/DI, DO = DI - R;
			O.x = D.x + DO/de, O.y = D.y + DO*k/de;
			return O;
		}
		// find intersect point P of midnormal of line AB and midnormal of line AI
		Point midAI((A.x + I.x)/2, (A.y + I.y)/2);
		if (equal0(I.y - A.y))
		{
			P.x = midAI.x;
			P.y = k*P.x + b;
		}
		else
		{
			double k2 = (A.x - I.x) / (I.y - A.y), b2 = midAI.y - k2*midAI.x;
			P.x = (b2 - b) / (k - k2);
			P.y = k2*P.x + b2;
			bool checkP = equal0((k*P.x + b) - (k2*P.x + b2));
			assert(checkP);
		}
	}
	double rP = dist(P, A);

	// obtain intersect point C of line AB and verticle line of circle I and circle P
	double _k = (I.x - P.x) / (P.y - I.y);
	double _b = (P.x*P.x - I.x*I.x + P.y*P.y - I.y*I.y + rI*rI - rP*rP)/2/(P.y - I.y);
	Point C(A.x, 0.0); // assume A.x == B.x
	if (equal0(B.x - A.x))
		C.y = _k*C.x + _b;
	else
	{
		double k = (B.y - A.y) / (B.x - A.x), b = A.y - k*A.x;
		C.x = (_b - b) / (k - _k), C.y = k*C.x + b;
	}
	info_log("I: (%f,%f), P:(%f,%f), rI: %f, rP: %f, C: (%f,%f)\n", I.x, I.y, P.x, P.y, rI, rP, C.x, C.y);

	// find tangent point
	double y0 = I.y - C.y; // alias
	double a = (2*C.x*I.x - C.x*C.x - I.x*I.x + rI*rI), b = 2*y0*(I.x - C.x), c = rI*rI - y0*y0;
	double delta = b*b - 4*a*c;
	assert(delta >= 0.0);
	double k1 = (-b + sqrt(delta))/2/a, k2 = (-b - sqrt(delta))/2/a;
	a = k1*k1 + 1, b = -2*(C.x*k1*k1 + y0*k1 + I.x), c = C.x*C.x*k1*k1 + 2*C.x*y0*k1 + I.x*I.x + y0*y0 - rI*rI;
	info_log("b*b - 4*a*c: %f\n", b*b - 4*a*c);
	Point T1(-b/2/a, k1*(-b/2/a)+C.y-k1*C.x);
	a = k2*k2 + 1, b = -2*(C.x*k2*k2 + y0*k2 + I.x), c = C.x*C.x*k2*k2 + 2*C.x*y0*k2 + I.x*I.x + y0*y0 - rI*rI;
	info_log("b*b - 4*a*c: %f\n", b*b - 4*a*c);
	Point T2(-b/2/a, k2*(-b/2/a)+C.y-k2*C.x);
	info_log("k1: %f, k2: %f, T1: (%f,%f), T2:(%f,%f)\n", k1, k2, T1.x, T1.y, T2.x, T2.y);

	// plot circle O1 by points A, B, T1; plot circle O2 by points A, B, T2
	Point O1, O2;
	double R1 = sqrt(circumcircle(A, B, T1, O1)), R2 = sqrt(circumcircle(A, B, T2, O2));
	info_log("O1: (%f,%f), O2:(%f,%f), R1: %f, R2: %f\n", O1.x, O1.y, O2.x, O2.y, R1, R2);
	bool checkO1 = dist(O1, T1) < dist(O1, I), checkO2 = dist(O2, T2) < dist(O2, I);
	assert(checkO1 || checkO2);
	return checkO1 ? O1 : O2;
}

Point _pointCircleCircle(const Point& A, const Point& B, const Point& C, double rA, double rB)
{
	bool radiusEqual = equal0(rA - rB);
	assert(radiusEqual);
	Point D((A.x + B.x)/2, (A.y + B.y)/2);
	double k = equal0(A.y - B.y) ? 1e10 : (A.x - B.x) / (B.y - A.y), b = D.y - k*D.x;
	double b1 = b - A.y, b2 = b - C.y;
	double t1 = (k*(b1 - b2) + C.x - A.x)/rA, t2 = (b1*b1 - b2*b2 + A.x*A.x - C.x*C.x - rA*rA)/(2*rA);
	double t3 = 2*(k*b2 - C.x), t4 = C.x*C.x + b2*b2;
	double _a = t1*t1 - k*k - 1, _b = 2*t1*t2 - t3, _c = t2*t2 - t4, delta = _b*_b - 4*_a*_c;
	assert(delta >= 0);
	double x1 = (-_b + sqrt(delta))/2/_a, x2 = (-_b - sqrt(delta))/2/_a;
	Point O1(x1, k*x1 + b), O2(x2, k*x2 + b);
	bool checkO1 = equal0(dist(A, O1) - dist(C, O1) - rA), checkO2 = equal0(dist(A, O2) - dist(C, O2) - rA);
	assert(checkO1 || checkO2);
	return checkO1 ? O1 : O2;
}

Point pointCircleCircle(const Point& A, const Point& B, const Point& C, double rA, double rB)
{
	bool radiusNotEqual = !equal0(rA - rB);
	assert(radiusNotEqual);
	double AB = dist(A, B), AS1 = rA * AB / (rA - rB), AS2 = rA * AB / (rA + rB);
	Point S1((B.x-A.x)*AS1/AB + A.x, (B.y-A.y)*AS1/AB + A.y), S2((B.x-A.x)*AS2/AB + A.x, (B.y-A.y)*AS2/AB + A.y);
	Point P((B.x-A.x)*rA/AB + A.x, (B.y-A.y)*rA/AB + A.y), Q((B.x-A.x)*(AB-rB)/AB + A.x, (B.y-A.y)*(AB-rB)/AB + A.y);
	info_log("S1: (%f,%f), S2:(%f,%f), P: (%f,%f), Q:(%f,%f)\n", S1.x, S1.y, S2.x, S2.y, P.x, P.y, Q.x, Q.y);
	// obtain circle O by points C, P, Q
	Point O;
	double R = sqrt(circumcircle(C, P, Q, O));
	// obtain intersect point B1 of line CS1 and circle O
	Point B1(C.x, O.y + O.y - C.y); // assume C.x == S1.x
	if (!equal0(C.x - S1.x))
	{
		double k_ = (S1.y - C.y) / (S1.x - C.x), b_ = C.y - k_*C.x - O.y; // b_ = C.y - _k*C.x; b_ -= O.y;
		double _a = k_*k_ + 1, _b = 2*(k_*b_ - O.x), _c = O.x*O.x +b_*b_ - R*R;
		double delta = _b*_b - 4*_a*_c;
		assert(delta >= 0.0);
		double _x1 = (-_b + sqrt(delta))/2/_a, _x2 = (-_b - sqrt(delta))/2/_a;
		B1.x = fabs(C.x - _x1) > fabs(C.x - _x2) ? _x1 : _x2;
		B1.y = k_*B1.x + b_ + O.y; // remember we have execute b_ -= O.y;
	}
	info_log("O: (%f,%f), R: %f, B1: (%f,%f)\n", O.x, O.y, R, B1.x, B1.y);
	return pointPointCircle(B1, C, A, rA);
}

Point _circleCircleCircle(const Point& A, const Point& B, const Point& C, double rA, double rB, double rC)
{
	assert(rA >= rB && rB >= rC);
	if (equal0(rA - rB))
	{
		if (equal0(rB - rC))
		{
			Point O;
			circumcircle(A, B, C, O);
			return O;
		}
		else
			return _pointCircleCircle(A, B, C, rA - rC, rB - rC);
	}
	else // (rA > rB)
	{
		if (equal0(rB - rC))
			return pointPointCircle(B, C, A, rA - rC);
		else
			return pointCircleCircle(A, B, C, rA - rC, rB - rC);
	}
}

Point circleCircleCircle(const Point& A, const Point& B, const Point& C, double rA, double rB, double rC)
{
	if (rA >= rB)
	{
		if (rB >= rC)
			return _circleCircleCircle(A, B, C, rA, rB, rC);
		else if (rC >= rA)
			return _circleCircleCircle(C, A, B, rC, rA, rB);
		else
			return _circleCircleCircle(A, C, B, rA, rC, rB);
	}
	else
	{
		if (rA >= rC)
			return _circleCircleCircle(B, A, C, rB, rA, rC);
		else if (rC >= rB)
			return _circleCircleCircle(C, B, A, rC, rB, rA);
		else
			return _circleCircleCircle(B, C, A, rB, rC, rA);
	}
}

}

bool Disc::add(Point *user)
{
	Point U = *user;
	if (math::dist2(O, U) > r2)
	{
		points.push_back(user);
		_minDiscWithPoint(U, static_cast<int>(points.size())-2);
		r = sqrt(r2);
		return true;
	}
	return false;
}

bool Disc::remove(Point *user)
{
	if (math::equal0(math::dist2(O, *user) - r2))
	{
		points.erase(std::remove(points.begin(), points.end(), user), points.end());
		cover();
		return true;
	}
	return false;
}

void Disc::cover()
{
	if (points.size() < 3)
	{
		error_log("points number less than 3!\n");
		exit(EXIT_FAILURE);
	}

	std::random_shuffle(points.begin(), points.end());

	__segmentMidpoint(*points[0], *points[1]);

	int n = static_cast<int>(points.size());
	for (int i = 2; i < n; ++i)
	{
		Point U = *points[i];
		if (math::dist2(O, U) > r2)
			_minDiscWithPoint(U, i-1);
	}
	r = sqrt(r2);
}

void Disc::_minDiscWithPoint(const Point& P, int n)
{
	__segmentMidpoint(P, *points[0]);

	for (int j = 1; j <= n; ++j)
	{
		Point U = *points[j];
		if (math::dist2(O, U) > r2)
			_minDiscWith2Points(P, U, j-1);
	}
}

void Disc::_minDiscWith2Points(const Point& P, const Point& Q, int n)
{
	__segmentMidpoint(P, Q);

	for (int k = 0; k <= n; ++k)
	{
		Point U = *points[k];
		if (math::dist2(O, U) > r2)
		{
			if (math::equal0(math::crossProduct(Point(Q.x - P.x, Q.y - P.y), Point(U.x - P.x, U.y - P.y))))
			{
				double d1 = math::dist2(P, Q), d2 = math::dist2(Q, U), d3 = math::dist2(U, P);
				if (d1 >= d2 && d1 >= d3)
					__segmentMidpoint(P, Q);
				else if (d2 >= d1 && d2 >= d3)
					__segmentMidpoint(Q, U);
				else
					__segmentMidpoint(U, P);
			}
			else
				r2 = math::circumcircle(P, Q, U, O);
		}
	}
}

void Disc::__segmentMidpoint(const Point& P, const Point& Q)
{
	O.x = (P.x + Q.x) / 2.0;
	O.y = (P.y + Q.y) / 2.0;
	r2 = math::dist2(O, P);
}

void ConvexHull::grahamScan()
{
	if (points.size() < 3)
	{
		error_log("points number less than 3!\n");
		exit(EXIT_FAILURE);
	}

	_sortPoints();

	S.clear();
	S.reserve(points.size());
	S.push_back(points[0]);
	S.push_back(points[1]);
	S.push_back(points[2]);
	size_t j = S.size();
	for (size_t i = 3; i < points.size(); ++i)
	{
		while (math::crossProduct(Point(points[i]->x - S[j-1]->x, points[i]->y - S[j-1]->y), Point(S[j-1]->x - S[j-2]->x, S[j-1]->y - S[j-2]->y)) > 0)
		{
			S.pop_back();
			--j;
		}
		S.push_back(points[i]);
		++j;
	}
}

void ConvexHull::_sortPoints()
{
	Point p0(1e8, 1e8);
	size_t i = 0, p0Idx = 0;
	for (i = 0; i < points.size(); ++i)
		if (p0.y > points[i]->y || (math::equal0(p0.y - points[i]->y) && p0.x > points[i]->x))
			p0 = *points[i], p0Idx = i;

	if (p0Idx > 0)
		std::swap(points[0], points[p0Idx]);

	std::sort(points.begin()+1, points.end(), [p0](const Point *lhs, const Point *rhs) {
		return math::crossProduct(Point(lhs->x - p0.x, lhs->y - p0.y), Point(rhs->x - p0.x, rhs->y - p0.y)) > 0;
	});

	if (log_level >= DEBUG_LEVEL)
		for (i = 0; i < points.size(); ++i)
			printf("i: %lu, x: %f, y: %f\n", i, points[i]->x, points[i]->y);
}

int parseInput(const char *filename)
{
	FILE *fd = fopen(filename, "r");
	if (fd == NULL)
	{
		error_log("Fail to open file %s.\n", filename);
		exit(EXIT_FAILURE);
	}

	int _x = 0, _y = 0, availableUAVNum = 0;
	fscanf(fd, "%d,%d,%d,%d\n", &_x, &_y, &numUser, &availableUAVNum);
	gX = _x, gY = _y;

	groundUsers = new Point[numUser];
	rateTable = new double[numUser];
	servedTable = new int[numUser];
	for (int i = 0; i < numUser; ++i)
	{
		fscanf(fd, "%lf,%lf,%lf\n", &groundUsers[i].x, &groundUsers[i].y, &rateTable[i]);
		servedTable[i] = 0;
	}

	fclose(fd);

	return availableUAVNum;
}

bool vectorFind(std::vector<int>& vec, const int key)
{
	return std::find(vec.begin(), vec.end(), key) != vec.end();
}

void vectorRemove(std::vector<int>& vec, const int key)
{
	vec.erase(std::remove(vec.begin(), vec.end(), key), vec.end());
}


////////////////    convenient functions of std::vector and std::list    ////////////////
void printVector(std::vector<int>& _vector, const char *_note, int _log_level)
{
	if (log_level >= _log_level)
	{
		printf("%s | %s", _log_level >= DEBUG_LEVEL ? "DEBUG" : "INFO", _note);
		for (std::vector<int>::iterator iter = _vector.begin(); iter != _vector.end(); ++iter)
			printf("%d ", *iter);
		printf("\n");
	}
}

void printVector(std::vector<double>& _vector, const char *_note, int _log_level)
{
	if (log_level >= _log_level)
	{
		printf("%s | %s", _log_level >= DEBUG_LEVEL ? "DEBUG" : "INFO", _note);
		for (std::vector<double>::iterator iter = _vector.begin(); iter != _vector.end(); ++iter)
			printf("%f ", *iter);
		printf("\n");
	}
}

void printList(std::list<int>& _list, const char *_note, int _log_level)
{
	if (log_level >= _log_level)
	{
		printf("%s | %s", _log_level >= DEBUG_LEVEL ? "DEBUG" : "INFO", _note);
		for (std::list<int>::iterator iter = _list.begin(); iter != _list.end(); ++iter)
			printf("%d ", *iter);
		printf("\n");
	}
}

void printList(std::list<double>& _list, const char *_note, int _log_level)
{
	if (log_level >= _log_level)
	{
		printf("%s | %s", _log_level >= DEBUG_LEVEL ? "DEBUG" : "INFO", _note);
		for (std::list<double>::iterator iter = _list.begin(); iter != _list.end(); ++iter)
			printf("%f ", *iter);
		printf("\n");
	}
}
