//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
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

#include <cstring>
#include <string>
#include <algorithm>

#include "Perlin.h"

int gX = 0;
int gY = 0;
static int numUser = 0;
static int numAvailableUAV = 0;

double dblrand(double lo, double hi) { return (hi - lo) * rand() / RAND_MAX + lo; }

class Point
{
public:
	Point() : x(0.0), y(0.0) {}
	Point(double X, double Y) : x(X), y(Y) {}

	double x;
	double y;
};

double dist2(const Point& A, const Point& B) { return (A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y); }

/////////////////////////    AbstractGenerator    /////////////////////////
class AbstractGenerator
{
public:
	virtual ~AbstractGenerator() {}

	void generate(Point *groundUsers, double *rateTable);
	virtual void generateGU(Point *groundUsers) = 0;
	virtual void generateRT(double *rateTable) = 0;
};

void AbstractGenerator::generate(Point *groundUsers, double *rateTable)
{
	generateGU(groundUsers);
	generateRT(rateTable);
}

/////////////////////////    UniformGenerator    /////////////////////////
class UniformGenerator: public AbstractGenerator
{
public:
	virtual void generateGU(Point *groundUsers);
	virtual void generateRT(double *rateTable);
};

void UniformGenerator::generateGU(Point *groundUsers)
{
	for (int i = 0; i < numUser; ++i)
	{
		groundUsers[i].x = dblrand(0, gX);
		groundUsers[i].y = dblrand(0, gY);
	}
}

void UniformGenerator::generateRT(double *rateTable)
{
	double rateArray[4] = { 500.0, 1000.0, 1500.0, 2000.0 };
	for (int i = 0; i < numUser; ++i)
		rateTable[i] = rateArray[rand() % 4];
}

/////////////////////////    PerlinGenerator    /////////////////////////
class PerlinGenerator: public AbstractGenerator
{
public:
	virtual void generateGU(Point *groundUsers);
	virtual void generateRT(double *rateTable);
};

void PerlinGenerator::generateGU(Point *groundUsers)
{
	int granularity = 50, borderSparse = static_cast<int>(gX/20.0), bgn = borderSparse / granularity; // border grid number
	int northEastRemove = static_cast<int>(gX/5.0), negn = northEastRemove / granularity; // north east grid number
	int i = 0, xGridNum = gX / granularity, yGridNum = gY / granularity;
	double *_populationMap = new double[xGridNum*yGridNum];
	double **populationMap = new double*[xGridNum];
	for (i = 0; i < xGridNum; ++i)
		populationMap[i] = _populationMap + i*yGridNum;
	PopulationMap::generate(populationMap, granularity);

	// normalize populationMap matrix, the sum of all elements is 1
	for (i = 1; i < xGridNum*yGridNum; ++i)
		_populationMap[i] += _populationMap[i-1];
	for (i = 0; i < xGridNum*yGridNum; ++i)
		_populationMap[i] /= _populationMap[xGridNum*yGridNum-1];

	for (i = 0; i < numUser;)
	{
		int index = std::lower_bound(_populationMap, _populationMap + xGridNum*yGridNum, dblrand(0, 1)) - _populationMap;
		int xIdx = index / yGridNum, yIdx = index % yGridNum;
		if (xIdx >= xGridNum-negn && yIdx >= yGridNum-negn)
			continue;
		if ((xIdx < bgn || xIdx >= xGridNum-bgn || yIdx < bgn || yIdx >= yGridNum-bgn) && dblrand(0.0, 1.0) > 0.2)
			continue;
		groundUsers[i].x = dblrand(xIdx*granularity, (xIdx+1)*granularity);
		groundUsers[i].y = dblrand(yIdx*granularity, (yIdx+1)*granularity);
		bool skip = false;
		for (int j = 0; j < i; ++j)
		{
			if (dist2(groundUsers[i], groundUsers[j]) < 1.0)
			{
				skip = true;
				break;
			}
		}
		if (!skip)
			++i;
	}

	delete []populationMap;
	delete []_populationMap;
}

void PerlinGenerator::generateRT(double *rateTable)
{
	double rateArray[4] = { 500.0, 1000.0, 1500.0, 2000.0 };
	for (int i = 0; i < numUser; ++i)
		rateTable[i] = rateArray[rand() % 4];
}


static void drawGroundUsers(const char *filename, Point *groundUsers, double *rateTable)
{
	FILE *fd = fopen(filename, "w");
	if (fd == NULL)
	{
		fprintf(stderr, "Fail to open file %s.\n", filename);
		exit(EXIT_FAILURE);
	}

	for (int i = 0; i < numUser; ++i)
		fprintf(fd, "%f,%f,%d\n", groundUsers[i].x, groundUsers[i].y, static_cast<int>(rateTable[i]/500.0));

	fclose(fd);
}

static void fileOutput(const char *filename, Point *groundUsers, double *rateTable)
{
	FILE *fd = fopen(filename, "w");
	if (fd == NULL)
	{
		fprintf(stderr, "Fail to open file %s.\n", filename);
		exit(EXIT_FAILURE);
	}

	fprintf(fd, "%d,%d,%d,%d\n", gX, gY, numUser, numAvailableUAV);

	for (int i = 0; i < numUser; ++i)
		fprintf(fd, "%f,%f\n", groundUsers[i].x, groundUsers[i].y);

	fclose(fd);
}

static void printHelp()
{
	printf("Usage:\n    ./genCases -x * -y * -n * -k * [-s *] case_x.txt\n");
	printf("Example:\n    ./genCases -x 4000 -y 4000 -n 1000 -k 10 [-s 1] case_1.txt\n");
}

int main(int argc, char *argv[])
{
	if (argc != 10 && argc != 12)
	{
		printHelp();
		exit(EXIT_FAILURE);
	}

	gX = atoi(argv[2]);
	gY = atoi(argv[4]);
	numUser = atoi(argv[6]);
	numAvailableUAV = atoi(argv[8]);
	unsigned int seed = argc == 12 ? atoi(argv[10]) : 1;
	srand(seed);
	if (gX <= 100 || gY <= 100)
	{
		fprintf(stderr, "The map is too small.\n");
		exit(EXIT_FAILURE);
	}
	if (numUser <= 0)
	{
		fprintf(stderr, "User number must be positive.\n");
		exit(EXIT_FAILURE);
	}
	if (numAvailableUAV <= 0)
	{
		fprintf(stderr, "Available UAV number must be positive.\n");
		exit(EXIT_FAILURE);
	}
	char *cfn = argv[argc-1]; // case file name
	if (strlen(cfn) < 6 || cfn[0] != 'c' || cfn[1] != 'a' || cfn[2] != 's' || cfn[3] != 'e' || cfn[4] != '_')
	{
		fprintf(stderr, "Case file name is invalid.\n");
		exit(EXIT_FAILURE);
	}

	Point *groundUsers = new Point[numUser];
	double *rateTable = new double[numUser];

	AbstractGenerator *generator = new PerlinGenerator;
	generator->generateGU(groundUsers);
	generator->generateRT(rateTable);
	delete generator;

	std::string gufn(cfn);
	size_t gufnL = gufn.size();
	gufn[0] = 'u'; gufn[1] = 's'; gufn[2] = 'e'; gufn[3] = 'r';
	gufn[gufnL-3] = 'c'; gufn[gufnL-2] = 's'; gufn[gufnL-1] = 'v';
	drawGroundUsers(gufn.c_str(), groundUsers, rateTable);
	fileOutput(argv[argc-1], groundUsers, rateTable);

	delete []groundUsers;
	delete []rateTable;

	return 0;
}
