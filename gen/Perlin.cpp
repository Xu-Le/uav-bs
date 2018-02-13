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

#include "Perlin.h"

extern int gX;
extern int gY;

uint8_t Perlin::perm[512] = { 0 };
uint8_t Perlin::p[256] = { 151,160,137,91,90,15,
131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
138,236,205,93,222,114, 67, 29, 24, 72,243,141,128,195,78,66,215,61,156,180 };

Perlin::Grad Perlin::gradP[512] = { Grad() };
Perlin::Grad Perlin::grad3[12] =  { Grad(1,1,0), Grad(-1,1,0), Grad(1,-1,0), Grad(-1,-1,0), 
									Grad(1,0,1), Grad(-1,0,1), Grad(1,0,-1), Grad(-1,0,-1),
									Grad(0,1,1), Grad(0,-1,1), Grad(0,1,-1), Grad(0,-1,-1) };

double Perlin::F2 = 0.5 * (sqrt(3.0) - 1.0);
double Perlin::G2 = (3.0 - sqrt(3.0)) / 6.0;
double Perlin::F3 = 1.0 / 3.0;
double Perlin::G3 = 1.0 / 6.0;

double PopulationMap::halfWidth = 0.0;
double PopulationMap::halfHeight = 0.0;
double PopulationMap::perturbation1 = 100.0;
double PopulationMap::perturbation2 = 400.0;
double PopulationMap::perturbation3 = 800.0;

// This isn't a very good seeding function, but it works ok. It supports 2^16
// different seed values. Write something better if you need more seeds.
void Perlin::seeding(uint16_t seed)
{
	if (seed < 256)
		seed |= seed << 8;
	for (int i = 0; i < 256; ++i)
	{
		int v;
		if (i & 0x0001)
			v = p[i] ^ (seed & 255);
		else
			v = p[i] ^ ((seed>>8) & 255);
		perm[i] = perm[i + 256] = v;
		gradP[i] = gradP[i + 256] = grad3[v % 12];
	}
}

double Perlin::simplex2(double xin, double yin)
{
	double n0, n1, n2; // Noise contributions from the three corners
	// Skew the input space to determine which simplex cell we're in
	double s = (xin + yin) * F2; // Hairy factor for 2D
	int i = static_cast<int>(xin + s);
	int j = static_cast<int>(yin + s);
	double t = (i + j) * G2;
	double x0 = xin - i + t; // The x,y distances from the cell origin, unskewed.
	double y0 = yin - j + t;
	// For the 2D case, the simplex shape is an equilateral triangle.
	// Determine which simplex we are in.
	int i1, j1; // Offsets for second (middle) corner of simplex in (i,j) coords
	if (x0 > y0) // lower triangle, XY order: (0,0)->(1,0)->(1,1)
	{
		i1 = 1;
		j1 = 0;
	}
	else // upper triangle, YX order: (0,0)->(0,1)->(1,1)
	{
		i1 = 0;
		j1 = 1;
	}
	// A step of (1,0) in (i,j) means a step of (1-c,-c) in (x,y), and
	// a step of (0,1) in (i,j) means a step of (-c,1-c) in (x,y), where
	// c = (3-sqrt(3))/6
	double x1 = x0 - i1 + G2; // Offsets for middle corner in (x,y) unskewed coords
	double y1 = y0 - j1 + G2;
	double x2 = x0 - 1 + 2 * G2; // Offsets for last corner in (x,y) unskewed coords
	double y2 = y0 - 1 + 2 * G2;
	// Work out the hashed gradient indices of the three simplex corners
	i &= 255;
	j &= 255;
	Grad gi0(gradP[i + perm[j]]);
	Grad gi1(gradP[i + i1 + perm[j+j1]]);
	Grad gi2(gradP[i + 1 + perm[j+1]]);
	// Calculate the contribution from the three corners
	double t0 = 0.5 - x0*x0 - y0*y0;
	if (t0 < 0)
		n0 = 0;
	else
	{
		t0 *= t0;
		n0 = t0 * t0 * gi0.dot2(x0, y0);  // (x,y) of grad3 used for 2D gradient
	}
	double t1 = 0.5 - x1*x1 - y1*y1;
	if (t1 < 0)
		n1 = 0;
	else
	{
		t1 *= t1;
		n1 = t1 * t1 * gi1.dot2(x1, y1);
	}
	double t2 = 0.5 - x2*x2 - y2*y2;
	if (t2 < 0)
		n2 = 0;
	else
	{
		t2 *= t2;
		n2 = t2 * t2 * gi2.dot2(x2, y2);
	}
	// Add contributions from each corner to get the final noise value.
	// The result is scaled to return values in the interval [-1,1].
	return 70 * (n0 + n1 + n2);
}

double Perlin::simplex3(double xin, double yin, double zin)
{
	double n0, n1, n2, n3; // Noise contributions from the four corners

	// Skew the input space to determine which simplex cell we're in
	double s = (xin + yin + zin) * F3; // Hairy factor for 2D
	int i = static_cast<int>(xin+s);
	int j = static_cast<int>(yin+s);
	int k = static_cast<int>(zin+s);

	double t = (i + j + k) * G3;
	double x0 = xin - i + t; // The x,y distances from the cell origin, unskewed.
	double y0 = yin - j + t;
	double z0 = zin - k + t;

	// For the 3D case, the simplex shape is a slightly irregular tetrahedron.
	// Determine which simplex we are in.
	int i1, j1, k1; // Offsets for second corner of simplex in (i,j,k) coords
	int i2, j2, k2; // Offsets for third corner of simplex in (i,j,k) coords
	if (x0 >= y0)
	{
		if (y0 >= z0)      { i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 1; k2 = 0; }
		else if (x0 >= z0) { i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 0; k2 = 1; }
		else               { i1 = 0; j1 = 0; k1 = 1; i2 = 1; j2 = 0; k2 = 1; }
	}
	else
	{
		if (y0 < z0)      { i1 = 0; j1 = 0; k1 = 1; i2 = 0; j2 = 1; k2 = 1; }
		else if (x0 < z0) { i1 = 0; j1 = 1; k1 = 0; i2 = 0; j2 = 1; k2 = 1; }
		else              { i1 = 0; j1 = 1; k1 = 0; i2 = 1; j2 = 1; k2 = 0; }
	}
	// A step of (1,0,0) in (i,j,k) means a step of (1-c,-c,-c) in (x,y,z),
	// a step of (0,1,0) in (i,j,k) means a step of (-c,1-c,-c) in (x,y,z), and
	// a step of (0,0,1) in (i,j,k) means a step of (-c,-c,1-c) in (x,y,z), where
	// c = 1/6.
	double x1 = x0 - i1 + G3; // Offsets for second corner
	double y1 = y0 - j1 + G3;
	double z1 = z0 - k1 + G3;

	double x2 = x0 - i2 + 2 * G3; // Offsets for third corner
	double y2 = y0 - j2 + 2 * G3;
	double z2 = z0 - k2 + 2 * G3;

	double x3 = x0 - 1 + 3 * G3; // Offsets for fourth corner
	double y3 = y0 - 1 + 3 * G3;
	double z3 = z0 - 1 + 3 * G3;

	// Work out the hashed gradient indices of the four simplex corners
	i &= 255;
	j &= 255;
	k &= 255;
	Grad gi0(gradP[i +      perm[j +      perm[k   ]]]);
	Grad gi1(gradP[i + i1 + perm[j + j1 + perm[k+k1]]]);
	Grad gi2(gradP[i + i2 + perm[j + j2 + perm[k+k2]]]);
	Grad gi3(gradP[i +  1 + perm[j +  1 + perm[k+ 1]]]);

	// Calculate the contribution from the four corners
	double t0 = 0.6 - x0*x0 - y0*y0 - z0*z0;
	if (t0 < 0)
		n0 = 0;
	else
	{
		t0 *= t0;
		n0 = t0 * t0 * gi0.dot3(x0, y0, z0);  // (x,y) of grad3 used for 2D gradient
	}
	double t1 = 0.6 - x1*x1 - y1*y1 - z1*z1;
	if (t1 < 0)
		n1 = 0;
	else
	{
		t1 *= t1;
		n1 = t1 * t1 * gi1.dot3(x1, y1, z1);
	}
	double t2 = 0.6 - x2*x2 - y2*y2 - z2*z2;
	if (t2 < 0)
		n2 = 0;
	else
	{
		t2 *= t2;
		n2 = t2 * t2 * gi2.dot3(x2, y2, z2);
	}
	double t3 = 0.6 - x3*x3 - y3*y3 - z3*z3;
	if (t3 < 0)
		n3 = 0;
	else
	{
		t3 *= t3;
		n3 = t3 * t3 * gi3.dot3(x3, y3, z3);
	}
	// Add contributions from each corner to get the final noise value.
	// The result is scaled to return values in the interval [-1,1].
	return 32 * (n0 + n1 + n2 + n3);
}

double Perlin::perlin2(double x, double y)
{
	// Find unit grid cell containing point
	int X = static_cast<int>(x), Y = static_cast<int>(y);
	// Get relative xy coordinates of point within that cell
	x -= X; y -= Y;
	// Wrap the integer cells at 255 (smaller integer period can be introduced here)
	X &= 255; Y &= 255;

	// Calculate noise contributions from each of the four corners
	double n00 = gradP[X +     perm[Y  ]].dot2(x,     y);
	double n01 = gradP[X +     perm[Y+1]].dot2(x,   y-1);
	double n10 = gradP[X + 1 + perm[Y  ]].dot2(x-1,   y);
	double n11 = gradP[X + 1 + perm[Y+1]].dot2(x-1, y-1);

	// Compute the fade curve value for x
	double u = fade(x);

	// Interpolate the four results
	return lerp( lerp(n00, n10, u), lerp(n01, n11, u), fade(y) );
}

double Perlin::perlin3(double x, double y, double z)
{
	// Find unit grid cell containing point
	int X = static_cast<int>(x), Y = static_cast<int>(y), Z = static_cast<int>(z);
	// Get relative xyz coordinates of point within that cell
	x -= X; y -= Y; z -= Z;
	// Wrap the integer cells at 255 (smaller integer period can be introduced here)
	X &= 255; Y &= 255; Z &= 255;

	// Calculate noise contributions from each of the eight corners
	double n000 = gradP[X +     perm[Y +     perm[Z  ]]].dot3(x,   y,     z);
	double n001 = gradP[X +     perm[Y +     perm[Z+1]]].dot3(x,   y,   z-1);
	double n010 = gradP[X +     perm[Y + 1 + perm[Z  ]]].dot3(x,   y-1,   z);
	double n011 = gradP[X +     perm[Y + 1 + perm[Z+1]]].dot3(x,   y-1, z-1);
	double n100 = gradP[X + 1 + perm[Y +     perm[Z  ]]].dot3(x-1,   y,   z);
	double n101 = gradP[X + 1 + perm[Y +     perm[Z+1]]].dot3(x-1,   y, z-1);
	double n110 = gradP[X + 1 + perm[Y + 1 + perm[Z  ]]].dot3(x-1, y-1,   z);
	double n111 = gradP[X + 1 + perm[Y + 1 + perm[Z+1]]].dot3(x-1, y-1, z-1);

	// Compute the fade curve value for x, y, z
	double u = fade(x);
	double v = fade(y);
	double w = fade(z);

	// Interpolate
	return lerp(
		lerp( lerp(n000, n100, u), lerp(n001, n101, u), w),
		lerp( lerp(n010, n110, u), lerp(n011, n111, u), w),
		v);
}

void PopulationMap::generate(double **populationMap, int granularity)
{
#ifdef _WIN32
	Perlin::seeding(rand() + rand() + 1); // 1 - 65535
#else
	Perlin::seeding(rand() % 65536); // 1 - 65535
#endif

	PopulationMap::halfWidth = gX / 2.0;
	PopulationMap::halfHeight = gY / 2.0;
	configPerlin();

	for (int i = 0; i < gX / granularity; ++i)
		for (int j = 0; j < gY / granularity; ++j)
			populationMap[i][j] = PopulationMap::populationAt(i*granularity, j*granularity);

#if 0
	FILE *fd = fopen("PopulationMap.csv", "w");
	if (fd == NULL)
	{
		fprintf(stderr, "Fail to open PopulationMap.csv!\n");
		exit(EXIT_FAILURE);
	}

	for (int i = 0; i < gX / granularity; ++i)
		for (int j = 0; j < gY / granularity; ++j)
			fprintf(fd, "%d,%d,%f\n", i*granularity, j*granularity, populationMap[i][j]);

	fclose(fd);
#endif
}

void PopulationMap::configPerlin()
{
	FILE *fd = fopen("Perlin.conf", "r");
	if (fd == NULL)
	{
		fprintf(stderr, "Fail to open Perlin.conf!\n");
		exit(EXIT_FAILURE);
	}

	fscanf(fd, "%lf,%lf,%lf\n", &PopulationMap::perturbation1, &PopulationMap::perturbation2, &PopulationMap::perturbation3);

	fclose(fd);
}

double PopulationMap::populationAt(double X, double Y)
{
	double value1 = (Perlin::simplex2(X / halfWidth / 2 + perturbation1, Y / halfHeight / 2 + perturbation1) + 1.0) / 2.0;
	double value2 = (Perlin::simplex2(X / halfWidth + perturbation2, Y / halfHeight + perturbation2) + 1.0) / 2.0;
	double value3 = (Perlin::simplex2(X / halfWidth + perturbation3, Y / halfHeight + perturbation3) + 1.0) / 2.0;
	return pow((value1 * value2 + value3) / 2.0, 2);
}

double PopulationMap::populationAt2(double X, double Y)
{
	double value1 = (Perlin::perlin2(X / halfWidth / 2 + perturbation1, Y / halfHeight / 2 + perturbation1) + 1.0) / 2.0;
	double value2 = (Perlin::perlin2(X / halfWidth + perturbation2, Y / halfHeight + perturbation2) + 1.0) / 2.0;
	double value3 = (Perlin::perlin2(X / halfWidth + perturbation3, Y / halfHeight + perturbation3) + 1.0) / 2.0;
	return pow((value1 * value2 + value3) / 2.0, 2);
}
