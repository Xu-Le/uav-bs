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

#ifndef __PERLIN_H__
#define __PERLIN_H__

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

class Perlin
{
public:
	class Grad
	{
	public:
		Grad() : x(0.0), y(0.0), z(0.0) {}
		Grad(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
		Grad(const Grad& rhs) : x(rhs.x), y(rhs.y), z(rhs.z) {}
		~Grad() {}

		Grad& operator=(const Grad& rhs) { x = rhs.x; y = rhs.y; z = rhs.z; return *this;  }
		double dot2(double X, double Y) { return x * X + y * Y; }
		double dot3(double X, double Y, double Z) { return x * X + y * Y + z * Z; }

	public:
		double x;
		double y; 
		double z;
	};

	static void seeding(uint16_t seed);
	static double simplex2(double xin, double yin);			 // 2D simplex noise
	static double simplex3(double xin, double yin, double zin); // 3D simplex noise
	static double fade(double t) { return t * t * t * (t*(t*6.0 - 15.0) + 10.0); }
	static double lerp(double a, double b, double t) { return (1.0 - t) * a + t * b; }
	static double perlin2(double x, double y);					// 2D Perlin Noise
	static double perlin3(double x, double y, double z);		// 3D Perlin Noise

public:
	// To remove the need for index wrapping, double the permutation table length
	static uint8_t perm[512];
	static uint8_t p[256];
	static Grad gradP[512];
	static Grad grad3[12];
	// Skewing and unskewing factors for 2, 3 dimensions
	static double F2;
	static double G2;
	static double F3;
	static double G3;
};

class PopulationMap
{
public:
	static void generate(double **populationMap, int granularity = 10);

private:
	static double populationAt(double X, double Y);
	static double populationAt2(double X, double Y);
	static void configPerlin();

public:
	static double halfWidth;
	static double halfHeight;
	static double perturbation1;
	static double perturbation2;
	static double perturbation3;
};

#endif
