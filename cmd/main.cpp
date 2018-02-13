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

double gX = 0.0;
double gY = 0.0;
int numUser = 0;
Point *groundUsers = NULL;
double *rateTable = NULL;
int *servedTable = NULL;

static void printHelp()
{
	printf("Usage:\n    ./UAV case_x.txt\n");
	printf("Example:\n    ./UAV case_1.txt\n");
}

static void deleteGlobal()
{
	delete []groundUsers;
	groundUsers = NULL;
	delete []rateTable;
	rateTable = NULL;
	delete []servedTable;
	servedTable = NULL;
}

int main(int argc, char *argv[])
{
	if (argc < 2 || argc > 3)
	{
		printHelp();
		exit(EXIT_FAILURE);
	}

	printf("Current log level: %s\n\n", getLogLevel());

	int numAvailableUAV = parseInput(argv[1]);
	configureUAV();

	{
		Solution solution;
		solution.deploy(numAvailableUAV, argc == 3 ? argv[2] : NULL);
		solution.result("UAVs.csv", "servedUsers.csv");
	}

	deleteGlobal();
	return 0;
}
