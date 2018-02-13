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

#include "mainwindow.h"
#include "ui_mainwindow.h"

double gX = 0.0;
double gY = 0.0;
int numUser = 0;
Point *groundUsers = NULL;
double *rateTable = NULL;
int *servedTable = NULL;

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow), solution()
{
	ui->setupUi(this);
	setGeometry(400, 100, 600, 600); //  1120

	numAvailableUAV = parseInput("case.txt");
	configureUAV();

	plotUsers();

	solution.initialize(numAvailableUAV);
	uavs.reserve(64);

	// setup a timer that repeatedly calls MainWindow::deployOne()
	connect(&dataTimer, SIGNAL(timeout()), this, SLOT(deployOne()));
	dataTimer.start(1000);
}

MainWindow::~MainWindow()
{
	solution.result("UAVs.csv", "servedUsers.csv");

	delete []groundUsers;
	delete []rateTable;
	delete []servedTable;
	delete ui;
}

void MainWindow::plotUsers()
{
	QPen pen;
	pen.setColor(Qt::blue);
	pen.setStyle(Qt::SolidLine);

	ui->customPlot->addGraph();
	ui->customPlot->graph(0)->setPen(pen);
	ui->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
	ui->customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 2));

	QVector<double> xCoord(numUser), yCoord(numUser);
	for (int i = 0; i < numUser; ++i)
	{
		xCoord[i] = groundUsers[i].x;
		yCoord[i] = groundUsers[i].y;
	}
	// configure right and top axis to show ticks but no labels:
	ui->customPlot->xAxis2->setVisible(true);
	ui->customPlot->xAxis2->setTickLabels(false);
	ui->customPlot->yAxis2->setVisible(true);
	ui->customPlot->yAxis2->setTickLabels(false);
	// make left and bottom axes always transfer their ranges to right and top axes:
	connect(ui->customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->xAxis2, SLOT(setRange(QCPRange)));
	connect(ui->customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->customPlot->yAxis2, SLOT(setRange(QCPRange)));
	// pass data points to graphs:
	ui->customPlot->graph(0)->setData(xCoord, yCoord);
	// give the axes some labels:
	// ui->customPlot->xAxis->setLabel("x");
	// ui->customPlot->yAxis->setLabel("y");
	// set axes ranges, so we see all data:
	ui->customPlot->xAxis->setRange(0, gX);
	ui->customPlot->yAxis->setRange(0, gY);
	// let the ranges scale themselves so graph 0 fits perfectly in the visible area:
	// ui->customPlot->graph(0)->rescaleAxes();
	// Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
	ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

	ui->customPlot->replot();
}

void MainWindow::deployOne()
{
	int numDeployedUAV = static_cast<int>(solution.UAVs.size());
	if (numDeployedUAV < numAvailableUAV)
	{
		std::vector<size_t> indices;
		solution.deployOne(indices);

		UAV &curUAV = solution.UAVs.back();
		QPen pen(curUAV.getType() == 1 ? Qt::SolidLine : Qt::DashLine);
		switch (curUAV.getBand())
		{
		case Band::_2_1: pen.setColor(Qt::darkGreen); break;
		case Band::_2_2: pen.setColor(Qt::darkCyan); break;
		case Band::_2_3: pen.setColor(Qt::darkYellow); break;
		case Band::_4_1: pen.setColor(Qt::darkGreen); break;
		case Band::_4_2: pen.setColor(Qt::darkCyan); break;
		case Band::_4_3: pen.setColor(Qt::darkYellow); break;
		case Band::_4_4: pen.setColor(Qt::magenta); break;
		}
		QCPItemEllipse *ellipse = new QCPItemEllipse(ui->customPlot);
		ellipse->topLeft->setCoords(curUAV.getX() - curUAV.getR(), curUAV.getY() + curUAV.getR());
		ellipse->bottomRight->setCoords(curUAV.getX() + curUAV.getR(), curUAV.getY() - curUAV.getR());
		ellipse->setPen(pen);
		uavs.push_back(ellipse);

		for (size_t k = 0; k < indices.size(); ++k)
		{
			UAV &oldUAV = solution.UAVs[indices[k]];
			QCPItemEllipse *&oldEllipse = uavs[indices[k]];
			oldEllipse->topLeft->setCoords(oldUAV.getX() - oldUAV.getR(), oldUAV.getY() + oldUAV.getR());
			oldEllipse->bottomRight->setCoords(oldUAV.getX() + oldUAV.getR(), oldUAV.getY() - oldUAV.getR());
		}

		ui->customPlot->replot();
	}
}

