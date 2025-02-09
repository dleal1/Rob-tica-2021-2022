/*
 *    Copyright (C) 2021 by DANIEL LEAL MIRANDA & ALEJANDRO GONZALEZ FERNANDEZ
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/

SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	this->condicion = rand() % 3;
    this->espiral = false;
    this->contador = 0;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}

void SpecificWorker::compute( )
{
    const float threshold = 500; // millimeters
    const float limiteRot = 2;// rads per second

    float va = 200;
    float rot = 0.6;

    try
    {
        // read laser data
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        //sort laser data from small to large distances using a lambda function.
        std::sort( ldata.begin() + 10, ldata.end() - 10, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
		
        if (rot > limiteRot){
            rot = 0.6;
        }

        if(ldata[10].dist <= threshold)
        {
            switch(condicion) {
                case 0:             //GIRO INVERSO
                    std::cout << "Movimiento Giro Inverso" << std::endl;

                    //ldata = laser_proxy->getLaserData();
                    //std::sort(ldata.begin() + 10, ldata.end() - 10, [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });
                    rot = rot - (M_PI_2);

                    if(abs(rot) > limiteRot) {
                        rot = 0.6;
                    }

                    differentialrobot_proxy->setSpeedBase(50, rot);
                    usleep(rand() % (1500000 - 100000 + 1) + 100000);

                    condicion = 2;
                    break;

                case 1:
                    std::cout << "Movimiento 90º" << std::endl;

                    rot = M_PI/2;
                    differentialrobot_proxy->setSpeedBase(50, M_PI/2);
                    usleep(rand() % (1500000 - 100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec

                    condicion = 2;
                    break;

                case 2:
                    std::cout << "Movimiento Aumento Rotacion" << std::endl;

                    ldata = laser_proxy->getLaserData();
                    std::sort(ldata.begin() + 10, ldata.end() - 10, [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });

                    if (ldata[10].dist <= threshold){
                        differentialrobot_proxy->setSpeedBase(50, rot);
                        usleep(rand() % (1500000 - 100000 + 1) + 100000);
                    }
                    else{
                        differentialrobot_proxy->setSpeedBase(50, 0);
                        usleep(rand() % (1500000 - 100000 + 1) + 100000);

                    }

                    differentialrobot_proxy->setSpeedBase(50, 0);
                    usleep(rand() % (1500000 - 100000 + 1) + 100000);

                    break;
            }

        }
        else
        {
            switch (condicion2){
                case 0:
                    while(ldata[10].dist > threshold && va<1000 && !espiral) {
                        std::cout << "Movimiento Espiral " << std::endl;

                        ldata = laser_proxy->getLaserData();
                        std::sort(ldata.begin() + 10, ldata.end() - 10,
                                  [](RoboCompLaser::TData a, RoboCompLaser::TData b) { return a.dist < b.dist; });

                        differentialrobot_proxy->setSpeedBase(va, 0.8);
                        va += 20;
                    }
                    condicion2++;
                    espiral = true;

                    break;

                case 1:
                    std::cout << "Movimiento recto" << std::endl;

                    differentialrobot_proxy->setSpeedBase(800, 0);
                    break;
            }
            condicion = rand() % 2;
        }
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

