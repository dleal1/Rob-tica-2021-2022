/*
 *    Copyright (C) 2021 by DANIEL LEAL MIRANDA & ALEJANDRO GONZÁLEZ FERNANDEZ
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
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

    QRectF dimensions (-5000, -2500, 10000, 5000);
    viewer = new AbstractGraphicViewer(this, dimensions);
    this->resize(900,450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract

    try
    {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        last_point = QPointF(bState.x, bState.z);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

    estado = Estado::IDLE;
}

void SpecificWorker::compute()
{
    RoboCompGenericBase::TBaseState bState;

        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        draw_laser(ldata);

            differentialrobot_proxy->getBaseState(bState);
            robot_polygon->setRotation(bState.alpha*180/M_PI);
            robot_polygon->setPos(bState.x, bState.z);

    switch(estado)
    {
        case Estado::IDLE:
            std::cout << "Estado -> IDLE " << std::endl;

            if(target.activo)
                estado = Estado::FORWARD;
            break;

        case Estado::FORWARD:
            std::cout << "Estado -> FORWARD, MIN-DIST -> " << umbral_Obst_dist(ldata, (ldata.size()/2) - 30, ldata.size()/2) << std::endl;

            if(!umbral_Obst(ldata, (ldata.size()/2) - 30, ldata.size()/2, 400))
                estado = Estado::TURN;
            else
                forward(bState);
            break;

        case Estado::BORDER:

            break;

        case Estado::TURN:
            std::cout << "Estado -> TURN, MIN-DIST -> " << umbral_Obst_dist(ldata, (ldata.size()/2) - 30, ldata.size()/2) << std::endl;

            if (umbral_Obst(ldata, (ldata.size()/2) - 30, ldata.size()/2, 400))
                estado = Estado::FORWARD;
            else {

                differentialrobot_proxy->setSpeedBase(0, 1);
            }
            break;
    }
  }

void SpecificWorker :: draw_laser(const RoboCompLaser:: TLaserData &ldata)
{
    static QGraphicsItem *laser_polygon = nullptr;
    QPolygonF poly;

    if(laser_polygon != nullptr) {
        viewer->scene.removeItem(laser_polygon);
    }

    poly << QPointF(0,0);

    for(auto &p : ldata)
    {
        float x = p.dist * sin(p.angle);
        float y = p.dist * cos(p.angle);
        poly << QPointF(x,y);
    }

    QColor color ("Verde claro");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}

void SpecificWorker :: new_target_slot(QPointF point){

    qInfo() << point;       // muestra coordenadas con click

    target.dest = point;
    target.activo = true;

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

QPointF SpecificWorker::world_to_robot(SpecificWorker::Target target, RoboCompGenericBase::TBaseState state) {

	float angulo = state.alpha;
	Eigen::Vector2f posdest(target.dest.x(), target.dest.y()), posrobot(state.x, state.z);
	Eigen::Matrix2f matriz(2,2);
	
	matriz << cos(angulo), sin(angulo), -sin(angulo), cos(angulo);
	
	Eigen::Vector2f estado = matriz * (posdest - posrobot);

    std::cout << estado[0] << "\t" << estado[1] << std::endl;

    return QPointF(estado[0], estado[1]); // crear matriz con eigen 2f. bstate.angle
}


float SpecificWorker::dist_to_target(float dist)
{
    if(dist > 1000)
        return 1;
    else
        return (1.0/1000.0) * dist;
}

float SpecificWorker::dist_to_obstacle( RoboCompLaser::TLaserData ldata)
{
    int minDistance = umbral_Obst_dist(ldata, (ldata.size()/2) - ldata.size()/2, 10);
    if(minDistance > 1000)
        return 1;
    else
        return (1.0/1000.0) * minDistance;
}

float SpecificWorker::rotation_speed(float beta) {

    static float lambda = -(0.5 * 0.5) / log(0.1);

    return exp(-(beta * beta) / lambda);
}

bool SpecificWorker::umbral_Obst(RoboCompLaser::TLaserData ldata, int a, int b, int threshold)
{
    auto min = std::min_element(ldata.begin() + a, ldata.begin() + a + b, [](auto a, auto b){ return a.dist < b.dist;});

    return (*min).dist > threshold;
}

int SpecificWorker::umbral_Obst_dist(RoboCompLaser::TLaserData ldata, int a, int b)
{
    auto min = std::min_element(ldata.begin() + a, ldata.begin() + a + b, [](auto a, auto b){ return a.dist < b.dist;});

    return (*min).dist;
}

void SpecificWorker::forward(RoboCompGenericBase::TBaseState bState) {
    Eigen::Vector2f robot_eigen(bState.x, bState.z);
    Eigen::Vector2f target_eigen (target.dest.x(), target.dest.y());

    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
    std::sort( ldata.begin() + 10, ldata.end() - 10, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return a.dist < b.dist;});

    if (float dist = (robot_eigen - target_eigen).norm(); dist > 100)
    {
        // convertir el target a coordenadas del robot
        QPointF pr = world_to_robot(target, bState);
        // obtener el angulo beta (robot - target)
        float beta = atan2(pr.x(), pr.y());
        // obtener velocidad de avance (primero a 0)
        float adv = max_adv_speed * dist_to_target(dist) * rotation_speed(beta) * dist_to_obstacle(ldata);
        // mover el robot con la velocidad obtenida

        try {
            differentialrobot_proxy->setSpeedBase(adv, beta);
        }
        catch (const Ice::Exception &ex) {
            std::cout << ex << std::endl;
        }
    } else {
        target.activo = false;
        estado = Estado::IDLE;
        differentialrobot_proxy->setSpeedBase(0, 0);
    }
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

