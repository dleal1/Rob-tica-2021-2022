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
            std::cout << "Estado: IDLE" << std::endl;
            if(target.activo)
                estado = Estado::FORWARD;
            break;

        case Estado::FORWARD:
            std::cout << "Estado: FORWARD" << std::endl;
            if(check_obstacle(ldata, 500))
                estado = Estado::TURN;
            else
                forward(bState);
            break;

        case Estado::TURN:
            std::cout << "Estado: TURN" << std::endl;
            if(!check_obstacle(ldata, 200))
                estado = Estado::BORDER;
            else
                differentialrobot_proxy->setSpeedBase(0, 0.6);
            break;

        case Estado::BORDER:
            std::cout << "Estado: BORDER" << std::endl;
            if(check_free_path_to_target(ldata, bState))
                estado = Estado::FORWARD;
            else
                doBorder(ldata);
            break;
    }
}

void SpecificWorker::forward(RoboCompGenericBase::TBaseState bState) {
    Eigen::Vector2f robot_eigen(bState.x, bState.z);
    Eigen::Vector2f target_eigen (target.dest.x(), target.dest.y());

    if (float dist = (robot_eigen - target_eigen).norm(); dist > 100)
    {
        // convertir el target a coordenadas del robot
        QPointF pr = world_to_robot(target, bState);
        // obtener el angulo beta (robot - target)
        float beta = atan2(pr.x(), pr.y());
        // obtener velocidad de avance (primero a 0)
        float adv = max_adv_speed * dist_to_target(dist) * rotation_speed(beta);
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

void SpecificWorker::doBorder(const RoboCompLaser::TLaserData &ldata)
{
    if(lateral_dist(ldata, 200))
    {
        differentialrobot_proxy->setSpeedBase(300, -0.6);
    }
}

bool SpecificWorker::check_obstacle(const RoboCompLaser::TLaserData &ldata, int dist)
{
    auto min = std::min_element(ldata.begin()+(ldata.size()/2 - 10), ldata.end()-(ldata.size()/2 + 10), [](auto a, auto b) { return a.dist < b.dist; });
    return (*min).dist < dist;
}

bool SpecificWorker::lateral_dist(const RoboCompLaser::TLaserData &ldata, int dist)
{
    return (std::min_element(ldata.begin() + 20, ldata.begin() + 50, [](auto a, auto b) { return a.dist < b.dist; }))->dist < dist;
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

Eigen::Vector2f SpecificWorker::robot_to_world(Eigen::Vector2f p, RoboCompGenericBase::TBaseState bState)
{
    float angle = bState.alpha;
    Eigen::Vector2f posRobot(bState.x, bState.z);
    Eigen::Matrix2f M(2,2);

    M << cos(angle), sin(angle), -sin(angle), cos(angle);

    return M.inverse() * (p - posRobot);
}

float SpecificWorker::dist_to_target(float dist)
{
    if(dist > 1000)
        return 1;
    else
        return (1.0/1000.0) * dist;
}

float SpecificWorker::rotation_speed(float beta) {

    static float lambda = -(0.5 * 0.5) / log(0.1);

    return exp(-(beta * beta) / lambda);
}

bool SpecificWorker::check_free_path_to_target( const RoboCompLaser::TLaserData &ldata,
                                                RoboCompGenericBase::TBaseState bState)
{
    bool free_path = true;
    // lambda to convert from Eigen to QPointF
    auto toQPointF = [](const Eigen::Vector2f &p){ return QPointF(p.x(),p.y());};

    // create polyggon
    QPolygonF pol;
    pol << QPointF(0,0);
    for(const auto &l: ldata)
        pol << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));

    // create tube lines
    auto goal_r = world_to_robot(target, bState);
    Eigen::Vector2f robot(0.0,0.0);
    Eigen::Vector2f vgoal_r(goal_r.x(), goal_r.y());
    // number of parts the target vector is divided into
    float parts = (vgoal_r).norm()/(ROBOT_LENGTH/4);
    Eigen::Vector2f rside(220, 200);
    Eigen::Vector2f lside(-220, 200);
    if(parts < 1) return false;

    QPointF p,q, r;
    for(float l=0.0; l <= 1.0; l+=1.0/parts)
    {
        p = toQPointF(robot*(1-l) + vgoal_r*l);
        q = toQPointF((robot+rside)*(1-l) + (vgoal_r+rside)*l);
        r = toQPointF((robot+lside)*(1-l) + (vgoal_r+lside)*l);
        if( not pol.containsPoint(p, Qt::OddEvenFill) or
            not pol.containsPoint(q, Qt::OddEvenFill) or
            not pol.containsPoint(r, Qt::OddEvenFill)) {
            free_path = false;

            break;
        } else
            free_path = true;
    }

    // draw
    QLineF line_center(toQPointF(robot_to_world(robot, bState)), toQPointF(robot_to_world(Eigen::Vector2f(p.x(),p.y()), bState)));
    QLineF line_right(toQPointF(robot_to_world(robot+rside, bState)), toQPointF(robot_to_world(Eigen::Vector2f(q.x(),q.y()), bState)));
    QLineF line_left(toQPointF(robot_to_world(robot+lside, bState)), toQPointF(robot_to_world(Eigen::Vector2f(r.x(),q.y()), bState)));
    static QGraphicsItem *graphics_line_center = nullptr;
    static QGraphicsItem *graphics_line_right = nullptr;
    static QGraphicsItem *graphics_line_left = nullptr;
    static QGraphicsItem *graphics_target = nullptr;
    if (graphics_line_center != nullptr)
        viewer->scene.removeItem(graphics_line_center);
    if (graphics_line_right != nullptr)
        viewer->scene.removeItem(graphics_line_right);
    if (graphics_line_left != nullptr)
        viewer->scene.removeItem(graphics_line_left);
    if (graphics_target != nullptr)
        viewer->scene.removeItem(graphics_target);
    graphics_line_center = viewer->scene.addLine(line_center, QPen(QColor("Blue"), 30));
    graphics_line_right = viewer->scene.addLine(line_right, QPen(QColor("Orange"), 30));
    graphics_line_left = viewer->scene.addLine(line_left, QPen(QColor("Magenta"), 30));
    graphics_target = viewer->scene.addEllipse(-100, -100, 200, 200, QPen(QColor("Blue")), QBrush(QColor("Blue")));
    graphics_target->setPos(target.dest.x(), target.dest.y());

    return free_path;
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

