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
#include <cppitertools/range.hpp>
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

    QRectF dimensions (-5100, -2600, 10200, 5200);
    viewer = new AbstractGraphicViewer(this, dimensions);
    this->resize(1200,600);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract


    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    RoboCompLaser::TLaserData ldata;

    try
    {
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);

        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz*180/M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);


    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
    estado = Estado::IDLE;
    grid.initialize(dimensions, 200, &viewer->scene);
}

void SpecificWorker::compute()
{
    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    RoboCompLaser::TLaserData ldata;

    try
    {
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz*180/M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);
        ldata = laser_proxy->getLaserData();
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    update_map(ldata, r_state);

    switch(estado)
    {
        case Estado::IDLE:
            std::cout << "Estado: IDLE" << std::endl;
            estado = Estado::INIT_TURN;
            break;

        case Estado::INIT_TURN:
            std::cout << "Estado: INIT_TURN" << std::endl;
            initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            estado = exploring(ldata, r_state);
            break;

        case Estado::EXPLORING:
            std::cout << "Estado: EXPLORING" << std::endl;
            try
            {
                estado = exploring(ldata,r_state);
            } catch (const Ice::Exception &e) {
                std::cout << e.what() << std::endl;
            }
            break;

        case Estado::DOOR:
            std::cout << "Estado: DOOR" << std::endl;
            try
            {
                estado = doorS(ldata, r_state);
            } catch (const Ice::Exception &e) {
                std::cout << e.what() << std::endl;
            }
            break;
    }
  }

void SpecificWorker::update_map(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    Eigen::Vector2f lineP;
    Eigen::Vector2f lw;

    for (auto &l:ldata) {
        float step = ceil(l.dist / (TILE_SIZE / 2.0));
        lw = Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle));
        lineP = robot_to_world(lw, r_state);
        float lastX = -1000000;
        float lastY = -1000000;
        float tarX = (lineP.x() - grid.dim.left()) / grid.TILE_SIZE;
        float tarY = (lineP.y() - grid.dim.bottom()) / grid.TILE_SIZE;
        for (const auto &&step: iter::range(0.0, 1.0 - (1.0 / step), 1.0 / step)) {
            lineP = robot_to_world( lw * step, r_state);
            float kx = (lineP.x() - grid.dim.left()) / grid.TILE_SIZE;
            float ky = (lineP.y() - grid.dim.bottom()) / grid.TILE_SIZE;
            if (kx != lastX && kx != tarX && ky != lastY && ky != tarY) {
                lineP = robot_to_world(lw * step, r_state);
                grid.add_miss(Eigen::Vector2f(lineP.x(), lineP.y()));
            }
            lastX = kx;
            lastY = ky;
        }

        if (l.dist <= MAX_LASER_DIST) {
            lineP = robot_to_world(lw, r_state);
            grid.add_hit(Eigen::Vector2f(lineP.x(), lineP.y()));
        }
    }
}

SpecificWorker::Estado SpecificWorker::exploring(const RoboCompLaser::TLaserData &ldata,const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    float current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;

    float stop = grid.percentage_changed()*1000;
    cout << "STOP VALUE: " << stop << endl;

    //update_map(ldata, r_state);

    if (fabs(current - initial_angle) < (M_PI + 0.1) and
           fabs(current - initial_angle) > (M_PI - 0.1))
    {
        differentialrobot_proxy->setSpeedBase(0,0);
        return Estado::DOOR;
    } else {
        // search for corners. Compute derivative wrt distance
        std::vector<float> derivatives(ldata.size());
        derivatives[0] = 0;
        for (const auto &&[k, l]: iter::sliding_window(ldata, 2) | iter::enumerate) {
            derivatives[k + 1] = l[1].dist - l[0].dist;
        }

        // filter derivatives greater than a threshold
        std::vector<Eigen::Vector2f> peaks;
        for (const auto &&[k, der]: iter::enumerate(derivatives)) {
            RoboCompLaser::TData l;
            if (der > 800) {
                l = ldata.at(k - 1);
                peaks.push_back(robot_to_world(Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle)), r_state));
            } else if (der < -800) {
                l = ldata.at(k);
                peaks.push_back(robot_to_world(Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle)), r_state));
            }
        }
        for (auto &&c: iter::combinations_with_replacement(peaks, 2)) {
            if ((c[0] - c[1]).norm() < 1100 and (c[0] - c[1]).norm() > 900) {
                Door d{c[0], c[1]};
                //d.rooms.insert(current_room);
                if (auto r = std::find_if(doors.begin(), doors.end(),
                                          [d](auto a) { return d == a; }); r == doors.end())
                    doors.emplace_back(d);
            }
        }
        cout << "Puertas: " << doors.size() << endl;

        static std::vector<QGraphicsItem*> door_points;
        for(auto dp : door_points) viewer->scene.removeItem(dp);
        door_points.clear();
        for(const auto p: peaks)
        {
            door_points.push_back(viewer->scene.addRect(QRectF(p.x()-100, p.y()-100, 200, 200),
                                                        QPen(QColor("Magenta")), QBrush(QColor("Magenta"))));
            door_points.back()->setZValue(200);
        }

        try {
            differentialrobot_proxy->setSpeedBase(0, 1);
        } catch (const Ice::Exception &e) {
            std::cout << e.what() << std::endl;
        }
        return Estado::EXPLORING;
    }
}

SpecificWorker::Estado SpecificWorker::doorS(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    float threshold = 500;
    static RoboCompLaser::TData best_distance_previous = ldata[180];
    RoboCompLaser::TData actual_distance = ldata[180];
    static int doors2 = 0;
    int cx, cy;

    if(doors2 == 2)
    {
        doors2 = 0;
        try
        {
            differentialrobot_proxy->setSpeedBase(0,0);
        } catch (const Ice::Exception &e) {
            std::cout << e.what() << std::endl;
        }
        return Estado::NEXTROOM;
    }
}

void SpecificWorker::draw_laser(const RoboCompLaser:: TLaserData &ldata)
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

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker :: new_target_slot(QPointF point){

    qInfo() << point;

    target.dest = point;
    target.activo = true;

}

QPointF SpecificWorker::world_to_robot(RoboCompFullPoseEstimation::FullPoseEuler &r_state) {

	float angulo = r_state.rz;
	Eigen::Vector2f posrobot(r_state.x, r_state.y), point_in_world(target.dest.x(), target.dest.y());
	Eigen::Matrix2f matriz(2,2);

	matriz << cos(angulo), sin(angulo), -sin(angulo), cos(angulo);

	Eigen::Vector2f estado = matriz * (point_in_world - posrobot);

    std::cout << estado[0] << "\t" << estado[1] << std::endl;

    return QPointF(estado[0], estado[1]); // crear matriz con eigen 2f. bstate.angle
}

Eigen::Vector2f SpecificWorker::robot_to_world(Eigen::Vector2f TW, const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    float angulo = r_state.rz;
    Eigen::Vector2f posrobot(r_state.x,r_state.y);
    Eigen::Matrix2f matriz(2,2);

    matriz << cos(angulo), sin(angulo), -sin(angulo), cos(angulo);

    auto estado = matriz * TW + posrobot;

    return estado;
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

