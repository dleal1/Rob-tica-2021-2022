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

    QRectF dimensions (-5300, -2800, 10600, 5600);
    viewer = new AbstractGraphicViewer(this, dimensions);
    this->resize(900,450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract


//    RoboCompFullPoseEstimation::FullPoseEuler r_state;
//    RoboCompLaser::TLaserData ldata;
//
//    try
//    {
//        ldata = laser_proxy->getLaserData();
//
//        r_state = fullposeestimation_proxy->getFullPoseEuler();
//        robot_polygon->setRotation(r_state.rz*180/M_PI);
//        robot_polygon->setPos(r_state.x, r_state.y);
//
//
//    }
//    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
    grid.initialize(dimensions, TILE_SIZE, &viewer->scene, false);
}

void SpecificWorker::compute() {
    RoboCompFullPoseEstimation::FullPoseEuler r_data;
    try {
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz * 180 / M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);
    }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

    RoboCompLaser::TLaserData ldata;
    try {
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
        update_map(ldata);
    }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

    static float initial_angle;
    static int current_room = 0;
    static Eigen::Vector2f center_room_w;

    switch (estado) {
        case Estado::IDLE: {
            std::cout << "Estado: IDLE" << std::endl;
            estado = Estado::INIT_TURN;
            break;
        }
        case Estado::INIT_TURN: {
            std::cout << "Estado: INIT_TURN" << std::endl;

            initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;

            try {
                differentialrobot_proxy->setSpeedBase(0.0, 0.5);
            }
            catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

            estado = Estado::EXPLORING;

            break;
        }
        case Estado::EXPLORING: {
            std::cout << "Estado: EXPLORING" << std::endl;
//            try
//            {
//                estado = exploring(ldata,r_state);
//            } catch (const Ice::Exception &e) {
//                std::cout << e.what() << std::endl;
//            }
//            break;

            float current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;

            if (fabs(current - initial_angle) < (M_PI + 0.1) and
                fabs(current - initial_angle) > (M_PI - 0.1)) {
                try {
                    differentialrobot_proxy->setSpeedBase(0.0, 0.0);
                }
                catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

                estado = Estado::DOOR;
            }

            // search for corners. Compute derivative wrt distance
            std::vector<float> derivatives(ldata.size());
            derivatives[0] = 0;
            for (const auto &&[k, l]: iter::sliding_window(ldata, 2) | iter::enumerate)
                derivatives[k + 1] = l[1].dist - l[0].dist;

            // filter derivatives greater than a threshold
            std::vector<Eigen::Vector2f> peaks;
            for (const auto &&[k, der]: iter::enumerate(derivatives)) {
                RoboCompLaser::TData l;
                if (der > 800) {
                    l = ldata.at(k - 1);
                    peaks.push_back(robot_to_world(Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
                } else if (der < -800) {
                    l = ldata.at(k);
                    peaks.push_back(robot_to_world(Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
                }
            }

            //pairwise comparison of peaks to filter in doors
            for (auto &&c: iter::combinations_with_replacement(peaks, 2)) {
                //qInfo() << __FUNCTION__ << "dist " << (c[0] - c[1]).norm();
                if ((c[0] - c[1]).norm() < 1100 and (c[0] - c[1]).norm() > 600) {
                    Door d{c[0], c[1]};
                    d.rooms.insert(current_room);
                    if (auto r = std::find_if(doors.begin(), doors.end(), [d](auto a) { return d == a; }); r == doors.end())
                        doors.emplace_back(d);
                }
            }

            // draw
            static std::vector<QGraphicsItem *> door_points;
            for (auto dp: door_points) viewer->scene.removeItem(dp);
            door_points.clear();
            for (const auto p: peaks) {
                door_points.push_back(viewer->scene.addRect(QRectF(p.x() - 100, p.y() - 100, 200, 200),
                                                            QPen(QColor("Magenta")), QBrush(QColor("Magenta"))));
                door_points.back()->setZValue(200);
            }

            break;
        }
        case Estado::DOOR: {
            std::cout << "Estado: DOOR" << std::endl;

            if (doors.size() > 0)
                selectedDoor = &doors[doors.size() - 1];

            way = selectedDoor->get_external_midpoint();

            selectedDoor->rooms.insert(current_room);
            estado = Estado::NEXTROOM;

            break;
        }
        case Estado::NEXTROOM: {
            std::cout << "Estado: NEXTROOM" << std::endl;

            // goto point
            auto tr = world_to_robot(selected_doors.front().get_external_midpoint());
            float dist = tr.norm();
            if (dist < 150)  // at target
            {
                //qInfo() << __FUNCTION__ << "    Robot reached target room" << data_state.next_room;
                try {
                    differentialrobot_proxy->setSpeedBase(0, 0);
                }
                catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

                if (auto obj_room = selected_doors.front().connecting_room(current_room); obj_room.has_value())
                    current_room = obj_room.value();
                else
                    current_room = rooms.size();

                //center point for new room
                float x = ldata.size() / 2;
                float d = x / 4;
                float laser_center = ldata[x].dist;
                float izq = ldata[d].dist;
                float der = ldata[ldata.size() - d].dist;
                Eigen::Vector2f center_r((der - izq) / 2, (-800 + laser_center) / 2);
                center_room_w = robot_to_world(center_r);

                //draw
                auto dest = viewer->scene.addEllipse(center_room_w.x() - 100, center_room_w.y() - 100, 200, 200,
                                                     QPen(QColor("Magenta"), 50));
                dest->setZValue(200);

                estado = Estado::CENTERROOM;
                break;
            } else  // continue to room
            {
                // call dynamic window
                QPolygonF laser_poly;
                for (auto &&l: ldata)
                    laser_poly << QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle));
                auto[_, __, adv, rot, ___] = dw.compute(tr, laser_poly,
                                                        Eigen::Vector3f(r_state.x, r_state.y, r_state.rz),
                                                        Eigen::Vector3f(r_state.vx, r_state.vy, r_state.vrz),
                                                        nullptr /*&viewer_robot->scene*/);
                const float rgain = 0.8;
                float rotation = rgain * rot;
                float dist_break = std::clamp(world_to_robot(target.to_eigen()).norm() / 1000.0, 0.0, 1.0);
                float advance = constants.MAX_ADV_SPEED * dist_break * rotation_speed(rotation);

                try{
                    differentialrobot_proxy->setSpeedBase(advance, rotation);
                }
                catch(const Ice::Exception &e){std::cout << e.what() << std::endl;}

                break;
            }
        }
        case Estado::CENTERROOM:
        {
            auto tr = robot_to_world(center_room_w);
            float dist = tr.norm();

            if(dist < 80)
            {
                try{
                    differentialrobot_proxy->setSpeedBase(0, 0);
                }
                catch(const Ice::Exception &e){std::cout << e.what() << std::endl;}

                qInfo() << "ARRIVED";
                break;
            }
            float beta = 0.0;

            if(dist > 80)
                beta = atan2(tr.x(), tr.y());
            float adv_break = std::clamp(constants.MAX_ADV_SPEED / 1000.0, 0.0, 1.0);
            float adv = constants.MAX_ADV_SPEED * adv_break * rotation_speed(beta);
            try{
                differentialrobot_proxy->setSpeedBase(adv, beta);
            }
            catch(const Ice::Exception &e){std::cout << e.what() << std::endl;}
            break;
        }
    }
}

void SpecificWorker::update_map(const RoboCompLaser::TLaserData &ldata)
{
    Eigen::Vector2f lw;

    for (const auto &l: ldata)
    {
        if(l.dist > constants.ROBOT_LENGTH/2)
        {
            Eigen::Vector2f tip(l.dist * sin(l.angle), l.dist * cos(l.angle));
            Eigen::Vector2f p = robot_to_world(tip);
            int target_kx = (p.x() - grid.dim.left()) / grid.TILE_SIZE;
            int target_kz = (p.y() - grid.dim.bottom()) / grid.TILE_SIZE;
            int last_kx = -1000000;
            int last_kz = -1000000;

            int num_steps = ceil(l.dist / (constants.TILE_SIZE / 2.0));
            for(const auto &&step : iter::range(0.0, 1.0 - (1.0 / num_steps), 1.0 / num_steps))
            {
                Eigen::Vector2f p = robot_to_world(tip * step);
                int kx = (p.x() - grid.dim.left()) / grid.TILE_SIZE;
                int kz = (p.y() - grid.dim.bottom()) / grid.TILE_SIZE;
                if(kx != last_kx and kx !=  target_kx and kz != last_kz and kz != target_kz)
                    grid.add_miss(robot_to_world(tip * step));
                last_kx = kx;
                last_kz = kz;
            }
            if(l.dist <= constants.MAX_LASER_DIST)
                grid.add_hit(robot_to_world(tip));
//            else
//                grid.add_miss(robot_to_world(tip));
        }
    }
}

float SpecificWorker::rotation_speed(float beta) {

    static float lambda = -(0.5 * 0.5) / log(0.1);

    return exp(-(beta * beta) / lambda);
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

Eigen::Vector2f SpecificWorker::world_to_robot(const Eigen::Vector2f &point) {

	float angulo = r_state.rz;
	Eigen::Vector2f posrobot(r_state.x, r_state.y);
	Eigen::Matrix2f matriz(2,2);

	matriz << cos(angulo), -sin(angulo), sin(angulo), cos(angulo);

    return (matriz.transpose() * (point - posrobot));
}

Eigen::Vector2f SpecificWorker::robot_to_world(const Eigen::Vector2f &point)
{
    float angulo = r_state.rz;
    Eigen::Vector2f posrobot(r_state.x,r_state.y);
    Eigen::Matrix2f matriz(2,2);

    matriz << cos(angulo), -sin(angulo), sin(angulo), cos(angulo);

    return matriz * point + posrobot;
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

