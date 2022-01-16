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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <eigen3/Eigen/Eigen>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
    void draw_laser(const RoboCompLaser:: TLaserData & ldata);
	void initialize(int period);
    void new_target_slot(QPointF point);

private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;

    AbstractGraphicViewer *viewer;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    QPointF last_point;

    const int ROBOT_LENGTH = 400;
    const float max_adv_speed = 1000;

    struct Target {
        QPointF dest;
        bool activo;
        float A,B,C;
        Eigen::Vector2f to_eigen() const {return Eigen::Vector2f(dest.x(), dest.y());}
    };
    Target target;

    enum class Estado {IDLE, FORWARD, TURN, BORDER};
    Estado estado;

    void forward(RoboCompGenericBase::TBaseState bState);
    float dist_to_target(float dist);
    float rotation_speed(float beta);

    bool check_free_path_to_target(const RoboCompLaser::TLaserData &ldata, RoboCompGenericBase::TBaseState bState);

    Eigen::Vector2f world_to_robot(const Eigen::Vector2f &p, RoboCompGenericBase::TBaseState bState);
    Eigen::Vector2f robot_to_world(const Eigen::Vector2f &p, RoboCompGenericBase::TBaseState bState);

    bool check_obstacle(const RoboCompLaser::TLaserData &ldata, int dist);

    void doBorder(const RoboCompLaser::TLaserData &ldata, RoboCompGenericBase::TBaseState bState);
};

#endif
