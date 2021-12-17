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
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations_with_replacement.hpp>
#include <grid2d/grid.h>

#include <vector>
#include <utility>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
    void initialize(int period);
	void compute();
	int startup_check();

    void draw_laser(const RoboCompLaser:: TLaserData & ldata);
    void new_target_slot(QPointF p);

private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;

    AbstractGraphicViewer *viewer;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    QPointF last_point;

    float initial_angle = 0.0;

    struct Target {
        QPointF dest;
        bool activo;
    };
    Target target;

    Grid grid;
    const int TILE_SIZE = 200;
    const int MAX_LASER_DIST = 4000;
    RoboCompFullPoseEstimation::FullPoseEuler r_state;

    QLineF door;
    QPointF target_to_robot;

    enum class Estado{IDLE, INIT_TURN, EXPLORING, DOOR, NEXTROOM, CENTERROOM};
    Estado estado;
    std::vector<int> VectorLinea;

    struct Door
    {
        Eigen::Vector2f dPoint1, dPoint2;
        std::set<int> rooms;
        bool operator==(const Door &d1)
        {
            const int EROR = 500;
            return (dPoint1 - d1.dPoint1).norm() < EROR and (dPoint2 - d1.dPoint2).norm() < EROR or
            (dPoint1 - d1.dPoint2).norm() < EROR and (dPoint2 - d1.dPoint1).norm() < EROR;
        };

        Eigen::Vector2f get_midpoint() const {return dPoint1 + ((dPoint2-dPoint1)/2.0);};
        Eigen::Vector2f get_external_midpoint() const
        {
            Eigen::ParametrizedLine<float, 2> r =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (dPoint1-dPoint2).unitOrthogonal());
            //qInfo() << __FUNCTION__ << r.pointAt(800.0).x() << r.pointAt(800.0).y();
            return r.pointAt(1000.0);
        };
    };

    std::vector<Door> doors;

    QPointF world_to_robot(RoboCompFullPoseEstimation::FullPoseEuler &r_state);
    Eigen::Vector2f robot_to_world(Eigen::Vector2f TW, const RoboCompFullPoseEstimation::FullPoseEuler &bState);

    void update_map(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state);

    SpecificWorker::Estado exploring(const RoboCompLaser::TLaserData &ldata,const RoboCompFullPoseEstimation::FullPoseEuler &r_state);
    SpecificWorker::Estado doorS(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state);
};

#endif
