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
#include <cppitertools/range.hpp>
#include <eigen3/Eigen/Eigen>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations_with_replacement.hpp>
#include <cppitertools/zip.hpp>
#include <cppitertools/zip_longest.hpp>
#include <grid2d/grid.h>
#include "dynamic_window.h"
#include "iou.h"
#include <random>
#include <numeric>

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
    void new_target_slot(QPointF p);

private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;
    AbstractGraphicViewer *viewer;

    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    QPointF last_point;

    const int ROBOT_LENGTH = 400;
    const int MAX_LASER_DIST = 4000;
    int TILE_SIZE = 200;

    void draw_laser(const RoboCompLaser:: TLaserData & ldata);

    struct Target {
        QPointF dest;
        bool activo = false;
        Eigen::Vector2f to_eigen() const {return Eigen::Vector2f(dest.x(), dest.y());}
    };
    Target target;

    RoboCompFullPoseEstimation::FullPoseEuler r_state;

    Grid grid;

    QLineF door;
    QPointF target_to_robot;

    enum class Estado{IDLE, INIT_TURN, EXPLORING, DOOR, NEXTROOM, CENTERROOM};
    Estado estado = Estado::IDLE;
    std::vector<int> VectorLinea;

    struct Constants
    {
        float ROBOT_LENGTH = 400;
        const float MAX_ADV_SPEED = 1000;
        const float TILE_SIZE = 200;
        const float MAX_LASER_DIST = 4000;
        float rot_speed = 0;
        float adv_speed = 0;
    };
    Constants constants;

    struct Door
    {
        Eigen::Vector2f dPoint1, dPoint2;
        std::set<int> rooms;
        bool used;

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
        std::optional<int> connecting_room(int inside_room) const {
            if (rooms.size() == 2 and rooms.find(inside_room) != rooms.end()) {
                auto r = std::ranges::find_if_not(rooms, [inside_room](auto a) { return a == inside_room; });
                return *r;
            } else
                return {};
        };
    };

    struct Room
            {
            IOU::Quad quad;
            int id;
            IOU::Vertexes points;
            const float diff = 300;
            bool operator == (const Room &d)
            {
                double iou = IOU::iou(quad, d.quad);
                return iou > 0.9;
            }
            Room(const IOU::Quad &quad_, int id_) : quad(quad_), id(id_)
            {
                quad.beInClockWise();
                quad.getVertList(points);
//            for(auto p:points)
//                qInfo() << p.x << p.y;
            };
            };
    std::vector<Room> rooms;

    Dynamic_Window dw;

    std::vector<Door> doors, selected_doors;

    Door *selectedDoor;
    Eigen::Vector2f way;

    float rotation_speed(float beta);

    Eigen::Vector2f world_to_robot(const Eigen::Vector2f &point);
    Eigen::Vector2f robot_to_world(const Eigen::Vector2f &point);

    void update_map(const RoboCompLaser::TLaserData &ldata);
};

#endif
