/*
 * Point.cc
 *
 *  Created on: Jan 22, 2022
 *      Author: veins
 */




#include "Point.h"

Point::Point(std::pair<float, float> point, int clusterID, size_t type) : m_point(point), m_clusterID(clusterID), m_type(type){}
Point::Point() : Point(std::make_pair(-1, -1), -1, 0) {}



std::pair<float, float> Point::getM_point(){
    return m_point;
}
int Point::getM_clusterID(){
    return m_clusterID;
}

size_t Point::getM_type(){
    return m_type;
}
