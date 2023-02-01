/*
 * Point.h
 *
 *  Created on: Jan 22, 2022
 *      Author: veins
 */

#pragma once

#include <cstddef>
#include <utility>
const size_t UNCLASSIFIED = 0;
const size_t CORE = 1;
const size_t BORDER = 2;
const size_t NOISE = 3;



class Point {
   public:
    std::pair<float, float> m_point;
    int m_clusterID;
    size_t m_type;

    Point(std::pair<float, float> point, int clusterID, size_t type);
    Point();

    size_t getM_type();
    std::pair<float, float> getM_point();
    int getM_clusterID();

    void setM_type(size_t);
    void setM_point(std::pair<float, float>);
    void setM_clusterID(int);
};






