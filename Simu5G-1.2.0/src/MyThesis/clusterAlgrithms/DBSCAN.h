/*
 * DBSCAN.h
 *
 *  Created on: Jan 22, 2022
 *      Author: veins
 */

#pragma once
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <vector>


#include "MyThesis/MyClass/CarInfo.h"
#include <MyThesis/MyClass/MyVeinsAppRSU.h>
#include <MyThesis/clusterAlgrithms/Intra-cluster_Connectivity.h>


#include "Point.h"


using namespace veins;

const float EPS = 400.0;
const size_t MIN_POINTS = 5;

class DBSCAN {
   private:
    static void label(std::vector<CarInfo*> const &v);

    static void make_clusters(std::vector<CarInfo*> const &v);

    static std::vector<Point> MapToVectorPoint(map<int, CarInfo*>  &m);
    static std::vector<CarInfo*> MapToVector(map<int, CarInfo*> const &m);

   public:

    static std::vector<std::vector<CarInfo*>> RunDBscan();
};
