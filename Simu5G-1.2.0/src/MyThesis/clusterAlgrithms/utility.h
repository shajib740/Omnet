/*
 * utility.h
 *
 *  Created on: Jan 22, 2022
 *      Author: veins
 */

#pragma once

#include <fstream>
#include <map>
#include <vector>
#include "veins/modules/messages/DemoServiceAdvertisement_m.h"
#include <MyThesis/MyClass/CarInfo.h>
#include <MyThesis/MyClass/MyVeinsAppRSU.h>
#include <MyThesis/MyClass/CellularTower.h>
using namespace veins;



class Utility {
   public:
        static double eucd(Coord &a,Coord &b);
        static double eucd(const std::pair<double,double> &p1, const std::pair<double,double> &p2);
        static int min_index(std::vector<double> vec);
        static std::vector<CarInfo*> getAllVehilcesTable(); //merge two cluster table (RSU and cellularTower)
        static std::vector<double> SetRelConnectivity(std::vector<CarInfo*> const &v); //set Relative connectivity for each vehicle and return their connectivity



        static void printVectors(std::vector<CarInfo*> const &v);
        static void printDoubleVectors(const std::vector<std::vector<CarInfo*>> &cluster);

};
