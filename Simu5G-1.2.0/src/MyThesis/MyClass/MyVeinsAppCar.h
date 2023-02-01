//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#pragma once

#include "veins/veins.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include <iostream>


#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/messages/Mac80211Pkt_m.h"
#include "veins/modules/messages/DemoServiceAdvertisement_m.h"
#include "veins/modules/messages/Ack_m.h"
#include "veins/modules/messages/DataMsg_m.h"
#include "veins/modules/phy/DeciderResult80211.h"
#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "veins/modules/messages/BeaconMsg_m.h"
#include <MyThesis/MyClass/CarInfo.h>
#include <MyThesis/MyClass/MyVeinsAppRSU.h>
#include <MyThesis/MyClass/CellularTower.h>
#include <MyThesis/clusterAlgrithms/Intra-cluster_Connectivity.h>
using namespace omnetpp;
using namespace std;



/**
 * @brief
 * This is a stub for a typical Veins application layer.
 * Most common functions are overloaded.
 * See MyVeinsApp.cc for hints
 *
 * @author David Eckhoff
 *
 */

class VEINS_API MyVeinsAppCar : public veins::DemoBaseApplLayer {
private:
    cMessage* node_start_flooding;
    cMessage* node_stop_flooding;
    cMessage* node_ack_msg;
    cMessage* node_finishing;


    double velocity;
    map<double, double> velocitySet;
    double direct;
    map<double, double> directSet;
    double DrivingDistance;
    double airToExitPoint;
    double rssi;

    double beaconInterval;
    veins::Coord historyPosition;
//    vector<int> neighbours;
//    void printMaps(map<int, vector<double>> const &m);
    double CalculateEuclideanDistance(veins::Coord &a,veins::Coord &b);
    double CalculateAngle(veins::Coord &a,veins::Coord &b);
    double LinearRegression(map<double, double> const &data);
    double AirToExitPoint(veins::Coord &a,veins::Coord &b);

    void UpdateVehiclesTable(double sourceID, double dist, veins::Coord &msgPosition, double delay);
    void UpdateDelayTable(double rsuId, double sourceID, double hops, double delay);



public:
    void initialize(int stage);
    std::vector<int> car;
    std::vector<CarInfo*> getAllCars();
    std::vector<CarInfo*> getDelayCars();

    void clearDelayTable();


protected:
    void onBSM(veins::BaseFrame1609_4* bsm);
    void onWSM(veins::BaseFrame1609_4* wsm);
    void onWSA(veins::DemoServiceAdvertisment* wsa);

    virtual void handleSelfMsg(cMessage* msg);
    virtual void handlePositionUpdate(cObject* obj);
    virtual void finish();


};


