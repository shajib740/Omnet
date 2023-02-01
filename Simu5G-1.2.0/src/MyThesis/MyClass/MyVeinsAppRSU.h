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
#include <fstream>
#include <algorithm>
#include <iostream>
#include <map>
#include <string.h>
#include <iterator>
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/messages/Ack_m.h"
#include "veins/modules/messages/BeaconMsg_m.h"
#include "veins/modules/messages/DataMsg_m.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/messages/DemoServiceAdvertisement_m.h"


#include <stdio.h>
#include <sstream>
#include <regex>
#include <vector>
#include <string>
#include <MyThesis/MyClass/CarInfo.h>
#include <MyThesis/clusterAlgrithms/kmeans.h>
#include <MyThesis/clusterAlgrithms/DBSCAN.h>
#include <MyThesis/clusterAlgrithms/utility.h>
#include <MyThesis/clusterAlgrithms/updateCluster.h>
#include <MyThesis/clusterAlgrithms/clusterAnalysis.h>
#include <fstream>

using namespace omnetpp;
using namespace std;



#include <iostream>
#include <unordered_map>
#include <cstdlib>





class VEINS_API MyVeinsAppRSU : public veins::DemoBaseApplLayer {

private:
    cMessage* make_cluster;
    cMessage* start_flooding;
    cMessage* PingPongTest;



protected:
    void onBSM(veins::BaseFrame1609_4* bsm) override;
    void onWSM(veins::BaseFrame1609_4* wsm) override;
    void onWSA(veins::DemoServiceAdvertisment* wsa) override;

    void handleLowerMsg(cMessage* msg) override;
    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
    double CalculateAngle(veins::Coord &a,veins::Coord &b);

    int numberRSU;
    bool runKmeans;
    bool enableNodeB;
    bool runDBscan;
    double beaconInterval;


public:
    map<int, CarInfo*> clusterTable;

    map<int, CarInfo*> getVehicleState();
    void setVehicleState(map<int, CarInfo*>);
    std::vector<std::vector<CarInfo*>> getClusterState();
    void setClusterState(std::vector<std::vector<CarInfo*>>);


    void initialize(int stage) override;
    double CalculateEuclideanDistance(veins::Coord &a,veins::Coord &b);
    double EstimatedDistance(int source_id, double velocity, double dist, veins::Coord &a,veins::Coord &b, double direct);
    void updateClusterTable(int source_id, int myid, double dist, double drivingDistance, double velocity, double nextDistance, double stayInRSUTime, double rssi, veins::Coord msgPosition);
    void printMaps(map<int, CarInfo*> const &m);
    void printVectors(std::vector<CarInfo*> const &v);
    void printDoubleVectors(const std::vector<std::vector<CarInfo*>> &cluster);
    void addingNeighbours(map<int, CarInfo*> carTable);
}; // namespace veins






