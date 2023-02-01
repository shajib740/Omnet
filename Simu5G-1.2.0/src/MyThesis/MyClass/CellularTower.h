/*
 * CellularTower.h
 *
 *  Created on: Feb 26, 2022
 *      Author: veins
 */


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

using namespace omnetpp;
using namespace std;



#include <iostream>
#include <unordered_map>
#include <cstdlib>





class VEINS_API CellularTower : public veins::DemoBaseApplLayer {

private:
    cMessage* make_cluster;
    cMessage* start_flooding;



protected:
    void onBSM(veins::BaseFrame1609_4* bsm) override;
    void onWSM(veins::BaseFrame1609_4* wsm) override;
    void onWSA(veins::DemoServiceAdvertisment* wsa) override;

    void handleLowerMsg(cMessage* msg) override;
    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
    double CalculateAngle(veins::Coord &a,veins::Coord &b);

    bool runKmeans;
    bool runDBscan;


public:
    map<int, CarInfo*> clusterTable;

    map<int, CarInfo*> getVehicleState();
    void setVehicleState(map<int, CarInfo*>);

    void initialize(int stage) override;
    double CalculateEuclideanDistance(veins::Coord &a,veins::Coord &b);
    double EstimatedDistance(int source_id, double velocity, double dist, veins::Coord &a,veins::Coord &b, double direct);
    void updateClusterTable(int source_id, int myid, double dist, double drivingDistance, double velocity, double nextDistance, double stayInRSUTime, double rssi, veins::Coord msgPosition);

    void printMaps(map<int, CarInfo*> const &m);
    void addingNeighbours(map<int, CarInfo*> carTable);
}; // namespace veins

