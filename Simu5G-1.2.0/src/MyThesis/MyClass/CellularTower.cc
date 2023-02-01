/*
 * CellularTower.cc
 *
 *  Created on: Feb 26, 2022
 *      Author: veins
 */




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

#include "CellularTower.h"

using namespace veins;

Define_Module(CellularTower);
map<int, CarInfo*> CarRecording1;
std::vector<std::vector<CarInfo*>> rankedClusters1;
int FirstRunRSU1;
bool initialClusters1 = true;




//initialize a RSU
void CellularTower::initialize(int stage){
    std::cout<<"Shajib:: CellularTower::initialize"<<endl;
    DemoBaseApplLayer::initialize(stage);
    runKmeans = par("runKmeans").boolValue();
    runDBscan = par("runDBscan").boolValue();
    findHost()->getDisplayString().setTagArg("r", 0, 400);
//    getDisplayString().setTagArg("r", 0, 700)



//    std::cout << findHost()->getDisplayString().getTagName(1) << endl;
//    std::cout << findHost()->getDisplayString().getTagName(2) << endl;
    if (stage == 0) {
        start_flooding = new cMessage("start_flooding");
        make_cluster = new cMessage("make_Cluster");
        scheduleAt(simTime()+4.9, make_cluster);
        scheduleAt(simTime()+1.6+uniform(0.05,0.1), start_flooding);
    }
}



//when RSU received a message from a car
void CellularTower::handleLowerMsg(cMessage* msg){
    std::cout<<"Shajib:: CellularTower::handleLowerMsg"<<endl;
    if(BeaconMsg* temp_bsm = dynamic_cast<BeaconMsg*>(msg)) {
        int source_id = temp_bsm->getSrcID();
        if(source_id!=0){
        int myid = myId;
        double velocity = temp_bsm->getVelocity();
        Coord myPosition = curPosition;
        Coord msgPosition = temp_bsm->getMessageOriginPosition();
        double drivingDistance = temp_bsm->getDrivingDistance();
        double airToExistPoint = temp_bsm->getAirToExistPoint();

        double dist = CalculateEuclideanDistance(myPosition, msgPosition);
        double direct = temp_bsm->getAngle();
        double nextDistance = EstimatedDistance(source_id, velocity, dist, myPosition, msgPosition, direct);

        double stayInRSUTime;
        double rssi = temp_bsm->getRssi();
        stayInRSUTime = airToExistPoint/velocity;

       // std::cout << drivingDistance << endl;
        updateClusterTable(source_id, myid, dist, drivingDistance, velocity, nextDistance, stayInRSUTime, rssi, msgPosition);
        }
   }
}

//periods send message by itself
void CellularTower::handleSelfMsg(cMessage* msg){
    std::cout<<"Shajib:: CellularTower::handleSelfMsg"<<endl;
    if(msg == start_flooding) {
        std::cout << "================================================" << endl;
        BeaconMsg* bsm = new BeaconMsg("Beacon");
        bsm->setRsuID(myId);
        bsm->setMessageOriginPosition(curPosition);
        bsm->setFromRSU(true);
        sendDown(bsm);
        populateWSM(bsm);
        scheduleAt(simTime()+1, start_flooding);
    }else if(msg == make_cluster){
        addingNeighbours(CarRecording1);

        map<int, CarInfo*> copyCarCluster;
        for(const auto &key: CarRecording1) {
            copyCarCluster.insert(std::make_pair(key.first, key.second));
        }
        for(const auto &key: copyCarCluster) {
            simtime_t time = simTime() - key.second->getTimestamp();
            if(time > 5){
                CarRecording1.erase(key.second->getCarID());
            }
        }
        scheduleAt(simTime()+1, make_cluster);
    }
}

void CellularTower::addingNeighbours(map<int, CarInfo*> carTable){


    map<int, CarInfo*> copyCarTable;
    for(const auto &key: carTable) {
        copyCarTable.insert(std::make_pair(key.first, key.second));
    }

    for(const auto &key: carTable) {
        std::vector<CarInfo*> temp;
        Coord a = key.second->getVehiclePosition();
        for(const auto &key1: carTable) {
            Coord b = key1.second->getVehiclePosition();
            double dist = Utility::eucd(a,b);
            if(dist < 400 && key1.second->getCarID() != key.second->getCarID()){
                temp.push_back(key1.second);
            }
        }
        key.second->setNeighbours(temp);
    }
}

//get distance between RSU and a car
double CellularTower::CalculateEuclideanDistance(Coord &a,Coord &b){
    double x = b.x - a.x; //calculating number to square in next step
    double y = b.y - a.y;
    double dist;
    dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
    dist = sqrt(dist);
    return dist;
}

//update information of the cluster table
void CellularTower::updateClusterTable(int source_id, int myid, double dist, double drivingDistance, double velocity, double nextDistance, double stayInRSUTime, double rssi, Coord msgPosition){
    bool createCar = true;
    for(const auto &key: CarRecording1) {
        if(source_id == key.second->getCarID()){
            createCar = false;
            if(myid == key.second->getRSUID()){//update
                key.second->setEuclidean_Distance(dist);
                key.second->setTimestamp(simTime());
                key.second->setCounter(key.second->getCounter()+1);
                key.second->setVelocity(velocity);
                key.second->setNextDistance(nextDistance);
                key.second->setDrivingDistance(drivingDistance);
                key.second->setStayInRSUTime(stayInRSUTime);
                key.second->setRSSI(rssi);
                key.second->setVehiclePosition(msgPosition);
            }else if(nextDistance < key.second->getNextDistance()){//replace
                key.second->setRSUID(myid);
                key.second->setEuclidean_Distance(dist);
                key.second->setTimestamp(simTime());
                key.second->setCounter(1);
                key.second->setVelocity(velocity);
                key.second->setNextDistance(nextDistance);
                key.second->setDrivingDistance(drivingDistance);
                key.second->setStayInRSUTime(stayInRSUTime);
                key.second->setRSSI(rssi);
                key.second->setVehiclePosition(msgPosition);
            }
        }
    }

    if(createCar){//create
        CarInfo *c = new CarInfo();
        c->setCarID(source_id);
        c->setRSUID(myid);
        c->setEuclidean_Distance(dist);
        c->setTimestamp(simTime());
        c->setCounter(1);
        c->setDrivingDistance(drivingDistance);
        c->setStayInRSUTime(stayInRSUTime);
        c->setRSSI(rssi);
        c->setVehiclePosition(msgPosition);
        CarRecording1.insert(std::make_pair(source_id, c));
    }
}



//Estimate next distance by linear regression... improve:how long the car coverd in rsu
double CellularTower::EstimatedDistance(int source_id, double velocity, double dist, Coord &myPosition,Coord &msgPosition, double direct){
    //calculate angular deviation
    double angle = CalculateAngle(myPosition, msgPosition);
    double angular_deviation = abs(direct-angle);

    if(angular_deviation>180){
        angular_deviation = 180-(angular_deviation - 180);
    }

    double directionVelocity = ((angular_deviation-90)/90)*velocity;

    double nextDistance;
    for(const auto &key: CarRecording1) {
        if(source_id == key.second->getCarID()){
            nextDistance = dist+directionVelocity;
        }
    }
    return nextDistance;
}

//calculate a relative angle between object a and object b.
double CellularTower::CalculateAngle(Coord &a,Coord &b){
    double x = b.x - a.x; //calculating number to square in next step
    double y = b.y - a.y;
    double dist;
    dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
    dist = sqrt(dist);
    double cos = x/dist;
    double radian = acos(cos);
    double angle = 180/(M_PI/radian);
    if(y<0){
        angle = 180+angle;
    }else if((y == 0) && (x<0)){
        angle = 180;
    }else if(y>0){
        angle = 180 - angle;
    }
    return angle;
}

//print cluster table information
void CellularTower::printMaps(map<int, CarInfo*> const &m) {
    std::cout << "--------------------------------------" << endl;
    if(!CarRecording1.empty()){
        std::cout << "I am an RSU, PrintMaps for clusterTable (If does not discover the Car more than 5s, delete the car from CLusterTable)" << endl;
    }


    map<int, CarInfo*> copyCarCluster;
    for(const auto &key: CarRecording1) {
        copyCarCluster.insert(std::make_pair(key.first, key.second));
    }
    for(const auto &key: copyCarCluster) {
        simtime_t time = simTime() - key.second->getTimestamp();
        if(time > 5){
                CarRecording1.erase(key.second->getCarID());
        }
    }


    //prepare the clusterTable
    for(const auto &key: CarRecording1) {
        if(key.second->getRSUID() == myId){
            CarInfo *carInfo = new CarInfo();
            carInfo->setCarID(key.second->getCarID());
            carInfo->setRSUID(myId);
            carInfo->setEuclidean_Distance(key.second->getEuclidean_Distance());
            carInfo->setTimestamp(key.second->getTimestamp());
            carInfo->setCounter(key.second->getCounter());
            carInfo->setVelocity(key.second->getVelocity());
            carInfo->setNextDistance(key.second->getNextDistance());
            carInfo->setDrivingDistance(key.second->getDrivingDistance());
            carInfo->setStayInRSUTime(key.second->getStayInRSUTime());
            carInfo->setRSSI(key.second->getRSSI());
            clusterTable.insert(std::make_pair(key.second->getCarID(), carInfo));
        }
    }

    copyCarCluster.clear();

    for(auto &key: m) {
        std::cout << "source Id is: " << key.first << " | carID: " << key.second->getCarID() << " | RSUID: " << key.second->getRSUID() << " | distance: " << key.second->getEuclidean_Distance() << " | timeStamp: " << key.second->getTimestamp() << " | Counter is: " << key.second->getCounter() << " | Velocity is: " << key.second->getVelocity() << " | Next distance is: " << key.second->getNextDistance() << " | Driving distance is: " << key.second->getDrivingDistance() << " | Stay In RSU Time is: " << key.second->getStayInRSUTime() << " | RSSI is: " << key.second->getRSSI() << endl;
    }
    clusterTable.clear();
}


map<int, CarInfo*> CellularTower::getVehicleState(){
    return CarRecording1;
}

void CellularTower::setVehicleState(map<int, CarInfo*> CR){
    CarRecording1 = CR;
}

void CellularTower::handlePositionUpdate(cObject* obj)
{
    //it is used to update the position of RSU
    DemoBaseApplLayer::handlePositionUpdate(obj);
}



void CellularTower::onBSM(BaseFrame1609_4* bsm){
    std::cout<<"Shajib:: CellularTower::onBSM"<<endl;
  // Your application has received a beacon message from another car or RSU
  // code for handling the message goes here
}

void CellularTower::onWSM(BaseFrame1609_4* wsm){
    std::cout<<"Shajib:: CellularTower::onWSM"<<endl;
    // Your application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
}

void CellularTower::onWSA(DemoServiceAdvertisment* wsa){
    std::cout<<"Shajib:: CellularTower::onWSA"<<endl;
    // Your application has received a service advertisement from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
}

