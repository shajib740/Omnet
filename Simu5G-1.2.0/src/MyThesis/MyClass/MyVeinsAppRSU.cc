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


#include "MyVeinsAppRSU.h"

using namespace veins;
Define_Module(MyVeinsAppRSU);










MyVeinsAppCar *myCarx = new MyVeinsAppCar();
map<int, CarInfo*> CarRecording;
std::vector<std::vector<CarInfo*>> rankedClusters;
int FirstRunRSU;
bool initialClusters = true;
std::vector<std::vector<CarInfo*>> DBScanInital;
std::vector<std::vector<CarInfo*>> KmeansInital;






//initialize a RSU
void MyVeinsAppRSU::initialize(int stage){
    std::cout << "Shajib:: MyVeinsAppRSU::initialize" << endl;
    DemoBaseApplLayer::initialize(stage);
    runKmeans = par("runKmeans").boolValue();
    enableNodeB = par("enableNodeB").boolValue();
    numberRSU = par("numberRSU").intValue();
    runDBscan = par("runDBscan").boolValue();
    beaconInterval = par("beaconInterval").doubleValue();
    findHost()->getDisplayString().setTagArg("r", 0, 400);


    if (stage == 0) {
        start_flooding = new cMessage("start_flooding");
        make_cluster = new cMessage("make_Cluster");
        PingPongTest = new cMessage("PingPongTest");

        scheduleAt(simTime()+5.6+uniform(0.05,0.1), start_flooding);
//        scheduleAt(simTime()+5, make_cluster);
//        scheduleAt(simTime()+5.2+uniform(0.05,0.1), PingPongTest);
    }
}

void MyVeinsAppRSU::onBSM(BaseFrame1609_4* bsm){
    std::cout << "Shajib:: MyVeinsAppRSU::onBSM" << endl;
  // Your application has received a beacon message from another car or RSU
  // code for handling the message goes here
}

void MyVeinsAppRSU::onWSM(BaseFrame1609_4* wsm){
    std::cout << "Shajib:: MyVeinsAppRSU::onWSM" << endl;
    // Your application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
}

void MyVeinsAppRSU::onWSA(DemoServiceAdvertisment* wsa){
    std::cout << "Shajib:: MyVeinsAppRSU::onWSA" << endl;
    // Your application has received a service advertisement from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
}

//when RSU received a message from a car
void MyVeinsAppRSU::handleLowerMsg(cMessage* msg){
    std::cout << "Shajib:: MyVeinsAppRSU::handleLowerMsg" << endl;
    if(BeaconMsg* temp_bsm = dynamic_cast<BeaconMsg*>(msg)) {
        if(!temp_bsm->getAckMsg()){
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
}

//periods send message by itself
void MyVeinsAppRSU::handleSelfMsg(cMessage* msg){
    std::cout << "Shajib:: MyVeinsAppRSU::handleSelfMsg" << endl;
    if(msg == start_flooding) {
        std::cout << "================================================" << endl;
        BeaconMsg* bsm = new BeaconMsg("Beacon");
        bsm->setRsuID(myId);
        bsm->setMessageOriginPosition(curPosition);
        bsm->setFromRSU(true);
        sendDown(bsm);
        populateWSM(bsm);

        scheduleAt(simTime()+1, make_cluster);
        scheduleAt(simTime()+1.5+uniform(0.05,0.1), PingPongTest);

        scheduleAt(simTime()+1+beaconInterval, start_flooding);


    }else if(msg == make_cluster){
        /* std::vector<CarInfo*> AllCars = myCarx->getAllCars();
        map<int, CarInfo*> AllCars1 = VectorToMap(AllCars);

      addingNeighbours(CarRecording);
        addingNeighbours(AllCars1);

        map<int, CarInfo*> copyCarCluster;
        for(const auto &key: CarRecording) {
            copyCarCluster.insert(std::make_pair(key.first, key.second));
        }
        for(const auto &key: copyCarCluster) {
            simtime_t time = simTime() - key.second->getTimestamp();
            if(time > 5){
                CarRecording.erase(key.second->getCarID());
            }
        }

        //print cluster table
        if(initialClusters && FirstRunRSU%numberRSU == 0){
            if(runKmeans){
                DBScanInital = DBSCAN::RunDBscan();
                KmeansInital = RunKmeans();
                initialClusters = false;
            }else if(runDBscan){
                DBSCAN::RunDBscan();
                initialClusters = false;
            }else{
                printMaps(clusterTable);
            }
        }else if(FirstRunRSU%numberRSU == 0){
            RunUpdateCluster(enableNodeB);
            RunAnalysisCluster(KmeansInital, DBScanInital,numberRSU,enableNodeB,beaconInterval);
        }
        FirstRunRSU += 1;
        cancelEvent(make_cluster);*/
//        scheduleAt(simTime()+beaconInterval, make_cluster);
    }else if(msg == PingPongTest){
        BeaconMsg* bsm = new BeaconMsg("Beacon");
        bsm->setAckMsg(true);
        bsm->setRsuID(myId);
        bsm->setSrcID(myId);
        bsm->setHop(1);
        bsm->setTimestamp(simTime());
        sendDown(bsm);
        populateWSM(bsm);
//        scheduleAt(simTime()+beaconInterval, PingPongTest);
        cancelEvent(PingPongTest);
    }
}

void MyVeinsAppRSU::addingNeighbours(map<int, CarInfo*> carTable){

    std::cout << "Shajib:: MyVeinsAppRSU::addingNeighbours" << endl;
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
double MyVeinsAppRSU::CalculateEuclideanDistance(Coord &a,Coord &b){
    double x = b.x - a.x; //calculating number to square in next step
    double y = b.y - a.y;
    double dist;
    dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
    dist = sqrt(dist);
    return dist;
}

//update information of the cluster table
void MyVeinsAppRSU::updateClusterTable(int source_id, int myid, double dist, double drivingDistance, double velocity, double nextDistance, double stayInRSUTime, double rssi, Coord msgPosition){
    bool createCar = true;
    for(const auto &key: CarRecording) {
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
        CarRecording.insert(std::make_pair(source_id, c));
    }
}



//Estimate next distance by linear regression... improve:how long the car coverd in rsu
double MyVeinsAppRSU::EstimatedDistance(int source_id, double velocity, double dist, Coord &myPosition,Coord &msgPosition, double direct){
    //calculate angular deviation
    double angle = CalculateAngle(myPosition, msgPosition);
    double angular_deviation = abs(direct-angle);

    if(angular_deviation>180){
        angular_deviation = 180-(angular_deviation - 180);
    }

    double directionVelocity = ((angular_deviation-90)/90)*velocity;

    double nextDistance;
    for(const auto &key: CarRecording) {
        if(source_id == key.second->getCarID()){
            nextDistance = dist+directionVelocity;
        }
    }
    return nextDistance;
}

//calculate a relative angle between object a and object b.
double MyVeinsAppRSU::CalculateAngle(Coord &a,Coord &b){
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
void MyVeinsAppRSU::printMaps(map<int, CarInfo*> const &m) {
    std::cout << "--------------------------------------" << endl;
    if(!CarRecording.empty()){
        std::cout << "I am an RSU, PrintMaps for clusterTable (If does not discover the Car more than 5s, delete the car from CLusterTable)" << endl;
    }


    map<int, CarInfo*> copyCarCluster;
    for(const auto &key: CarRecording) {
        copyCarCluster.insert(std::make_pair(key.first, key.second));
    }
    for(const auto &key: copyCarCluster) {
        simtime_t time = simTime() - key.second->getTimestamp();
        if(time > 5){
                CarRecording.erase(key.second->getCarID());
        }
    }


    //prepare the clusterTable
    for(const auto &key: CarRecording) {
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

void MyVeinsAppRSU::printVectors(std::vector<CarInfo*> const &v) {
    std::cout << "-------------print vector------------" << endl;

    for(int i=0; i<v.size(); i++){
                std::cout << " | carID: " << v[i]->getCarID() << " | RSUID: " << v[i]->getRSUID() << " | distance: " << v[i]->getEuclidean_Distance() << " | timeStamp: " << v[i]->getTimestamp() << " | Counter is: " << v[i]->getCounter() << " | Velocity is: " << v[i]->getVelocity() << " | Next distance is: " << v[i]->getNextDistance() << " | Driving distance is: " << v[i]->getDrivingDistance() << " | Stay In RSU Time is: " << v[i]->getStayInRSUTime() << " | RSSI is: " << v[i]->getRSSI() << endl;
    }
}

void MyVeinsAppRSU::printDoubleVectors(const std::vector<std::vector<CarInfo*>> &cluster) {
    std::cout << "-------------print nested vector------------" << endl;

    for(int i=0; i<cluster.size(); i++){
        std::cout << "-------I am currently in cluster: "  << i << "--------" << endl;
        if(!cluster[i].empty()){
            for(int j=0; j<cluster[i].size(); j++){
                std::cout << " | carID: " << cluster[i][j]->getCarID() << " | RSUID: " << cluster[i][j]->getRSUID() << " | distance: " << cluster[i][j]->getEuclidean_Distance() << " | timeStamp: " << cluster[i][j]->getTimestamp() << " | Counter is: " << cluster[i][j]->getCounter() << " | Velocity is: " << cluster[i][j]->getVelocity() << " | Next distance is: " << cluster[i][j]->getNextDistance() << " | Driving distance is: " << cluster[i][j]->getDrivingDistance() << " | Stay In RSU Time is: " << cluster[i][j]->getStayInRSUTime() << " | RSSI is: " << cluster[i][j]->getRSSI() << " | ClusterConnectivity is: " << cluster[i][j]->getClusterConnectivity() << endl;
            }
        }
    }
}


map<int, CarInfo*> MyVeinsAppRSU::getVehicleState(){
    return CarRecording;
}

void MyVeinsAppRSU::setVehicleState(map<int, CarInfo*> CR){
    CarRecording = CR;
}


std::vector<std::vector<CarInfo*>> MyVeinsAppRSU::getClusterState(){
    return rankedClusters;
}

void MyVeinsAppRSU::setClusterState(std::vector<std::vector<CarInfo*>> CS){
    rankedClusters = CS;
}

void MyVeinsAppRSU::handlePositionUpdate(cObject* obj)
{
    //it is used to update the position of RSU
    DemoBaseApplLayer::handlePositionUpdate(obj);
}
