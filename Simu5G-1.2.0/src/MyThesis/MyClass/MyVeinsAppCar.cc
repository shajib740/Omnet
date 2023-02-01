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


#include "MyVeinsAppCar.h"
#include <math.h>

Define_Module(MyVeinsAppCar);
using namespace veins;


#include <string>
#include <sstream>
#include <algorithm>
#include <unordered_set>
#include <iostream>
#include "MyThesis/clusterAlgrithms/kmeans.h"
MyVeinsAppRSU *myRSU2 = new MyVeinsAppRSU();


std::vector<CarInfo*> AllVehicles;
std::vector<CarInfo*> DelayTable;
int counthop3 = 0;

//initialize a vehicle
void MyVeinsAppCar::initialize(int stage){
    std::cout<<"Shaib:: MyVeinsAppCar::initialize"<<endl;
    DemoBaseApplLayer::initialize(stage);
    if(stage == 0){
        node_start_flooding = new cMessage("Node_Start_Flooding");
        node_stop_flooding = new cMessage("Node_Stop_Flooding");
        node_ack_msg = new cMessage("Node_Ack_Msg");
        node_finishing = new cMessage("Node_FINISH");
        beaconInterval = par("beaconInterval").doubleValue();

        //open this code => car will send beacon.
        scheduleAt(simTime()+1+uniform(0.05,0.2), node_start_flooding);
    }
}



void MyVeinsAppCar::onBSM(BaseFrame1609_4* bsm){
    std::cout<<"Shaib:: MyVeinsAppCar::onBSM"<<endl;
 //my Car received a message from a RSU or a Car, (if from RSU, sending a Ack message), (if from
 //a Car's Ack message, ignore it), if from a Car's beacon message, add info and re-sending this Beacon message)
    std::vector<std::vector<CarInfo*>> cluster = myRSU2->getClusterState();


    if(BeaconMsg* temp_bsm = dynamic_cast<BeaconMsg*>(bsm)) {
        if(temp_bsm->getFromRSU()){
            Coord RsuPosition = temp_bsm->getMessageOriginPosition();
            Coord myPosition = curPosition;
            DrivingDistance = traci->getDistance(myPosition,RsuPosition,true);
            airToExitPoint = AirToExitPoint(myPosition,RsuPosition);
            rssi = check_and_cast<DeciderResult80211*>(check_and_cast<PhyToMacControlInfo*>(bsm -> getControlInfo()) -> getDeciderResult()) -> getRecvPower_dBm();

//            std::cout << "Driving distance between vehicle " << myId << " and RSU " << rsu_id << " is: " << DrivingDistance <<endl;
//

            //string r = TraCIMobilityAccess().get(getParentModule())->getRoadId();
            //a = traci->lane(r).getLength();
        }else if(temp_bsm->getAckMsg()){

            bool forward = true;


            int rsuId = temp_bsm->getRsuID();
            int SrcId = temp_bsm->getSrcID();
            int hops = temp_bsm->getHop();
            double time = temp_bsm->getTimestamp().dbl();
            double delay = simTime().dbl() - time;


            for(int i=0; i<DelayTable.size(); i++){
                if(DelayTable[i]->getCarID() == myId){
                    forward = false;
                }
            }


            //decide when to senddonw (no duplicate)
            if(forward){
                UpdateDelayTable(rsuId, SrcId, hops, delay);
                if(hops == 1){
                    BeaconMsg* bsm = new BeaconMsg("Beacon");
                    bsm->setAckMsg(true);
                    bsm->setRsuID(rsuId);
                    bsm->setSrcID(myId);
                    bsm->setHop(2);
                    bsm->setTimestamp(temp_bsm->getTimestamp());
                    sendDelayedDown(bsm,uniform(0.001,0.05));
                    populateWSM(bsm);
                }

                if(hops == 2){
                    BeaconMsg* bsm = new BeaconMsg("Beacon");
                    bsm->setAckMsg(true);
                    bsm->setRsuID(rsuId);
                    bsm->setSrcID(myId);
                    bsm->setHop(3);
                    bsm->setTimestamp(temp_bsm->getTimestamp());
                    sendDelayedDown(bsm,uniform(0.0000000001,0.001));
                    populateWSM(bsm);
                }

                if(hops == 3){

                }
            }



        }else{
            double source = temp_bsm->getSrcID();
            Coord a = temp_bsm->getMessageOriginPosition();
            Coord b = curPosition;
            double dist = CalculateEuclideanDistance(a,b);
            Coord msgPostion = temp_bsm->getMessageOriginPosition();
            double time1 = temp_bsm->getDelay();
            UpdateVehiclesTable(source, dist, msgPostion, time1);
        }
    }
}

void MyVeinsAppCar::UpdateDelayTable(double rsuId, double SrcId, double hops, double delay){
    CarInfo* temp = new CarInfo();
    temp->setRSUID(rsuId);
    temp->setCarID(myId);
    temp->setSourceID(SrcId);
    temp->setHops(hops);
    temp->setdelay(delay);
    DelayTable.push_back(temp);
}


void MyVeinsAppCar::UpdateVehiclesTable(double sourceID, double dist, veins::Coord &msgPosition, double time1){
    bool createCar = true;
    double time2 = simTime().dbl();
    double delay = time2 - time1;

    for(int i=0; i<AllVehicles.size(); i++){
        if(sourceID == AllVehicles[i]->getCarID()){
            createCar = false;
            AllVehicles[i]->setEuclidean_Distance(dist);
            AllVehicles[i]->setTimestamp(simTime());
            AllVehicles[i]->setRSSI(rssi);
            AllVehicles[i]->setVehiclePosition(msgPosition);


            std::vector<std::vector<double>> delayInfo = AllVehicles[i]->getDelay();
            bool addNew = true;

            for(int j=0; j<delayInfo.size(); j++){//find if the element is new or not
                if(delayInfo[j][0] == sourceID && delayInfo[j][1] ==myId){
                    addNew = false;
                    delayInfo[j][2] = delay;
                }
            }

            if(addNew){//if this delay info not int the carInfo yet, create new and add it
                delayInfo = AllVehicles[i]->getDelay();
                std::vector<double> temp;
                temp.push_back(sourceID);
                temp.push_back(myId);
                temp.push_back(time1);
                temp.push_back(delay);
                delayInfo.push_back(temp);

            }


            for(int j=0; j<delayInfo.size(); j++){//check and delete
                if((time2 - delayInfo[j][2]) > 5){
                    delayInfo.erase(delayInfo.begin()+j);
                    j = j-1;
                }
            }

            AllVehicles[i]->setDelay(delayInfo);
        }
    }

    if(createCar){//create
        CarInfo *c = new CarInfo();
        c->setCarID(sourceID);
        c->setRSUID(-1);
        c->setEuclidean_Distance(dist);
        c->setTimestamp(simTime());
        c->setRSSI(rssi);
        c->setVehiclePosition(msgPosition);

        std::vector<std::vector<double>> delayInfo;
        std::vector<double> temp;
        temp.push_back(sourceID);
        temp.push_back(myId);
        temp.push_back(time1);
        temp.push_back(delay);
        delayInfo.push_back(temp);
        c->setDelay(delayInfo);

        AllVehicles.push_back(c);
    }
}



void MyVeinsAppCar::handleSelfMsg(cMessage* msg)
{
    std::cout<<"Shaib:: MyVeinsAppCar::handleSelfMsg"<<endl;
    if(msg == node_start_flooding) {
       BeaconMsg* bsm = new BeaconMsg("Beacon");
       bsm->setHop(1);
       bsm->setSrcID(myId);
       bsm->setMessageOriginPosition(curPosition);
       bsm->setIsFlooding(true);
       velocity = CalculateEuclideanDistance(historyPosition, curPosition)/beaconInterval;
       bsm->setVelocity(velocity);
       direct = CalculateAngle(curPosition,historyPosition);
       bsm->setAngle(direct);
       bsm->setDrivingDistance(DrivingDistance);

       bsm->setAirToExistPoint(airToExitPoint);
       bsm->setRssi(rssi);

       bsm->setTimestamp(simTime());



       double time = simTime().dbl();
       if(time > 3){
           velocitySet.insert(std::make_pair(time, velocity));
           directSet.insert(std::make_pair(time, direct));
       }
       if(velocitySet.size()>10){
          velocity = LinearRegression(velocitySet);
       }


       historyPosition = curPosition;
       //make a velocity, send it to rsu()

       sendDown(bsm);
       populateWSM(bsm);
       scheduleAt(simTime()+beaconInterval, node_start_flooding);
    }
}


//Estimate the time that vehicles will stay how long time in this RSU
double MyVeinsAppCar::AirToExitPoint(Coord &a,Coord &b){
    std::cout<<"Shaib:: MyVeinsAppCar::AirToExitPoint"<<endl;
//    double estimatedStayTime;
//    double timeBasedOnRealRoadDistance = 0;
//    double timeBasedOnAirDistance = 0;
    Coord exitPoint;
    //Using Trigonometric functions to calculate the exist point,need start point, degree, and distance

    double distance = sqrt(pow(CalculateEuclideanDistance(a,b), 2) + pow(250,2));
    exitPoint.x = a.x+sin(direct)*distance;
    exitPoint.y = a.y+cos(direct)*distance;

    if((exitPoint.x-b.x)>250) exitPoint.x = b.x+250;
    if((b.x-exitPoint.x)>250) exitPoint.x = b.x-250;
    if((exitPoint.y-b.y)>250) exitPoint.y = b.y+250;
    if((b.y-exitPoint.y)>250) exitPoint.y = b.y-250;



//    double drivingToExistPoint = traci->getDistance(curPosition,existPoint,true);

     double dist = CalculateEuclideanDistance(curPosition,exitPoint);
//     double drivingToExistPoint = traci->getDistance(curPosition,existPoint,true)/velocity;
//
//     if(drivingToExitPoint>1000){
//         drivingToExistPoint = -1;
//     }
//     if(drivingToExitPoint<0){
//         drivingToExitPoint = -1;
//     }
//     std::cout << "my Car Id IS: " << myId << drivingToExitPoint << endl;


//    std::cout << myId << endl;
//    std::cout << "start point " << a.x << "|" << a.y << endl;
//    std::cout << "exist point " << existPoint.x << "|" << exitPoint.y << endl;
//    std::cout << airToExitPoint << endl;

    return dist;
}

//y = a+bx
//map<time, distance>
double MyVeinsAppCar::LinearRegression(map<double, double> const &data){
    double xBar;
    double yBar;
    double a;
    double b;
    double result;

    double sumX = 0;
    double sumY = 0;
    for(const auto &key: data) {
        sumX+=key.first;
        sumY+=key.second;
    }

    xBar = sumX/data.size();
    yBar = sumY/data.size();

    double bUp = 0;
    double bDown = 0;
    for(const auto &key: data) {
        bUp += (key.first - xBar)*(key.second - yBar);
        bDown += pow((key.first - xBar),2);
    }
    bUp = bUp/data.size();
    bDown = bDown/data.size();
    b = bUp/bDown;
    a = yBar - b*xBar;

    double x = simTime().dbl();
    result = a+b*(x+1);
    return result;
}



double MyVeinsAppCar::CalculateEuclideanDistance(Coord &a,Coord &b){
    double x = b.x - a.x; //calculating number to square in next step
    double y = b.y - a.y;
    double dist;
    dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
    dist = sqrt(dist);
    return dist;
}

double MyVeinsAppCar::CalculateAngle(Coord &a,Coord &b){
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




void MyVeinsAppCar::onWSM(BaseFrame1609_4* wsm){
    std::cout<<"Shaib:: MyVeinsAppCar::onWSM"<<endl;
}

void MyVeinsAppCar::onWSA(DemoServiceAdvertisment* wsa)
{
    std::cout<<"Shaib:: MyVeinsAppCar::onWSA"<<endl;
    // Your application has received a service advertisement from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
}


void MyVeinsAppCar::handlePositionUpdate(cObject* obj)
{
    std::cout<<"Shaib:: MyVeinsAppCar::handlePositionUpdate"<<endl;
    std::vector<const char*> color;
    color.push_back("green");
    color.push_back("red");
    color.push_back("cYan");
    color.push_back("yellow");
    color.push_back("blue");
    color.push_back("orange");
    color.push_back("violet");
    color.push_back("gold");
    color.push_back("indigo");
    color.push_back("pink");
    color.push_back("purple");
    color.push_back("tan");
    color.push_back("brown");
    color.push_back("azure");
    color.push_back("silver");




    findHost()->getDisplayString().setTagArg("i", 1, "white");


    std::vector<std::vector<CarInfo*>> clusters = myRSU2->getClusterState();

    std::vector<CarInfo*> vehicles = Utility::getAllVehilcesTable();

//    std::vector<CarInfo*> vehicles2 = Utility::MultiHop(vehicles, true);

    for(int i=0; i<vehicles.size(); i++){
        if(myId == vehicles[i]->getCarID()){
            findHost()->getDisplayString().setTagArg("i", 1, "black");
        }
    }



    for(int i=0; i<clusters.size(); i++){
        for(int j=0; j<clusters[i].size(); j++){
            if(myId == clusters[i][j]->getCarID()){
                if(i<15){
                    findHost()->getDisplayString().setTagArg("i", 1, color[i]);
                }else{
                    findHost()->getDisplayString().setTagArg("i", 1, "black");
                }
            }
        }
    }


    DemoBaseApplLayer::handlePositionUpdate(obj);
    // the vehicle has moved. Code that reacts to new positions goes here.
    // member variables such as currentPosition and currentSpeed are updated in the parent class
}

std::vector<CarInfo*> MyVeinsAppCar::getAllCars(){
    return AllVehicles;
}


std::vector<CarInfo*> MyVeinsAppCar::getDelayCars(){
    return DelayTable;
}

void MyVeinsAppCar::clearDelayTable(){
    DelayTable.clear();
}

void MyVeinsAppCar::finish()
{
    DemoBaseApplLayer::finish();
    // statistics recording goes here
}





