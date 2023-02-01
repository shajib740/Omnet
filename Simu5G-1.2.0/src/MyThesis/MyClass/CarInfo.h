/*
 * CarInfo.h
 *
 *  Created on: Dec 10, 2021
 *      Author: veins
 */

#ifndef MYTHESISCLASS_CARINFO_H_
#define MYTHESISCLASS_CARINFO_H_

#include <fstream>
#include <algorithm>
#include <iostream>
#include <stack>
#include <algorithm>
#include <map>
#include <vector>
#include <tuple>

#include "veins/veins.h"
#include <fstream>
#include <algorithm>


using namespace omnetpp;
using namespace std;

#include <iostream>
#include <unordered_map>
#include <cstdlib>

#include "veins/modules/messages/DemoServiceAdvertisement_m.h"
#include "MyThesis/clusterAlgrithms/Point.h"

using namespace std;
using namespace veins;

class CarInfo{
    veins::Coord VehiclePosition;
    int CarID;
    int RSUID;
    int SourceID;
    int Counter;
    simtime_t Timestamp;
    double Euclidean_Distance;
    double DrivingDistance;
    double velocity;
    double angular_deviation;
    double direction_velocity;
    double next_distance;
    double stayInRSUTime;
    double rssi;
    Point point;
    std::vector<CarInfo*> neighbours;
    int belong_cluster;//the cluster should be the center's car ID
    int hops;
    double connectivity;
    std::vector<std::vector<double>> delay;
    double delay1;
    double clusterConnectivity;


public:
    veins::Coord getVehiclePosition();
    int getCarID();
    int getRSUID();
    int getSourceID();
    int getCounter();
    simtime_t getTimestamp();
    double getEuclidean_Distance();
    double getDrivingDistance();
    double getStayInRSUTime();
    double getVelocity();
    double getAngularDeviation();
    double getDirectionVelocity();
    double getNextDistance();
    double getRSSI();
    Point getPoint();
    std::vector<CarInfo*> getNeighbours();
    int getBelongCluster();
    int getHops();
    double getConnectivity();
    std::vector<std::vector<double>> getDelay();
    double getdelay();
    double getClusterConnectivity();


    void setVehiclePosition(veins::Coord);
    void setCarID(int);
    void setRSUID(int);
    void setSourceID(int);
    void setCounter(int);
    void setTimestamp(simtime_t);
    void setEuclidean_Distance(double);
    void setDrivingDistance(double);
    void setStayInRSUTime(double);
    void setVelocity(double);
    void setAngularDeviation(double);
    void setDirectionVelocity(double);
    void setNextDistance(double);
    void setRSSI(double);
    void setPoint(Point);
    void setNeighbours(std::vector<CarInfo*>);
    void setBelongCluster(int);
    void setHops(int);
    void setConnectivity(double);
    void setDelay(std::vector<std::vector<double>>);
    void setdelay(double);
    void setClusterConnectivity(double);


};


#endif /* MYTHESISCLASS_CARINFO_H_ */
