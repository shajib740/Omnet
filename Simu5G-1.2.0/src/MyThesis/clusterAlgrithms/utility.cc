/*
 * utility.cc
 *
 *  Created on: Jan 22, 2022
 *      Author: veins
 */




#include "utility.h"

#include <algorithm>
#include <boost/bind/bind.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <vector>
using namespace veins;
/**
 * Return sthe eucledian distance between two vehicles.
 * @param v1 First vehicle.
 * @param v2 Second vehicle.
 * @return Eucledian distance between two vehicles.
 */


double Utility::eucd(Coord &a,Coord &b){
    double x = b.x - a.x; //calculating number to square in next step
    double y = b.y - a.y;
    double dist;
    dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
    dist = sqrt(dist);
    return dist;
}

double Utility::eucd(const std::pair<double, double> &p1, const std::pair<double, double> &p2) {
    return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
}

int Utility::min_index(std::vector<double> vec) {
    std::vector<double>::iterator smallest_dist =
        std::min_element(vec.begin(), vec.end());
    return std::distance(vec.begin(), smallest_dist);
}

MyVeinsAppRSU *myRSUx = new MyVeinsAppRSU();
CellularTower *myCellTowerx = new CellularTower();
std::vector<CarInfo*> Utility::getAllVehilcesTable(){
    map<int, CarInfo*> RSUvehicleStateMap;
    map<int, CarInfo*> CTvehicleStateMap;

    std::vector<CarInfo*> RSUVehicles;
    std::vector<CarInfo*> CellularTowerVehicles;

    std::vector<CarInfo*> result;

    RSUvehicleStateMap = myRSUx->getVehicleState();
    CTvehicleStateMap = myCellTowerx->getVehicleState();

    RSUVehicles = MapToVector(RSUvehicleStateMap);//get all cars' info in vector
    CellularTowerVehicles = MapToVector(CTvehicleStateMap);//get all cars' info in vector

    SetRelConnectivity(RSUVehicles);
    SetRelConnectivity(CellularTowerVehicles);

    for(int i=0; i<RSUVehicles.size(); i++){
        CarInfo *temp;
        bool add = true;
        for(int j=0; j<CellularTowerVehicles.size(); j++){
            if(RSUVehicles[i]->getCarID() == CellularTowerVehicles[j]->getCarID()){
                if(RSUVehicles[i]->getConnectivity() < CellularTowerVehicles[j]->getConnectivity()){
                    temp = CellularTowerVehicles[j];
                    temp->setConnectivity(RSUVehicles[i]->getConnectivity() + CellularTowerVehicles[j]->getConnectivity());
                    add = false;
                }else{
                    temp = RSUVehicles[i];
                    temp->setConnectivity(RSUVehicles[i]->getConnectivity() + CellularTowerVehicles[j]->getConnectivity());
                    add = false;
                }
            }
        }

        if(add){
            temp = RSUVehicles[i];
        }
        result.push_back(temp);
    }

    for(int i=0; i<CellularTowerVehicles.size(); i++){
        CarInfo *temp;
        bool add = true;
        for(int j=0; j<result.size(); j++){
            if(CellularTowerVehicles[i]->getCarID() == result[j]->getCarID()){
                add = false;
            }
        }
        if(add){
            temp = CellularTowerVehicles[i];
            result.push_back(temp);
        }
    }
    return result;
}



std::vector<double> Utility::SetRelConnectivity(std::vector<CarInfo*> const &v){
    std::vector<double> Dist;
    std::vector<double> RSSI;
    std::vector<double> Time;
    std::vector<double> Cent;

    std::vector<double> distRel;
    std::vector<double> RSSIRel;
    std::vector<double> TimeRel;
    std::vector<double> CentralityRel;

    for(int i=0; i<v.size(); i++){
        if(!isnan(v[i]->getNextDistance()) && v[i]->getNextDistance()!=0 &&v[i]->getNextDistance()<10000 &&v[i]->getNextDistance()>1){
            Dist.push_back(v[i]->getNextDistance());
        }
        RSSI.push_back(v[i]->getRSSI());
        if(isfinite(v[i]->getStayInRSUTime())){
            Time.push_back(v[i]->getStayInRSUTime());
        }
        Cent.push_back(v[i]->getNeighbours().size());
    }

    double maxVehicleDist;//calculate speed
    double minVehicleDist = 10000;
    double maxVehicleRSSI;//calculate RSSI
    double minVehicleRSSI = 10000;
    double maxVehicleTime;//calculate Time
    double minVehicleTime = 10000;
    double maxClusterCentrality;


    for(int i=0; i<Dist.size(); i++){
        if(maxVehicleDist<Dist[i]){
            maxVehicleDist = Dist[i];
        }
        if(minVehicleDist>Dist[i]){
            minVehicleDist = Dist[i];
        }
    }

    for(int i=0; i<RSSI.size(); i++){
        if(maxVehicleRSSI<RSSI[i]){
            maxVehicleRSSI = RSSI[i];
        }
        if(minVehicleDist>RSSI[i]){
            minVehicleRSSI = RSSI[i];
        }
    }

    for(int i=0; i<Time.size(); i++){
        if(maxVehicleTime<Time[i]){
            maxVehicleTime = Time[i];
        }
        if(minVehicleTime>Time[i]){
            minVehicleTime = Time[i];
        }
    }

    for(int i=0; i<Cent.size(); i++){//calculate ClusterCentrality
        if(maxClusterCentrality<Cent[i]){
            maxClusterCentrality = Cent[i];
        }
    }


    for(int e = 0; e<v.size(); e++){
        if(!isnan(v[e]->getNextDistance()) && v[e]->getNextDistance()!=0 &&v[e]->getNextDistance()<10000 &&v[e]->getNextDistance()>1 && (maxVehicleDist-minVehicleDist)>0.1){
            distRel.push_back((v[e]->getNextDistance()-minVehicleDist)/(maxVehicleDist-minVehicleDist));//recording speed
        }else{
            distRel.push_back(0.1);
        }

        if((maxVehicleRSSI-minVehicleRSSI)>0.1){
            RSSIRel.push_back((v[e]->getRSSI()-minVehicleRSSI)/(maxVehicleRSSI-minVehicleRSSI));//recording RSSI
        }else{
            RSSIRel.push_back(1);
        }

        if(isfinite(v[e]->getStayInRSUTime())&& (maxVehicleTime-minVehicleTime)>0.1){
            TimeRel.push_back((v[e]->getStayInRSUTime()-minVehicleTime)/(maxVehicleTime-minVehicleTime));//recording Time
        }else{
            TimeRel.push_back(1);
        }

        CentralityRel.push_back((v[e]->getNeighbours().size())/(maxClusterCentrality));//recording ClusterCentrality
    }

    std::vector<double> connectivity;
    for(int a = 0; a<v.size(); a++){
        connectivity.push_back((1-distRel[a])+RSSIRel[a]+TimeRel[a]+CentralityRel[a]);
        v[a]->setConnectivity((1-distRel[a])+RSSIRel[a]+TimeRel[a]+CentralityRel[a]);
    }
    return connectivity;
}


void Utility::printVectors(std::vector<CarInfo*> const &v) {
    std::cout << "-------------print vector------------" << endl;

    for(int i=0; i<v.size(); i++){
                std::cout << " | carID: " << v[i]->getCarID() << " | RSUID: " << v[i]->getRSUID() << " | distance: " << v[i]->getEuclidean_Distance() << " | timeStamp: " << v[i]->getTimestamp() << " | Counter is: " << v[i]->getCounter() << " | Velocity is: " << v[i]->getVelocity() << " | Next distance is: " << v[i]->getNextDistance() << " | Driving distance is: " << v[i]->getDrivingDistance() << " | Stay In RSU Time is: " << v[i]->getStayInRSUTime() << " | RSSI is: " << v[i]->getRSSI() << endl;
    }
}

void Utility::printDoubleVectors(const std::vector<std::vector<CarInfo*>> &cluster) {
    std::cout << "-------------print nested vector------------" << endl;

    for(int i=0; i<cluster.size(); i++){
        std::cout << "-------I am currently in cluster: "  << i << "--------" << endl;
        if(!cluster[i].empty()){
            for(int j=0; j<cluster[i].size(); j++){
                std::cout << " | carID: " << cluster[i][j]->getCarID() << " | RSUID: " << cluster[i][j]->getRSUID() << " | distance: " << cluster[i][j]->getEuclidean_Distance() << " | timeStamp: " << cluster[i][j]->getTimestamp() << " | Counter is: " << cluster[i][j]->getCounter() << " | Velocity is: " << cluster[i][j]->getVelocity() << " | Next distance is: " << cluster[i][j]->getNextDistance() << " | Driving distance is: " << cluster[i][j]->getDrivingDistance() << " | Stay In RSU Time is: " << cluster[i][j]->getStayInRSUTime() << " | RSSI is: " << cluster[i][j]->getRSSI() << endl;
            }
        }
    }
}



