/*
 * DBSCAN.cc
 *
 *  Created on: Jan 22, 2022
 *      Author: veins
 */



#include "DBSCAN.h"
#include "utility.h"

using namespace veins;

MyVeinsAppRSU *myRSUDBscan = new MyVeinsAppRSU();
map<int, CarInfo*> vehicleStateMap;
std::vector<CarInfo*> vehicleStateVector;
int clusterNum;



void DBSCAN::label(std::vector<CarInfo*> const &v) {


    // find core points
    for(int i = 0; i< v.size(); i++){
        size_t neighbors = 0;

        for(int j = 0; j< v.size(); j++){
            if(Utility::eucd(v[i]->getPoint().m_point, v[j]->getPoint().m_point) <= EPS){
                ++neighbors;
            }
        }

//        v[i]->getPoint().setM_type(neighbors >= MIN_POINTS ? CORE : NOISE);
        v[i]->setPoint(Point{std::make_pair(v[i]->getVehiclePosition().x, v[i]->getVehiclePosition().y), v[i]->getPoint().getM_clusterID(), neighbors >= MIN_POINTS ? CORE : NOISE});
    }

    // find border points
    for(int i = 0; i< v.size(); i++){

        if (v[i]->getPoint().m_type == CORE) continue;

        for(int j = 0; j< v.size(); j++){
            if(Utility::eucd(v[i]->getPoint().m_point, v[j]->getPoint().m_point) <= EPS && v[j]->getPoint().m_type == CORE){
//                v[i]->getPoint().setM_type(BORDER);
                v[i]->setPoint(Point{std::make_pair(v[i]->getVehiclePosition().x, v[i]->getVehiclePosition().y), v[i]->getPoint().getM_clusterID(), BORDER});
                break;
            }
        }
    }
}


void DBSCAN::make_clusters(std::vector<CarInfo*> const &v) {

    int c = 0;

    for(int i = 0; i< v.size(); i++){

        if (v[i]->getPoint().m_type != CORE) continue;

        if (v[i]->getPoint().m_clusterID == -1){
//            v[i]->getPoint().setM_clusterID(++c);
            v[i]->setPoint(Point{std::make_pair(v[i]->getVehiclePosition().x, v[i]->getVehiclePosition().y), c++, v[i]->getPoint().m_type});
        }

        for(int j = 0; j< v.size(); j++){
            if(v[j]->getPoint().m_clusterID == -1){
                if (Utility::eucd(v[i]->getPoint().m_point, v[j]->getPoint().m_point) <= EPS) {
                    v[j]->setPoint(Point{std::make_pair(v[j]->getVehiclePosition().x, v[j]->getVehiclePosition().y), v[i]->getPoint().m_clusterID, v[j]->getPoint().m_type});
//                  v[j]->getPoint().setM_clusterID(v[i]->getPoint().m_clusterID);
                }
            }
        }
    }

    clusterNum = c;
}

std::vector<Point> DBSCAN::MapToVectorPoint(map<int, CarInfo*>  &m){
    std::vector<Point> points{};

    for(const auto &key: m) {
        points.push_back(Point{std::make_pair(key.second->getVehiclePosition().x, key.second->getVehiclePosition().y), 0, UNCLASSIFIED});
    }
    return points;
}



std::vector<CarInfo*> DBSCAN::MapToVector(map<int, CarInfo*> const &m){
    std::vector<CarInfo*> temp;
    for(const auto &key: m) {
        key.second->setPoint(Point{std::make_pair(key.second->getVehiclePosition().x, key.second->getVehiclePosition().y), -1, UNCLASSIFIED});
        temp.push_back(key.second);
    }
    return temp;
}



std::vector<std::vector<CarInfo*>> DBSCAN::RunDBscan() {
    vehicleStateMap = myRSUDBscan->getVehicleState();//get all cars' info directly
    vehicleStateVector = MapToVector(vehicleStateMap);//get all cars' info in vector

//    std::vector<Point> points = MapToVectorPoint(vehicleStateMap);


    label(vehicleStateVector);
//
    make_clusters(vehicleStateVector);

    std::vector<std::vector<CarInfo*>> sorted;


    for(int j = 0; j<clusterNum; j++){
        std::vector<CarInfo*> temp;
        for(int k = 0; k<vehicleStateVector.size(); k++){
            if(vehicleStateVector[k]->getPoint().getM_clusterID() == j){
                temp.push_back(vehicleStateVector[k]);
            }
        }
        sorted.push_back(temp);
    }

    myRSUDBscan->printDoubleVectors(sorted);

    myRSUDBscan->setClusterState(sorted);

    return sorted;

//    for (int i=0; i<vehicleStateVector.size(); i++) {
//        std::cout << "(" << vehicleStateVector[i]->getPoint().m_point.first << "," << vehicleStateVector[i]->getPoint().m_point.second << ")  " << vehicleStateVector[i]->getPoint().m_clusterID << " " << vehicleStateVector[i]->getPoint().m_type << std::endl;
//    }
}

/**
 * Can border points have same number of points as core points?
 *
 */
