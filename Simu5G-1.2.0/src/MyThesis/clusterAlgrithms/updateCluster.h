/*
 * updateCluster.h
 *
 *  Created on: Feb 15, 2022
 *      Author: veins
 */

#pragma once
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <vector>
#include <map>
#include <set>
#include <vector>
#include <MyThesis/MyClass/CarInfo.h>
#include <MyThesis/MyClass/MyVeinsAppRSU.h>
#include <MyThesis/MyClass/MyVeinsAppCar.h>
#include <MyThesis/MyClass/CellularTower.h>
#include <MyThesis/clusterAlgrithms/Intra-cluster_Connectivity.h>



using namespace veins;

std::vector<CarInfo*> MapToVector(map<int, CarInfo*> const &m);
map<int, CarInfo*> VectorToMap(std::vector<CarInfo*> const &v);

void RunUpdateCluster(bool gNodeb);

CarInfo* FindCenter(std::vector<CarInfo*> &v);//find center point, analysis/rank each of the node's connectivity. Making the highest connectivity node to be the new center point.(center point must be directly connect to RSU)

std::vector<std::vector<CarInfo*>> MakeClusters(std::vector<CarInfo*> &cent, std::vector<CarInfo*> const &vector);//based centers to add the neighbors nodes making the clusters, if a node belong to two clusters than rank the connectivity to decide it.

std::vector<std::vector<CarInfo*>> MakeClustersByRSU(std::vector<std::vector<CarInfo*>> &vector, std::vector<CarInfo*> const &vehicleTable);//if an RSU has more then 5 nearby vehicles, making clusters.

std::vector<std::vector<CarInfo*>> Merge(std::vector<std::vector<CarInfo*>> &v);//decide if there is needed to merge cluster. 1.sharing nodes>1, 2.cluster nodes<5

std::vector<std::vector<CarInfo*>> Split(std::vector<std::vector<CarInfo*>> &v, int numberOfAllCars);//decide if there is needed to split cluster. 1.total clusters < 3, 2.a cluster with more then 15 vehicles.

std::vector<std::vector<CarInfo*>> MultiHop(std::vector<std::vector<CarInfo*>> &AllVehiclesTable,std::vector<CarInfo*> &AllCars, std::vector<CarInfo*> &center, bool initial);

std::vector<std::vector<CarInfo*>> gNodeCovered(std::vector<std::vector<CarInfo*>> &AllVehiclesTable);

std::vector<std::vector<CarInfo*>> MultInfras(std::vector<std::vector<CarInfo*>> &AllVehiclesTable);

