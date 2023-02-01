/*
 * clusterAnalysis.h
 *
 *  Created on: Mar 29, 2022
 *      Author: root
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


void RunAnalysisCluster(std::vector<std::vector<CarInfo*>> KmeansInital, std::vector<std::vector<CarInfo*>> DBScanInital,int numberRSU,bool enableNodeB,int beaconInterval);

void WriteTitle();
void WriteClusterDistribution();
void CompareDBScanAndKmeans(std::vector<std::vector<CarInfo*>> KmeansInital, std::vector<std::vector<CarInfo*>> DBScanInital);
void WriteNetworkCluster();
void WriteClusterConnectivity();



void AddNeighbours();
void AddDelayInfo();




