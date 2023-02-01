/*
 * Intra-cluster_Connectivity.h
 *
 *  Created on: Jan 25, 2022
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
#include <MyThesis/clusterAlgrithms/kmeans.h>


using namespace veins;


double RunIntraCluster_Connectivity(const std::vector<std::vector<CarInfo*>> &cluster);


std::vector<std::vector<double>> calculateData(const std::vector<std::vector<CarInfo*>> &cluster);

std::vector<std::vector<CarInfo*>> runRankCluster(std::vector<std::vector<CarInfo*>> &cluster);

double betweennessCentrality(const std::vector<std::vector<CarInfo*>> &cluster);

std::vector<std::vector<std::vector<CarInfo*>>> uniquePairOfNode(const std::vector<std::vector<CarInfo*>> &cluster);

void searching(CarInfo* node, const std::vector<CarInfo*> &clu);


