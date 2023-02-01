/*
 * kmeans.h
 *
 *  Created on: Jan 21, 2022
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
#include <MyThesis/MyClass/CellularTower.h>
#include <MyThesis/clusterAlgrithms/Intra-cluster_Connectivity.h>


using namespace veins;

std::vector<std::vector<CarInfo*>> RunKmeans();

std::vector<std::vector<CarInfo*>> RunSplitKmeans(std::vector<CarInfo*> &v);

std::vector<CarInfo*> MapToVector(map<int, CarInfo*> const &m);
map<int, CarInfo*> VectorToMap(std::vector<CarInfo*> const &v);


std::vector<int> kpp_cntrds(std::vector<CarInfo*> const &v, const size_t k);

size_t kpp_next_cntrd(std::vector<int> &cntrds, const std::vector<CarInfo*> &vehs);

std::set<int> select_cntrd(const std::vector<CarInfo*> &vehs, const size_t k);

std::vector<std::vector<CarInfo*>> init_cluster(const std::vector<CarInfo*> &vehs, const size_t k);

std::vector<std::vector<CarInfo*>> make_cluster(const std::vector<CarInfo*> &vehs, const size_t k, const size_t times);

std::vector<std::pair<double, double>> select_cntrd(const std::vector<std::vector<CarInfo*>> &cluster);

double cluster_var(const std::vector<std::vector<CarInfo*>> &cluster);

double pos_mean(const std::vector<CarInfo*> &vehs, char cord);

double diff_sum(const std::vector<CarInfo*> &vehs, double mean, char cord);











