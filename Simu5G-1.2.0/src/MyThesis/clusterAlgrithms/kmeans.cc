/*
 * kmeans.cc
 *
 *  Created on: Jan 21, 2022
 *      Author: veins
 */



#include "kmeans.h"
#include "utility.h"

using namespace veins;


MyVeinsAppRSU *myRSU = new MyVeinsAppRSU();


double alpha = 0.3;

std::vector<std::vector<CarInfo*>> RunKmeans(){

    std::vector<CarInfo*> vehicleStateVector;
    vehicleStateVector = Utility::getAllVehilcesTable();

    std::vector<std::vector<CarInfo*>> result;
    result = make_cluster(vehicleStateVector,7,10);//k-means to get the clusters

    std::vector<std::vector<CarInfo*>> ranked = runRankCluster(result);//ranking the cluster and print out from high quality cluster to the low quality cluster.

    Utility::printDoubleVectors(ranked);
    myRSU->setClusterState(ranked);
    return ranked;
}

std::vector<std::vector<CarInfo*>> RunSplitKmeans(std::vector<CarInfo*> &v){
    std::vector<std::vector<CarInfo*>> result;
    result = make_cluster(v,2,10);//k-means to get the clusters
    return result;
}



std::vector<int> kpp_cntrds(std::vector<CarInfo*> const &vehs,const size_t k) {
    std::vector<int> indexs{};

    indexs.push_back(rand() % vehs.size());

    for (size_t i = 0; i < k-1; ++i){
        indexs.push_back(kpp_next_cntrd(indexs, vehs));
    }

    return indexs;
}


size_t kpp_next_cntrd(std::vector<int> &cntrds, const std::vector<CarInfo*> &vehs) {
    std::vector<float> dists(vehs.size(), std::numeric_limits<float>::max());
        for (size_t i = 0; i < cntrds.size(); i++) {
            for (size_t j = 0; j < vehs.size(); j++) {
                if (vehs.at(i) == vehs.at(j)) {
                    continue;
                }

                Coord a = vehs.at(cntrds.at(i))->getVehiclePosition();
                Coord b = vehs.at(j)->getVehiclePosition();
                float d = myRSU->CalculateEuclideanDistance(a,b);

                if (d < dists.at(j)) {
                    dists.at(j) = d;
                }
            }
        }

        float max_dist = *std::max_element(dists.begin(), dists.end());
        size_t index = std::find(dists.begin(), dists.end(), max_dist) - dists.begin();

        return index;
}

std::set<int> select_cntrd(const std::vector<CarInfo*> &vehs, const size_t k) {
    std::set<int> init_cluster{};

    while (init_cluster.size() < k) {
        init_cluster.insert(rand() % vehs.size());
    }

    return init_cluster;
}


std::vector<std::vector<CarInfo*>> init_cluster(const std::vector<CarInfo*> &vehs, const size_t k) {
    // outer vector is the cluster number
    // inner vector is the vehicles in the cluster.
    std::vector<std::vector<CarInfo*>> cluster{k, std::vector<CarInfo*>{}};

    std::vector<int> cntrds = kpp_cntrds(vehs, k);

    std::vector<int>::iterator it = cntrds.begin();

    for (size_t i = 0; i < k; i++) {
        cluster.at(i).push_back(vehs.at(*(it++)));
    }

    for (size_t i = 0; i < vehs.size(); i++) {
        std::vector<double> dist{};

        if (std::find(cntrds.begin(), cntrds.end(), i) != cntrds.end()) {
            continue;
        }

        for (int c : cntrds) {
            Coord a = vehs.at(i)->getVehiclePosition();
            Coord b = vehs.at(c)->getVehiclePosition();
            float d = myRSU->CalculateEuclideanDistance(a,b);
            dist.push_back(d);
        }

        int cluster_id = Utility::min_index(dist);


        std::vector<int> vec;
        vec.assign(cntrds.begin(),cntrds.end());
        int center = vec.at(cluster_id);

        Coord centerC = vehs.at(center)->getVehiclePosition();
        Coord testVeh = vehs.at(i)->getVehiclePosition();
        double commDist = myRSU->CalculateEuclideanDistance(centerC,testVeh);

        // add the vehicle to the closest centroid (communication distance must < 400)
        if(commDist < 400){
            cluster.at(cluster_id).push_back(vehs.at(i));
        }
    }

    return cluster;
}


/**
 * Puts all the vehicles with their parent cluster.
 * @param vehs Al the vehicles.
 * @param k Number of clusters.
 * @return The final clusters, key is the cluster-number and the value is the
 * list of all the vehicles in that cluster.
 */
std::vector<std::vector<CarInfo*>>
next_cluster(const std::vector<std::vector<CarInfo*>> &parent_cluster,
             const std::vector<CarInfo*> &vehs, const size_t k) {
    std::vector<std::vector<CarInfo*>> cluster{k, std::vector<CarInfo*>{}};

    std::vector<std::pair<double, double>> centroids =
            select_cntrd(parent_cluster);

    for (size_t i = 0; i < vehs.size(); i++) {
        std::vector<double> dist{};

        for (std::pair<double, double> c : centroids) {
            Coord c_pos;
            c_pos.x = c.first;
            c_pos.y = c.second;
            Coord a = vehs.at(i)->getVehiclePosition();
            double d = Utility::eucd(a, c_pos);
            dist.push_back(d);
        }

        int cluster_id = Utility::min_index(dist);

        // add the vehicle to the closest centroid
        cluster.at(cluster_id).push_back(vehs.at(i));
    }

    return cluster;
}

std::vector<std::pair<double, double>> select_cntrd(const std::vector<std::vector<CarInfo*>> &cluster) {
    std::vector<std::pair<double, double>> new_cntrds{};

    for (size_t i = 0; i < cluster.size(); i++) {
        std::pair<double, double> centroid{0, 0};

        for (size_t j = 0; j < cluster.at(i).size(); j++) {
            centroid.first += cluster.at(i).at(j)->getVehiclePosition().x;
            centroid.second += cluster.at(i).at(j)->getVehiclePosition().y;
        }

        centroid.first = centroid.first / cluster.at(i).size();
        centroid.second = centroid.second / cluster.at(i).size();

        new_cntrds.push_back(centroid);
    }

    return new_cntrds;
}

std::vector<std::vector<CarInfo*>> make_cluster(const std::vector<CarInfo*> &vehs, const size_t k, const size_t times) {

    std::vector<std::vector<CarInfo*>> old_cluster = init_cluster(vehs, k);

    std::vector<std::vector<CarInfo*>> new_cluster{};

    for (size_t i = 0; i < times; i++) {
        new_cluster = next_cluster(old_cluster, vehs, k);

//         std::stringstream ss;
//         ss << "iterations/kmeans" << i << ".txt";
//         Utility::write_cluster(new_cluster, ss.str());

//        std::cout << "old " << cluster_var(old_cluster) << " new " << cluster_var(new_cluster) << std::endl;

        if (cluster_var(new_cluster) >= cluster_var(old_cluster)) {
            return old_cluster;
        }

        old_cluster = new_cluster;
    }

    return new_cluster;
}

double cluster_var(const std::vector<std::vector<CarInfo*>> &cluster) {
    if (cluster.empty()) {
        return DBL_MAX;
    }

    double total_variance = 0;

    for (size_t i = 0; i < cluster.size(); i++) {

        std::vector<CarInfo*> cluster_vehs = cluster.at(i);
        if (cluster_vehs.empty()) {
            continue;
        }

        double x_mean = pos_mean(cluster_vehs, 'x');
        double y_mean = pos_mean(cluster_vehs, 'y');

        double x_variance = pow(diff_sum(cluster_vehs, x_mean, 'x'), 2);
        double y_variance = pow(diff_sum(cluster_vehs, y_mean, 'y'), 2);

        total_variance += x_variance + y_variance;
    }

    double connectivity = RunIntraCluster_Connectivity(cluster);//larger is good

    double connectivityVariance = alpha * total_variance + (1-alpha) * (1 / connectivity);

    return connectivityVariance;//lower is good
}


double pos_mean(const std::vector<CarInfo*> &vehs, char cord) {
    double running_total = 0;

    for (CarInfo* v : vehs) {
        switch (cord) {
            case 'x': {
                running_total += v->getVehiclePosition().x;
            } break;

            case 'y': {
                running_total += v->getVehiclePosition().y;
            }

            default:
                break;
        }
    }

    return running_total / vehs.size();
}


double diff_sum(const std::vector<CarInfo*> &vehs, double mean, char cord) {
    double running_diff_sum = 0;

    for (CarInfo* v : vehs) {
        switch (cord) {
            case 'x': {
                running_diff_sum += pow(v->getVehiclePosition().x - mean, 2);
            } break;

            case 'y': {
                running_diff_sum += pow(v->getVehiclePosition().y - mean, 2);
            }

            default:
                break;
        }
    }

    return running_diff_sum / vehs.size();
}



std::vector<CarInfo*> MapToVector(map<int, CarInfo*> const &m){
    std::vector<CarInfo*> temp;
    for(const auto &key: m) {
        temp.push_back(key.second);
    }
    return temp;
}

map<int, CarInfo*> VectorToMap(std::vector<CarInfo*> const &v){
    map<int, CarInfo*> temp;
    for(int i = 0; i<v.size(); i++){
        temp.insert(std::make_pair(v[i]->getCarID(), v[i]));
    }
    return temp;
}



