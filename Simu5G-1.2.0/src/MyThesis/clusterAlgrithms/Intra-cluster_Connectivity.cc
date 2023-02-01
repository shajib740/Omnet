/*
 * Intra-cluster_Connectivity.cc
 *
 *  Created on: Jan 25, 2022
 *      Author: veins
 */




#include "Intra-cluster_Connectivity.h"
#include "utility.h"

using namespace veins;

MyVeinsAppRSU *RSU = new MyVeinsAppRSU();


std::vector<std::vector<CarInfo*>> runRankCluster(std::vector<std::vector<CarInfo*>> &cluster){
    std::vector<std::vector<CarInfo*>> result;
    std::vector<double> recordingClusterConnectivity;
    std::vector<double> sorted;

    std::vector<std::vector<double>> data;
    data = calculateData(cluster);
//    double betweenness = betweennessCentrality(cluster);
//    std::cout << betweenness << endl;



    for(int c = 0; c<cluster.size(); c++){
        double clusterConnectivity = 0;
        clusterConnectivity =  (1-data[c][0])+data[c][1]+data[c][2]+data[c][3];//Weight the parameters and get the inner connectivity of the cluster which based on vehicles-RSU connectivity.
        recordingClusterConnectivity.push_back(clusterConnectivity);
        sorted.push_back(clusterConnectivity);
    }

    sort(sorted.rbegin(),sorted.rend());




    vector<double> :: iterator it;
    for(int i = 0; i<cluster.size(); i++){
        double key = sorted[i];
        it = find(recordingClusterConnectivity.begin(),recordingClusterConnectivity.end(),key);
        if(it != recordingClusterConnectivity.end()){
            result.push_back(cluster[distance(recordingClusterConnectivity.begin(),it)]);
        }
    }


    for(int i = 0; i<result.size(); i++){
        for(int j = 0; j<result[i].size(); j++){
            result[i][j]->setClusterConnectivity(0);
        }
        result[i][0]->setClusterConnectivity(sorted[i]);
    }

    return result;
}




double RunIntraCluster_Connectivity(const std::vector<std::vector<CarInfo*>> &cluster){
    double totalConnectivityValue;

    std::vector<std::vector<double>> data;
    data = calculateData(cluster);

    for(int c = 0; c<cluster.size(); c++){
        double clusterConnectivity = 0;
        clusterConnectivity =  (1-data[c][0])+data[c][1]+data[c][2]+data[c][3];//Weight the parameters and get the inner connectivity of the cluster which based on vehicles-RSU connectivity.
        totalConnectivityValue = totalConnectivityValue + clusterConnectivity;
    }
    return totalConnectivityValue;
};



double betweennessCentrality(const std::vector<std::vector<CarInfo*>> &cluster){
    double result;
    std::vector<std::vector<std::vector<CarInfo*>>> pairs = uniquePairOfNode(cluster);



    result = 0;
    return result;
}



std::vector<std::vector<CarInfo*>> clusterPairs;
std::vector<CarInfo*> recodingUnique;

std::vector<std::vector<std::vector<CarInfo*>>> uniquePairOfNode(const std::vector<std::vector<CarInfo*>> &cluster){
    std::vector<std::vector<std::vector<CarInfo*>>> result;
    for(int i = 0; i<cluster.size(); i++){
        for(int j = 0; j<cluster[i].size(); j++){
            recodingUnique.push_back(cluster[i][j]);
            searching(cluster[i][j],cluster[i]);
        }
    }
    return result;
}



int deeps;
CarInfo* illegle;
void searching(CarInfo* node, const std::vector<CarInfo*> &clu){

    bool adding = true;
    Coord a = node->getVehiclePosition();

    for(int p = 0; p<clu.size(); p++){
        if(node->getCarID() == clu[p]->getCarID()){//not self pairing
            adding = false;
        }

        if(adding != false){
            for(int q = 0; q<recodingUnique.size(); q++){//checking unique
                if(recodingUnique[q]->getCarID() == clu[p]->getCarID()){
                    adding = false;
                }
            }
        }

        if(adding != false){
            Coord b = clu[p]->getVehiclePosition();
            if(RSU->CalculateEuclideanDistance(a,b)>400){//checking distance
                adding = false;
            }
        }

        if(adding){
            int in = p;
            CarInfo* node1 = clu[p];
            recodingUnique.push_back(node1);
            clusterPairs.push_back(recodingUnique);
            deeps+=1;
            searching(node1, clu);
            std::cout << in << endl;
            p = in;
        }

        adding = true;
        if(recodingUnique.size()>1 && p == clu.size()-1){
            illegle = recodingUnique[recodingUnique.size()-1];
            recodingUnique.pop_back();

        }
    }

    deeps = deeps - 1;
}












std::vector<std::vector<double>> calculateData(const std::vector<std::vector<CarInfo*>> &cluster){
    std::vector<std::vector<double>> result;//variable of speed
    std::vector<double> avgClusterDist;
    std::vector<double> recordingDist;
    std::vector<double> distRel;
    double tempSumSpeed;

    std::vector<double> avgClusterRSSI;//variable of RSSI
    std::vector<double> recordingRSSI;
    std::vector<double> RSSIRel;
    double tempSumRSSI;

    std::vector<double> avgClusterTime;//variable of stayInRSUTime
    std::vector<double> recordingTime;
    std::vector<double> TimeRel;
    double tempSumTime;

    std::vector<double> avgClusterCentrality;//variable of Centrality
    std::vector<double> recordingCentrality;
    std::vector<double> CentralityRel;
    double tempSumCentrality;
    double tempCarCentrality;

    for(int c = 0; c<cluster.size(); c++){//select data
        for(int d = 0; d<cluster[c].size(); d++){
            if(!isnan(cluster[c][d]->getNextDistance()) && cluster[c][d]->getNextDistance()!=0 ){
                tempSumSpeed = tempSumSpeed + cluster[c][d]->getNextDistance();//selecet speed
                recordingDist.push_back(cluster[c][d]->getNextDistance());
            }

            tempSumRSSI = tempSumRSSI + cluster[c][d]->getRSSI();//selecet RSSI
            recordingRSSI.push_back(cluster[c][d]->getRSSI());

            if(isfinite(cluster[c][d]->getStayInRSUTime())){
                tempSumTime = tempSumTime + cluster[c][d]->getStayInRSUTime();//selecet stay in RSU time
                recordingTime.push_back(cluster[c][d]->getStayInRSUTime());
            }

            for(int e = 0; e<cluster[c].size(); e++){
                Coord a = cluster[c][d]->getVehiclePosition();
                Coord b = cluster[c][e]->getVehiclePosition();
                if(RSU->CalculateEuclideanDistance(a,b)<=400){
                    tempSumCentrality = tempSumCentrality + 1;
                    tempCarCentrality = tempCarCentrality + 1;
                }
            }
            recordingCentrality.push_back(tempCarCentrality);
            tempCarCentrality = 0;
        }


        avgClusterCentrality.push_back(tempSumCentrality/cluster[c].size());
        tempSumCentrality = 0;

        avgClusterDist.push_back(tempSumSpeed/cluster[c].size());//selecet speed
        tempSumSpeed = 0;

        avgClusterRSSI.push_back(tempSumRSSI/cluster[c].size());//selecet RSSI
        tempSumRSSI = 0;

        avgClusterTime.push_back(tempSumTime/cluster[c].size());//selecet Time
        tempSumTime = 0;
    }

    double maxVehicleDist = *max_element(recordingDist.begin(),recordingDist.end());//calculate speed
    double minVehicleDist = *min_element(recordingDist.begin(),recordingDist.end());

    double maxVehicleRSSI = *max_element(recordingRSSI.begin(),recordingRSSI.end());//calculate RSSI
    double minVehicleRSSI = *min_element(recordingRSSI.begin(),recordingRSSI.end());

    double maxVehicleTime = *max_element(recordingTime.begin(),recordingTime.end());//calculate Time
    double minVehicleTime = *min_element(recordingTime.begin(),recordingTime.end());

    double maxClusterCentrality = *max_element(recordingCentrality.begin(),recordingCentrality.end());//calculate ClusterCentrality

    for(int e = 0; e<cluster.size(); e++){
        distRel.push_back((avgClusterDist[e]-minVehicleDist)/(maxVehicleDist-minVehicleDist));//recording speed
        RSSIRel.push_back((avgClusterRSSI[e]-minVehicleRSSI)/(maxVehicleRSSI-minVehicleRSSI));//recording RSSI
        TimeRel.push_back((avgClusterTime[e]-minVehicleTime)/(maxVehicleTime-minVehicleTime));//recording Time
        CentralityRel.push_back((avgClusterCentrality[e])/(maxClusterCentrality));//recording ClusterCentrality
    }

    for(int f = 0; f<cluster.size(); f++){//make data(1.distance, 2.RSSI, 3.time 4.ClusterCentrality)
        std::vector<double> temp;

        temp.push_back(distRel[f]);
        temp.push_back(RSSIRel[f]);
        temp.push_back(TimeRel[f]);
        temp.push_back(CentralityRel[f]);
        result.push_back(temp);
    }

    return result;
}










