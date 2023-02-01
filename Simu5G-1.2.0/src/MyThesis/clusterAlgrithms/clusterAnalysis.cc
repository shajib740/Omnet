/*
 * clusterAnalysis.cc
 *
 *  Created on: Mar 29, 2022
 *      Author: root
 */



#include "clusterAnalysis.h"
#include "utility.h"
#include "kmeans.h"

using namespace veins;

MyVeinsAppRSU *RSUData = new MyVeinsAppRSU();
MyVeinsAppCar *CarData = new MyVeinsAppCar();

string testFileNameDensity = "Density100";

string testFileName = "";

ofstream myfile;


void RunAnalysisCluster(std::vector<std::vector<CarInfo*>> KmeansInital, std::vector<std::vector<CarInfo*>> DBScanInital,int numberRSU,bool enableNodeB,int beaconInterval){

    int gNodeb = 0;
    if(enableNodeB){
        gNodeb = 1;
    }

    stringstream ss;
    ss << numberRSU;
    string numberRSU1 = ss.str();

    stringstream sss;
    sss << gNodeb;
    string gNodeb1 = sss.str();

    stringstream ssss;
    ssss << beaconInterval;
    string beaconInterval1 = ssss.str();

    testFileName = "RSU"+numberRSU1+"GNodeB"+gNodeb1+"Interval"+beaconInterval1;

    std::cout << testFileName << endl;

    CompareDBScanAndKmeans(KmeansInital,  DBScanInital);// record largest/smallest cluster size in Kmeans/DBScan (in initialized part, so all records should be same)
    AddNeighbours();
    AddDelayInfo();
    WriteTitle();
    WriteClusterDistribution();

    WriteClusterConnectivity();
    WriteNetworkCluster();
}



void WriteTitle(){
    int numberOfAllCars = CarData->getAllCars().size();
//    std::cout << "                   " << endl;
//    std::cout << "======== Clusters distribution at simulation time: " << simTime() << " ========" << endl;
//    std::cout << "======== Scenario setting:  Cologne Map ========" << endl;
//    std::cout << "======== Parameters setting: | Car density: "<< numberOfAllCars <<" | RSU density: 7 | "<< "5G: True " <<"========" << endl;
//    std::cout << "                   " << endl;
}

void WriteClusterConnectivity(){
    std::vector<std::vector<CarInfo*>> clusters = RSUData->getClusterState();

    double avgConnectivity;
    double temp = 0;

    for(int i = 0; i<clusters.size(); i++){
        temp += clusters[i][0]->getClusterConnectivity();
    }
    avgConnectivity = temp/clusters.size();


    myfile.open(testFileNameDensity+"/"+testFileName+"/ClusterConnectivity/BestClusterConnectivity.txt",ios::app);
    myfile << clusters[0][0]->getClusterConnectivity() << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/ClusterConnectivity/WorstClusterConnectivity.txt",ios::app);
    myfile << clusters[clusters.size()-1][0]->getClusterConnectivity() << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/ClusterConnectivity/AvgClusterConnectivity.txt",ios::app);
    myfile << avgConnectivity << endl;
    myfile.close();
}




void CompareDBScanAndKmeans(std::vector<std::vector<CarInfo*>> KmeansInital, std::vector<std::vector<CarInfo*>> DBScanInital){
//    std::cout << "      Kmeans initial the cluster with 'k' value: 7 clusters, and DBScan initial the cluster with " << DBScanInital.size() << " clusters." << endl;

    int largest = 0;
    string largestBelong = "none";
    int smallest = 1000;
    string smallestBelong = "none";

    for(int i=0; i<KmeansInital.size(); i++){
        if(KmeansInital[i].size() > largest){
            largest = KmeansInital[i].size();
            largestBelong = "KmeansInital";
        }
        if(KmeansInital[i].size() < smallest){
            smallest = KmeansInital[i].size();
            smallestBelong = "KmeansInital";
        }
    }


    myfile.open(testFileNameDensity+"/"+testFileName+"/InitialCompare/KmeanLargestCluster.txt",ios::app);
    myfile << largest << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/InitialCompare/KmeansamllestCluster.txt",ios::app);
    myfile << smallest << endl;
    myfile.close();



    int DBlargest = 0;
    int DBsmallest = 1000;
    for(int i=0; i<DBScanInital.size(); i++){
        if(DBScanInital[i].size() > DBlargest){
            DBlargest = DBScanInital[i].size();
            largestBelong = "DBScanInital";
        }
        if(DBScanInital[i].size() < DBsmallest){
            DBsmallest = DBScanInital[i].size();
            smallestBelong = "DBScanInital";
        }
    }

//    std::cout << " Comparing with these two technical to initial cluster, the largest cluster has " << largest << " clusters, which is from " << largestBelong << " technical. | the smallest cluster has " << smallest << " clusters, which is from " << smallestBelong << endl;

    myfile.open(testFileNameDensity+"/"+testFileName+"/InitialCompare/DBLargestCluster.txt",ios::app);
    myfile << DBlargest << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/InitialCompare/DBsamllestCluster.txt",ios::app);
    myfile << DBsmallest << endl;
    myfile.close();


    myfile.open(testFileNameDensity+"/"+testFileName+"/InitialCompare/DBClusterSize.txt",ios::app);
    myfile << DBScanInital.size() << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/InitialCompare/KmeansClusterSize.txt",ios::app);
    myfile << KmeansInital.size() << endl;
    myfile.close();
}


void WriteNetworkCluster(){
    std::vector<CarInfo*> temp;
    temp = CarData->getDelayCars();

    std::vector<double> recordDelayInAllClusters;
    std::vector<std::vector<CarInfo*>> clusters = RSUData->getClusterState();

    std::vector<double> recordPacketLossInAllClusters;


    for(int i=0; i<clusters.size(); i++){
//        std::cout << "-------I am currently in cluster: " << i << " -------------" << endl;
//        std::vector<CarInfo*> oneHop;
//        std::vector<CarInfo*> twoHops;
//        std::vector<CarInfo*> threeHops;


        double clusterDelay = 0;
        double recordNotFind = 0;
        for(int j=0; j<clusters[i].size(); j++){//for a specific cluster, getting the unconnected vehicles
            bool find = false;//not find a car by 3 hops, we loss it
            for(int m=0; m<temp.size(); m++){
                if(temp[m]->getCarID() == clusters[i][j]->getCarID()){
                    find = true;
//                    std::cout << "RSU ID is: " << temp[m]->getRSUID() << " | Car ID is: " << temp[m]->getCarID() << " | sorce ID is : " << temp[m]->getSourceID() << " | hops is: " << temp[m]->getHops() << " | delay is: " << temp[m]->getdelay() << endl;
                    clusterDelay = clusterDelay + temp[m]->getdelay();
                }
            }
            if(!find){
                recordNotFind+=1;
            }

        }
        clusterDelay = clusterDelay/clusters[i].size();

//        std::cout << "-------Cluster: " << i << " average delay is: " << clusterDelay <<" -------------" << endl;
        recordPacketLossInAllClusters.push_back(recordNotFind/clusters[i].size());
        recordDelayInAllClusters.push_back(clusterDelay);
    }

    double minDelayInAllClusters = 1000;  //test for minimum delay in all clusters
    for(int d=0; d<recordDelayInAllClusters.size(); d++){
        if(recordDelayInAllClusters[d] < minDelayInAllClusters){
            minDelayInAllClusters = recordDelayInAllClusters[d];
        }
    }

    double maxDelayInAllClusters = -1;//test for maximum delay in all clusters
    for(int d=0; d<recordDelayInAllClusters.size(); d++){
        if(recordDelayInAllClusters[d] > maxDelayInAllClusters){
            maxDelayInAllClusters = recordDelayInAllClusters[d];
        }
    }

    double avgDelayInAllClusters = -1;//test for average delay in all clusters
    double total = 0;
    for(int d=0; d<recordDelayInAllClusters.size(); d++){
        total += recordDelayInAllClusters[d];
    }
    avgDelayInAllClusters = total/recordDelayInAllClusters.size();


    myfile.open(testFileNameDensity+"/"+testFileName+"/NetworkClusters/minimumDelay.txt",ios::app);
    myfile << minDelayInAllClusters << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/NetworkClusters/maximumDelay.txt",ios::app);
    myfile << maxDelayInAllClusters << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/NetworkClusters/avgDelay.txt",ios::app);
    myfile << avgDelayInAllClusters << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/NetworkClusters/bestClusterDelay.txt",ios::app);
    myfile << recordDelayInAllClusters[0] << endl;
    myfile.close();




    double minPacketLoss = 1000;  //test for minimum delay in all clusters
    for(int d=0; d<recordPacketLossInAllClusters.size(); d++){
        if(recordPacketLossInAllClusters[d] < minPacketLoss){
            minPacketLoss = recordPacketLossInAllClusters[d];
        }
    }

    double maxPacketLoss = -1;//test for maximum delay in all clusters
    for(int d=0; d<recordPacketLossInAllClusters.size(); d++){
        if(recordPacketLossInAllClusters[d] > maxPacketLoss){
            maxPacketLoss = recordPacketLossInAllClusters[d];
        }
    }

    double avgPacketLoss = -1;//test for average delay in all clusters
    double totalloss = 0;
    for(int d=0; d<recordPacketLossInAllClusters.size(); d++){
        totalloss += recordPacketLossInAllClusters[d];
    }
    avgPacketLoss = totalloss/recordPacketLossInAllClusters.size();


    myfile.open(testFileNameDensity+"/"+testFileName+"/NetworkClusters/minimumPacketLoss.txt",ios::app);
    myfile << minPacketLoss << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/NetworkClusters/maximumPacketLoss.txt",ios::app);
    myfile << maxPacketLoss << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/NetworkClusters/avgPacketLoss.txt",ios::app);
    myfile << avgPacketLoss << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/NetworkClusters/bestClusterPacketLoss.txt",ios::app);
    myfile << recordPacketLossInAllClusters[0] << endl;
    myfile.close();

    CarData->clearDelayTable();
}


void AddDelayInfo(){
    std::vector<CarInfo*> allCars = CarData->getAllCars();
    std::vector<std::vector<CarInfo*>> clusters = RSUData->getClusterState();

    for(int k=0; k<allCars.size(); k++){

        for(int i=0; i<clusters.size(); i++){
            for(int j=0; j<clusters[i].size(); j++){
                if(clusters[i][j]->getCarID() == allCars[k]->getCarID()){
                    clusters[i][j]->setDelay(allCars[k]->getDelay());
                }
            }

        }
    }
    RSUData->setClusterState(clusters);
}


void AddNeighbours(){
    std::vector<CarInfo*> allCars = CarData->getAllCars();
    std::vector<std::vector<CarInfo*>> clusters = RSUData->getClusterState();


    for(int i=0; i<clusters.size(); i++){
        for(int j=0; j<clusters[i].size(); j++){
            clusters[i][j]->getNeighbours().clear();
            std::vector<CarInfo*> temp;
            Coord a = clusters[i][j]->getVehiclePosition();



            for(int k=0; k<allCars.size(); k++){
                Coord b = allCars[k]->getVehiclePosition();
                double dist = Utility::eucd(a,b);
                if(dist < 400 && clusters[i][j]->getCarID() != allCars[k]->getCarID()){
                    temp.push_back(allCars[k]);
                }
            }

            clusters[i][j]->setNeighbours(temp);
        }
    }

    RSUData->setClusterState(clusters);
}

void WriteClusterDistribution(){
    int MaxSize = 0;
    int MinSize = 1000;
    double AvgSize = 0;
    double tempAvgTotalSize = 0;
    std::vector<std::vector<CarInfo*>> clusters = RSUData->getClusterState();
    int clusterSize = clusters.size();
    for(int i=0; i<clusters.size(); i++){
        if(clusters[i].size() > MaxSize){
            MaxSize = clusters[i].size();//testing for clusters distribution
        }
        if(clusters[i].size() < MinSize){
            MinSize = clusters[i].size();//testing for clusters distribution
        }
        tempAvgTotalSize += clusters[i].size();
    }
    AvgSize = tempAvgTotalSize/clusters.size(); //testing for clusters distribution

    myfile.open(testFileNameDensity+"/"+testFileName+"/ClusterDistribution/maxClusterSize.txt",ios::app);
    myfile << MaxSize << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/ClusterDistribution/minClusterSize.txt",ios::app);
    myfile << MinSize << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/ClusterDistribution/avgClusterSize.txt",ios::app);
    myfile << AvgSize << endl;
    myfile.close();

    myfile.open(testFileNameDensity+"/"+testFileName+"/ClusterDistribution/numOfClusters.txt",ios::app);
    myfile << clusterSize << endl;
    myfile.close();

//    std::cout << "======== Number of Clusters: " << clusters.size() << " | Max Size: " << MaxSize << " | Min Size: " << MinSize << " | Avg Size: " << AvgSize <<  " ========" << endl;
}


