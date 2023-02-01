/*
 * updateCluster.cc
 *
 *  Created on: Feb 15, 2022
 *      Author: veins
 */


//1.add a new way to make clusters, if tehre is more then 5 vehicles nearby the RSU, then we trying to make a new cluster.

#include "updateCluster.h"
#include "utility.h"
#include "kmeans.h"

using namespace veins;


MyVeinsAppRSU *myRSU1 = new MyVeinsAppRSU();
MyVeinsAppCar *myCar1 = new MyVeinsAppCar();


void RunUpdateCluster(bool gNodeb){
    std::vector<CarInfo*> vehicleStateVector;

    vehicleStateVector = Utility::getAllVehilcesTable();
    std::vector<std::vector<CarInfo*>> clusters = myRSU1->getClusterState();

    std::vector<CarInfo*> AllCars = myCar1->getAllCars();
    int numberOfAllCars = AllCars.size();
    std::vector<std::vector<CarInfo*>> making_clusters;
    std::vector<std::vector<CarInfo*>> making_clustersB;
    std::vector<std::vector<CarInfo*>> making_clustersC;
    std::vector<std::vector<CarInfo*>> making_clustersTemp;
    std::vector<std::vector<CarInfo*>> making_clustersD;


    std::vector<std::vector<CarInfo*>> merge_clusters;
    std::vector<std::vector<CarInfo*>> split_clusters;





    std::vector<CarInfo*> centers;//find centers
    for(int i=0; i<clusters.size(); i++){
        bool makeCenter = false;
        for(int j=0; j<clusters[i].size(); j++){
            if(clusters[i][j]->getRSUID() != -1){
                makeCenter = true;
            }
        }
        if(clusters[i].size()>1 && makeCenter){
            centers.push_back(FindCenter(clusters[i]));
        }
    }


    making_clusters = MakeClusters(centers, vehicleStateVector);
    making_clustersB = MakeClustersByRSU(making_clusters, vehicleStateVector);
    making_clustersTemp = MultInfras(making_clustersB);//adding 5G info into cluster.



    for(int i=0; i<making_clustersTemp.size(); i++){//rule for add belong cluster
       if(making_clustersTemp[i].size()>2){
           for(int j=0; j<making_clustersTemp[i].size(); j++){
               making_clustersTemp[i][j]->setBelongCluster(making_clustersTemp[i][0]->getCarID());
           }
           making_clusters.push_back(making_clustersTemp[i]);
       }
    }

    merge_clusters = Merge(making_clusters);
    split_clusters = Split(merge_clusters, numberOfAllCars);
    making_clustersC = MultiHop(split_clusters,AllCars,centers,true);//make once multi-hop for current clusters
    std::vector<std::vector<CarInfo*>> ranked;
    if(gNodeb){
        making_clustersD = gNodeCovered(making_clustersC);//adding gNodeB info for cars
        ranked = runRankCluster(making_clustersD);//ranking the cluster and print out from high quality cluster to the low quality cluster.
    }else{
        ranked = runRankCluster(making_clustersC);//ranking the cluster and print out from high quality cluster to the low quality cluster.
    }

//    myRSU1->printDoubleVectors(ranked);
    myRSU1->setClusterState(ranked);

}


////adding 5G cover nodes to cluster table
std::vector<std::vector<CarInfo*>> gNodeCovered(std::vector<std::vector<CarInfo*>> &AllVehiclesTable){

    std::vector<CarInfo*> AllCars =  myCar1->getAllCars();

    for(int i=0; i<AllCars.size(); i++){
        bool adding = true;

        for(int k=0; k<AllVehiclesTable.size(); k++){
            for(int z=0; z<AllVehiclesTable[k].size(); z++){
                if(AllVehiclesTable[k][z]->getCarID() == AllCars[i]->getCarID()){
                    adding = false;
//                    std::cout << "adding: " << adding << endl;
                }

            }
        }

//        std::cout << "belong to RSU: " << AllCars[i]->getBelongCluster() << endl;
//        std::cout << "adding: " << adding << endl;

        if(adding){
            double min = 10000;
            double belonging;
            int clusterRecord;

            for(int x = 0; x<AllVehiclesTable.size(); x++){
                Coord a = AllVehiclesTable[x][0]->getVehiclePosition();
                Coord b = AllCars[i]->getVehiclePosition();


                double dist = Utility::eucd(a, b);

                if(dist < min){
                    min = dist;
                    belonging = AllVehiclesTable[x][0]->getCarID();
                    clusterRecord = x;
                }
            }

            AllCars[i]->setBelongCluster(belonging);
            AllVehiclesTable[clusterRecord].push_back(AllCars[i]);
        }
    }

    return AllVehiclesTable;
}






std::vector<int> kernel;//record kernel
std::vector<std::vector<CarInfo*>> MultInfras(std::vector<std::vector<CarInfo*>> &AllVehiclesTable){
    std::vector<std::vector<double>> gNodeBTable;
    std::cout << "------------------------------------test----------------------------------" << endl;
    ifstream f2;
    f2.open("gNodeB.txt");
    string line;
    int count = 0;

    std::vector<double> temp;
    while(getline(f2,line)){
        stringstream ss(line);
        double result;
        ss >> result;
        temp.push_back(result);
        count+=1;

        if(count==3){
            gNodeBTable.push_back(temp);
            temp.clear();
        }


//        std::cout << "Result is: " << result << endl;
        bool added = false;
        if(count!=3 && count%3==0){
//            std::cout << "the input temp is: " << temp[0] << " car is: " << temp[1] << "distance is : " << temp[2] << endl;
            for(int s=0; s<gNodeBTable.size(); s++){
                if(gNodeBTable[s][1] == temp[1] && gNodeBTable[s][2] > temp[2]){
//                    std::cout << "data: " << gNodeBTable[s][0] << " car is: " << gNodeBTable[s][1] << "distance is : " << gNodeBTable[s][2] << "!. Is WORTH than" << "data: " << temp[0] << " car is: " << temp[1] << "distance is : " << temp[2] << endl;
                    gNodeBTable[s][0] = temp[0];
                    gNodeBTable[s][2] = temp[2];
                    temp.clear();
                    added = true;
                }
                if(gNodeBTable[s][1] == temp[1] && gNodeBTable[s][2] < temp[2]){
//                    std::cout << "data: " << gNodeBTable[s][0] << " car is: " << gNodeBTable[s][1] << "distance is : " << gNodeBTable[s][2] << "!. Is BETTER than" << "data: " << temp[0] << " car is: " << temp[1] << "distance is : " << temp[2] << endl;
                    temp.clear();
                    added = true;
                }
            }
        }

        if(count!=3 && count%3==0 && temp[1]>2000){
            if(!added){
                gNodeBTable.push_back(temp);
                temp.clear();
                added = true;
            }
        }

    }

    f2.close();

    std::ofstream clear;
    clear.open("gNodeB.txt",std::ios::trunc);
    clear.close();


    for(int i=0; i<AllVehiclesTable.size(); i++){
        for(int j=0;j<AllVehiclesTable[i].size(); j++){
            for(int k=0; k<gNodeBTable.size(); k++){
                double fitWave = (AllVehiclesTable[i][j]->getCarID() - 336) / 56;
                double fitGnode = gNodeBTable[k][1] - 2049;
                if(fitWave == fitGnode){
                    double dist = AllVehiclesTable[i][j]->getNextDistance() - (300 - gNodeBTable[k][2]/10);
                    if(dist<10){dist = 10;};
                    AllVehiclesTable[i][j]->setNextDistance(dist);
                    kernel.push_back(gNodeBTable[k][1]);
                }
            }
        }
    }

    gNodeBTable.clear();
    return AllVehiclesTable;
}


std::vector<std::vector<CarInfo*>> MultiHop(std::vector<std::vector<CarInfo*>> &AllVehiclesTable,std::vector<CarInfo*> &AllCars,std::vector<CarInfo*> &center, bool initial){
    std::vector<std::vector<CarInfo*>> result;
    if(initial){//if first run multi hop set initial hops to 1
        for(int i=0; i<AllVehiclesTable.size(); i++){
            for(int j = 0; j<AllVehiclesTable[i].size(); j++){
                AllVehiclesTable[i][j]->setHops(1);
            }
        }

        for(int i=0; i<AllCars.size(); i++){
                AllCars[i]->setHops(1);
        }
    }



    for(int i=0; i<AllVehiclesTable.size(); i++){// multi hop for all cars that can directly connect to RSU
        for(int j = 0; j<AllVehiclesTable[i].size(); j++){
            std::vector<CarInfo*> neighbors;
            neighbors = AllVehiclesTable[i][j]->getNeighbours();

            for(int k = 0; k<neighbors.size(); k++){
                if(neighbors[k]->getCarID() != AllVehiclesTable[i][j]->getCarID()){
                    bool smallerHops = false;
                    if(AllVehiclesTable[i][j]->getHops()<(neighbors[k]->getHops()-1)){
                        smallerHops = true;
                        AllVehiclesTable[i][j]->getNeighbours()[k]->setHops(AllVehiclesTable[i][j]->getHops()+1);
                        AllVehiclesTable[i][j]->getNeighbours()[k]->setBelongCluster(AllVehiclesTable[i][j]->getBelongCluster());
                        CarInfo* temp = AllVehiclesTable[i][j]->getNeighbours()[k];
                        AllVehiclesTable[i].push_back(temp);
                    }

                    if(neighbors[k]->getBelongCluster()==0){
                        AllVehiclesTable[i][j]->getNeighbours()[k]->setBelongCluster(AllVehiclesTable[i][j]->getBelongCluster());
                        CarInfo* temp = AllVehiclesTable[i][j]->getNeighbours()[k];
                        AllVehiclesTable[i].push_back(temp);
                    }
                }
            }

        }
    }


    for(int i=0; i<AllVehiclesTable.size(); i++){//find all vehicles that didn't use to a cluster
        for(int j = 0; j<AllVehiclesTable[i].size(); j++){
            for(int k=0; k<AllCars.size(); k++){
                if(AllCars[k]->getCarID() == AllVehiclesTable[i][j]->getCarID()){
                    AllCars.erase(AllCars.begin()+k);
                    k = k-1;
                }
            }
        }
    }


    for(int i=0; i<AllVehiclesTable.size(); i++){
        std::vector<CarInfo*> temp;
            for(int j = 0; j<AllVehiclesTable[i].size(); j++){
                temp.push_back(AllVehiclesTable[i][j]);
            }
            result.push_back(temp);
    }

    for(int k=0; k<AllCars.size(); k++){
        double minDist = 400;
        int bestCluster = -1;
        int bestNeighbor;

        for(int i=0; i<AllVehiclesTable.size(); i++){
            for(int j = 0; j<AllVehiclesTable[i].size(); j++){

                Coord a = AllVehiclesTable[i][j]->getVehiclePosition();
                Coord b = AllCars[k]->getVehiclePosition();
                double dist = Utility::eucd(a, b);

                if(dist<400){
                    if(minDist > dist){
                        minDist = dist;
                        bestCluster = i;
                        bestNeighbor = j;
                    }
                }

            }
        }

        if(bestCluster != -1){
            AllCars[k]->setBelongCluster(result[bestCluster][bestNeighbor]->getCarID());
            result[bestCluster].push_back(AllCars[k]);
            AllCars.erase(AllCars.begin()+k);
            k = k-1;
        }
    }
    return result;

}

std::vector<std::vector<CarInfo*>> Split(std::vector<std::vector<CarInfo*>> &v, int numberOfAllCars){
    std::vector<CarInfo*> split1;
    std::vector<CarInfo*> split2;
    std::vector<std::vector<CarInfo*>> temp;
    int record;
    std::vector<CarInfo*> recordCluster;

    for(int i=0; i<v.size(); i++){
        if(v[i].size()< 2){
            v.erase(v.begin()+i);
            i = i-1;
        }
    }



    for(int i=0; i<v.size(); i++){
        if(v[i].size()>(numberOfAllCars*0.3) && v.size()<7){
            temp = RunSplitKmeans(v[i]);
            split1 = temp[0];
            split2 = temp[1];
            v.erase(v.begin()+i);
            i=i-1;
            v.push_back(split1);
            v.push_back(split2);
        }
    }

    bool flage = true;
    for(int i=0; i<v.size(); i++){
        if(v[i].size()<=4){
            flage = false;
        }
    }

    if(v.size()<4 && flage){
        for(int i=0; i<v.size(); i++){
            if(temp.size()<v[i].size()){
                recordCluster = v[i];
                record = i;
            }
        }
        temp = RunSplitKmeans(recordCluster);
        split1 = temp[0];
        split2 = temp[1];
        v.erase(v.begin()+record);
        v.push_back(split1);
        v.push_back(split2);
    }

    return v;
}


std::vector<std::vector<CarInfo*>> Merge(std::vector<std::vector<CarInfo*>> &v){
    std::vector<std::vector<CarInfo*>> compare;
    std::vector<std::vector<std::vector<double>>> final;

    for(int i=0; i<v.size(); i++){//potential to be merged clusters
        std::vector<CarInfo*> temp;
        temp = v[i];
        if(temp.size()<5){
            compare.push_back(temp);
        }
    }

    int cluster;
    int comcluster;

    for(int i=0; i<v.size(); i++){
        std::vector<std::vector<double>> infos;
        for(int j=0; j<v[i].size(); j++){

            for(int k=0; k<compare.size(); k++){
                int count = 0;
                for(int l=0; l<compare[k].size(); l++){
                    Coord a = v[i][j]->getVehiclePosition();
                    Coord b = compare[k][l]->getVehiclePosition();
                    double dist = Utility::eucd(a, b);
                    std::vector<double> info;
                    if(dist<400){
                        count+=1;
                        cluster = i;
                        comcluster = k;
                        info.push_back(cluster);
                        info.push_back(comcluster);
                        info.push_back(count);
                        infos.push_back(info);
                    }
                }
            }
        }
        final.push_back(infos);
    }

    std::vector<std::vector<double>> data;
    for(int i=0; i<final.size(); i++){
        if(!final[i].empty()){
            data.push_back(final[i].back());
        }
    }

    for(int i=0; i<data.size(); i++){
        if(v[data[i][0]][0]->getCarID() == compare[data[i][1]][0]->getCarID()){
            data.erase(data.begin() + i);
            i = i-1;
        }
    }

    for(int i=0; i<compare.size(); i++){
        double max;
        int merge = -1;
        int bemerged = -1;
        for(int j=0; j<data.size(); j++){
            if(data[j][1] == i && data[j][2]>max){
                max = data[j][2];
                merge = data[j][0];
                bemerged = data[j][1];
            }
        }
        if(merge != -1){
            for(int a = 0; a<v.size(); a++){//delete
                if(v[a][0]->getCarID() == compare[bemerged][0]->getCarID()){
                    v[a].clear();
                }
            }
            for(int b=0; b<compare[bemerged].size(); b++){
                v[merge].push_back(compare[bemerged][b]);
            }

        }
    }

    for(int i=0; i<v.size(); i++){
        if(v[i].empty()){
            v.erase(v.begin()+i);
        }
    }
    return v;
}

std::vector<std::vector<CarInfo*>> MakeClustersByRSU(std::vector<std::vector<CarInfo*>> &vector, std::vector<CarInfo*> const &vehicleTable){
    std::vector<std::vector<CarInfo*>> result;

    std::vector<CarInfo*> potentialVehicles;

    for(int i=0; i<vehicleTable.size(); i++){
        potentialVehicles.push_back(vehicleTable[i]);
    }


    for(int i=0; i<vector.size(); i++){
        for(int j=0;j<vector[i].size(); j++){
            for(int k=0; k<potentialVehicles.size(); k++){
                if(potentialVehicles[k]->getCarID() == vector[i][j]->getCarID()){
                    potentialVehicles.erase(potentialVehicles.begin()+k);
                    k = k-1;
                }
            }
        }
    }

    for(int i=0; i<potentialVehicles.size(); i++){
        bool once = true;
        if(result.size() ==0){
            std::vector<CarInfo*> adding;
            adding.push_back(potentialVehicles[i]);
            result.push_back(adding);
            once = false;
        }else{
            for(int a=0; a<result.size(); a++){
                for(int b=0; b<result[a].size(); b++){
                    if(result[a][b]->getRSUID() == potentialVehicles[i]->getRSUID() && once){
                        result[a].push_back(potentialVehicles[i]);
                        once = false;
                    }
                }
            }
            if(once){
                std::vector<CarInfo*> adding;
                adding.push_back(potentialVehicles[i]);
                result.push_back(adding);
                once = false;
            }
        }
    }

    return result;
}

std::vector<std::vector<CarInfo*>> MakeClusters(std::vector<CarInfo*> &cent, std::vector<CarInfo*> const &vector){
    std::vector<std::vector<CarInfo*>>  result;
    std::vector<CarInfo*> usedVehicles;
    for(int i=0; i<cent.size(); i++){

        if(cent[i]==nullptr || cent[i]->getCarID() == -1){
            cent.erase(cent.begin()+i);
            i = i-1;
        }
    }

    for(int a = 0; a<cent.size(); a++){
        usedVehicles.push_back(cent[a]);
    }


    for(int i=0; i<cent.size(); i++){
        std::vector<CarInfo*> temp;
        temp.push_back(cent[i]);
        usedVehicles.push_back(cent[i]);
        for(int j=0; j<vector.size(); j++){
            Coord a = cent[i]->getVehiclePosition();
            Coord b = vector[j]->getVehiclePosition();
            double dist = Utility::eucd(a,b);
            if(dist<400){

                if(!count(usedVehicles.begin(),usedVehicles.end(),vector[j])){
                    vector[j]->setBelongCluster(temp[0]->getCarID());
                    temp.push_back(vector[j]);
                    usedVehicles.push_back(vector[j]);
                }else{
                    double distance;
                    for(int s = 0; s<result.size(); s++){
                        for(int k=0; k<result[s].size(); k++){
                            if(result[s][k]->getCarID() == vector[j]->getCarID()){
                                Coord a1 = result[s][0]->getVehiclePosition();
                                Coord b1 = b;
                                distance = Utility::eucd(a1,b);
                                if(dist<distance){
                                    vector[j]->setBelongCluster(temp[0]->getCarID());
                                    temp.push_back(vector[j]);
                                    auto iter = std::remove(result[s].begin(), result[s].end(), vector[j]);
                                    result[s].erase(iter,result[s].end());
                                }
                            }
                        }
                    }

                }
            }
        }
        temp[0]->setBelongCluster(temp[0]->getCarID());
        result.push_back(temp);
    }

    return result;
}




CarInfo* FindCenter(std::vector<CarInfo*> &v){
    CarInfo* result = v[0];
    std::vector<double> connectivity;

    connectivity = Utility::SetRelConnectivity(v);


    int best;
    for(int i=0; i<connectivity.size(); i++){//calculate connectivity
        if(best<connectivity[i] && v[i]->getRSUID() != -1){
            best = connectivity[i];
            result = v[i];
        }
    }

    for(int b = 0; b<connectivity.size(); b++){
        if(connectivity[b] == best){
            result = v[b];
            best = 0;
        }
    }


    return result;
}











