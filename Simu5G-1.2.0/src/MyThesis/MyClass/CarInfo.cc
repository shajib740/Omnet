/*
 * CarInfo.cc
 *
 *  Created on: Dec 10, 2021
 *      Author: veins
 */


#include "CarInfo.h"

Coord CarInfo::getVehiclePosition(){
    return VehiclePosition;
}

int CarInfo::getCarID(){
    return CarID;
}

int CarInfo::getRSUID(){
    return RSUID;
}

int CarInfo::getSourceID(){
    return SourceID;
}

int CarInfo::getCounter(){
    return Counter;
}

simtime_t CarInfo::getTimestamp(){
    return Timestamp;
}

double CarInfo::getEuclidean_Distance(){
    return Euclidean_Distance;
}

double CarInfo::getDrivingDistance(){
    return DrivingDistance;
}

double CarInfo::getStayInRSUTime(){
    return stayInRSUTime;
}

double CarInfo::getVelocity(){
    return velocity;
}

double CarInfo::getAngularDeviation(){
    return angular_deviation;
}

double CarInfo::getDirectionVelocity(){
    return direction_velocity;
}

double CarInfo::getNextDistance(){
    return next_distance;
}

double CarInfo::getRSSI(){
    return rssi;
}

Point CarInfo::getPoint(){
    return point;
}

std::vector<CarInfo*> CarInfo::getNeighbours(){
    return neighbours;
}

int CarInfo::getBelongCluster(){
    return belong_cluster;
}

int CarInfo::getHops(){
    return hops;
}

double CarInfo::getConnectivity(){
    return connectivity;
}


std::vector<std::vector<double>> CarInfo::getDelay(){
    return delay;
}

double CarInfo::getdelay(){
    return delay1;
}

double CarInfo::getClusterConnectivity(){
    return clusterConnectivity;
}








void CarInfo::setVehiclePosition(Coord c){
    VehiclePosition = c;
}

void CarInfo::setCarID(int id){
    CarID = id;
}

void CarInfo::setRSUID(int RSUid){
    RSUID = RSUid;
}

void CarInfo::setSourceID(int sID){
    SourceID = sID;
}

void CarInfo::setCounter(int counter){
    Counter = counter;
}

void CarInfo::setTimestamp(simtime_t time){
    Timestamp = time;
}

void CarInfo::setEuclidean_Distance(double dist){
    Euclidean_Distance = dist;
}

void CarInfo::setDrivingDistance(double Ddist){
    DrivingDistance = Ddist;
}

void CarInfo::setStayInRSUTime(double SIRT){
    stayInRSUTime = SIRT;
}

void CarInfo::setVelocity(double velo){
    velocity = velo;
}

void CarInfo::setAngularDeviation(double ang){
    angular_deviation = ang;
}


void CarInfo::setDirectionVelocity(double DV){
    direction_velocity = DV;
}

void CarInfo::setNextDistance(double distance){
    next_distance = distance;
}

void CarInfo::setRSSI(double rs){
    rssi = rs;
}

void CarInfo::setPoint(Point p){
    point = p;
}

void CarInfo::setNeighbours(std::vector<CarInfo*> n){
    neighbours = n;
}

void CarInfo::setBelongCluster(int b){
    belong_cluster = b;
}

void CarInfo::setHops(int h){
    hops = h;
}

void CarInfo::setConnectivity(double c){
    connectivity = c;
}


void CarInfo::setDelay(std::vector<std::vector<double>>  d){
    delay = d;
}

void CarInfo::setdelay(double  d){
    delay1 = d;
}

void CarInfo::setClusterConnectivity(double  conn){
    clusterConnectivity = conn;
}

