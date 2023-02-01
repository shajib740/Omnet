// 
//                  Simu5G
//
// Authors: Giovanni Nardini, Giovanni Stea, Antonio Virdis (University of Pisa)
// 
// This file is part of a software released under the license included in file
// "license.pdf". Please read LICENSE and README files before using it.
// The above files and the present reference are part of the software itself, 
// and cannot be removed from it.
//
package simu5g.simulations.NR.cars;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.networklayer.ipv4.RoutingTableRecorder;
import inet.node.inet.AdhocHost;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import inet.node.ethernet.Eth10G;

import simu5g.world.radio.LteChannelControl;
import simu5g.common.carrierAggregation.CarrierAggregation;
import simu5g.nodes.Upf;
import simu5g.common.binder.Binder;
import simu5g.nodes.NR.gNodeB;
import simu5g.nodes.cars.Car;


import org.car2x.veins.nodes.Scenario;
import org.car2x.veins.nodes.RSU;
//import org.car2x.veins.nodes.Car;    // Replaced By import simu5g.nodes.cars.Car; (Line 25 ) //Shajib:Watchout!
//import org.car2x.veins.nodes.CellularTower;
import org.car2x.veins.base.connectionManager.ConnectionManager;
import org.car2x.veins.base.modules.BaseWorldUtility;
import org.car2x.veins.visualizer.roads.RoadsCanvasVisualizer;
import org.car2x.veins.modules.world.annotations.AnnotationManager;


import simu5g.veins_inet.src.veins_inet.VeinsInetManager; //Shajib:Watchout!
//import org.car2x.veins.subprojects.veins_inet.VeinsInetManager;

network Highway
{
    parameters:
        @display("i=background/green");

        double playgroundSizeX @unit(m); // x size of the area the nodes are in (in meters)
        double playgroundSizeY @unit(m); // y size of the area the nodes are in (in meters)
        double playgroundSizeZ @unit(m); // z size of the area the nodes are in (in meters)
        @display("bgb=$playgroundSizeX,$playgroundSizeY");

    submodules:

//        routingRecorder: RoutingTableRecorder {
//            @display("p=50,75;is=s");
//        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=50,125");
            config = xmldoc("demo.xml");
        }

        annotations: AnnotationManager {
            @display("p=270,50");
        }
        rsu[7]: RSU {
            @display("p=330,50;i=veins/sign/yellowdiamond;is=vs");
        }

        baseWroldUtility: BaseWorldUtility {
            parameters:
                playgroundSizeX = 5532m;
                playgroundSizeY = 3869m;
                playgroundSizeZ = 50m;

                @display("p=30,50;i=misc/globe");
        }
        connectionManager: ConnectionManager {
            @display("p=228,175");
        }

        roadsCanvasVisualizer: RoadsCanvasVisualizer {
            @display("p=210,50");
        }

        car[0]: Car {
            @display("p=1759,215");
        }

        //# Veins manager module
        veinsManager: VeinsInetManager {
            @display("p=50,227;is=s");
        }











        //# LTE modules
        channelControl: LteChannelControl {
            @display("p=50,25;is=s");
        }
        binder: Binder {
            @display("p=50,175;is=s");
        }
        carrierAggregation: CarrierAggregation {
            @display("p=50.993748,258.7;is=s");
        }
        server: StandardHost {
            @display("p=660,136;is=n;i=device/server");
        }
        router: Router {
            @display("p=561,135;i=device/smallrouter");
        }
        upf: Upf {
            @display("p=462,136;is=l");
        }
        gNodeB1: gNodeB {
            @display("p=1000,3000;is=vl");
        }
        gNodeB2: gNodeB {
            @display("p=3000,1000;is=vl");
        }



    connections allowunconnected:
        server.pppg++ <--> Eth10G <--> router.pppg++;
        router.pppg++ <--> Eth10G <--> upf.filterGate;
        upf.pppg++ <--> Eth10G <--> gNodeB1.ppp;
        upf.pppg++ <--> Eth10G <--> gNodeB2.ppp;

        //# X2 connections
        gNodeB1.x2++ <--> Eth10G <--> gNodeB2.x2++;
}