/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.navigation;

import no.ntnu.et.map.MapLocation;

/**
 * This class represents a node in a graph for A* planning
 * 
 * @author Eirik Thon
 */
public class MapNode {
    private MapLocation location;
    private double heuristicCost;
    private double traversedCost;
    private MapNode previous;
    boolean gay;

    public MapNode(MapLocation location, double gCost, double hCost, MapNode previous) {
        this.location = location;
        this.traversedCost = gCost;
        this.heuristicCost = hCost;
        this.previous = previous;
        
    }

    public MapLocation getLocation() {
        return location;
    }

    public double getHeuristicCost() {
        return heuristicCost;
    }

    public double getTraversedCost() {
        return traversedCost;
    }
    
    public double getTotalCost() {
        return traversedCost + heuristicCost;
    }
    
    public MapNode getPrevious(){
        return previous;
    }
    
    public static boolean equals(MapNode node1, MapNode node2){
        return MapLocation.equals(node1.location, node2.location);
    }
    
    public void setPrevious(MapNode newPrev){
        previous = newPrev;
    }

    public void setHeuristicCost(double heuristicCost) {
        this.heuristicCost = heuristicCost;
    }

    public void setTraversedCost(double traversedCost) {
        this.traversedCost = traversedCost;
    }
}
