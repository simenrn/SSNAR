/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.navigation;

import no.ntnu.et.map.MapLocation;
import java.util.ArrayList;

/**
 * A class for easily sorting map nodes based og cost.
 * 
 * @author Eirik Thon
 */
public class SortedMapNodeList extends ArrayList<MapNode>{
    public boolean sortedAdd(MapNode node){
        double cost = node.getTotalCost();
        int index = 0;
        for(MapNode otherNode: this){
            double otherCost = otherNode.getTotalCost();
            if(otherCost > cost){
                break;
            }
            index++;
        }
        super.add(index, node);
        return true;
    }
    
    public MapNode containsLocation(MapLocation location){
        for(MapNode node: this){
            if(MapLocation.equals(location, node.getLocation())){
                return node;
            }
        }
        return null;
    }
}
