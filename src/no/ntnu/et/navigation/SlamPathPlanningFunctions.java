/**
 * This code is written as a part of a Master Thesis
 * the fall of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.et.navigation;

import java.util.ArrayList;
import no.ntnu.et.general.Position;
import no.ntnu.et.map.GridMap;
import no.ntnu.et.map.MapLocation;
import no.ntnu.et.mapping.MappingController;

/**
 * This class contains static functions used by the SLAMrobot for path planning.
 * The algorithms are based on the lists of obstructed and frontier cells.
 * 
 * @author geirhei
 */
public class SlamPathPlanningFunctions {
    
    static ArrayList<MapLocation> findPath(GridMap map, MapLocation startLocation , MapLocation targetLocation){
        MapNode currentNode = new MapNode(MapLocation.copy(startLocation), 0, MapLocation.distance(startLocation, targetLocation)*map.getCellSize(), null);
        SortedMapNodeList closedSet = new SortedMapNodeList();
        SortedMapNodeList openSet = new SortedMapNodeList();
        openSet.add(currentNode);
        while(openSet.size() > 0){
            currentNode = openSet.remove(0);
            closedSet.add(currentNode);
            
            // Test if finished
            if(MapLocation.equals(currentNode.getLocation(), targetLocation)){
                ArrayList<MapLocation> path = constructPath(currentNode);
                return path;
            }
            
            // Create successors
            ArrayList<MapNode> successors = generateSuccessors(map, currentNode, targetLocation);
            
            for(MapNode successor: successors){
                MapNode existingSuccessor = closedSet.containsLocation(successor.getLocation());
                if(existingSuccessor != null){
                    continue;
                }
                existingSuccessor = openSet.containsLocation(successor.getLocation());
                if(existingSuccessor == null){
                    openSet.sortedAdd(successor);
                }
                else if (existingSuccessor.getTraversedCost() > successor.getTraversedCost()){
                    existingSuccessor.setPrevious(currentNode);
                    existingSuccessor.setTraversedCost(successor.getTraversedCost());
                }
            }
        }
        return null;
    }
    
    static private ArrayList<MapNode> generateSuccessors(GridMap map, MapNode parentNode, MapLocation target){
        ArrayList<MapNode> successors = new ArrayList<MapNode>();
        double gCost = parentNode.getTraversedCost();
        ArrayList<MapLocation> diagonalNeighbors = map.findDiagonalNeighborCells(parentNode.getLocation());
        for(MapLocation neighbor: diagonalNeighbors){
            if(map.findCell(neighbor).isWeaklyTargetable()){
                if(map.findCell(neighbor).isWeaklyRestricted()){
                    successors.add(new MapNode(neighbor, gCost+1.415*10, MapLocation.distance(neighbor, target)*map.getCellSize(), parentNode));
                }else{
                    successors.add(new MapNode(neighbor, gCost+1.415, MapLocation.distance(neighbor, target)*map.getCellSize(), parentNode));
                }
            }
        }
        ArrayList<MapLocation> directNeighbors = map.findDirectNeighborCells(parentNode.getLocation());
        for(MapLocation neighbor: directNeighbors){
            if(map.findCell(neighbor).isWeaklyTargetable()){
                if(map.findCell(neighbor).isWeaklyRestricted()){
                    successors.add(new MapNode(neighbor, gCost+10, MapLocation.distance(neighbor, target)*map.getCellSize(), parentNode));
                }else{
                    successors.add(new MapNode(neighbor, gCost+1, MapLocation.distance(neighbor, target)*map.getCellSize(), parentNode));
                }
            }
        }
        return successors;
    }
    
    static private ArrayList<MapLocation> constructPath(MapNode lastNode){
        ArrayList<MapLocation> path = new ArrayList<MapLocation>();
        MapNode current = lastNode.getPrevious();
        path.add(lastNode.getLocation());
        while (current != null){
            path.add(current.getLocation());
            //map.findCell(current.getLocation()).setPath();
            current = current.getPrevious();
        }
        return path;
    }
    
    static MapLocation findNearestFreeCell(GridMap map, MapLocation startLocation){
        ArrayList<MapLocation> closedSet = new ArrayList<MapLocation>();
        ArrayList<MapLocation> openSet = new ArrayList<MapLocation>();
        openSet.add(startLocation);
        MapLocation currentLocation;
        while(openSet.size() > 0){
            currentLocation = openSet.remove(0);
            closedSet.add(currentLocation);
            
            // Test if finished
            if(map.findCell(currentLocation).isFreelyTargetable()){
                return currentLocation;
            }
            // Create successors
            ArrayList<MapLocation> successors = generateSuccessorsBFS(map, currentLocation);
            
            for(MapLocation successor: successors){
                if(closedSet.contains(successor)){
                    continue;
                }
                if(!openSet.contains(successor)){
                    openSet.add(successor);
                }
            }
        }
        return null;
    }
    
    static private ArrayList<MapLocation> generateSuccessorsBFS(GridMap map, MapLocation parentLocation){
        ArrayList<MapLocation> successors = new ArrayList<MapLocation>();
        ArrayList<MapLocation> diagonalNeighbors = map.findDiagonalNeighborCells(parentLocation);
        for(MapLocation neighbor: diagonalNeighbors){
            if(map.findCell(neighbor).isOccupied()){
                continue;
            }
            successors.add(neighbor);
        }
        ArrayList<MapLocation> directNeighbors = map.findDirectNeighborCells(parentLocation);
        for(MapLocation neighbor: directNeighbors){
            if(map.findCell(neighbor).isOccupied()){
                continue;
            }
            successors.add(neighbor);
        }
        return successors;
    }
    
    static ArrayList<Position> generateWaypoints(GridMap map, ArrayList<MapLocation> path) {
        ArrayList<Position> waypoints = new ArrayList<Position>();
        MapLocation currentWaypoint = path.get(0);
        MapLocation next = path.get(0);
        MapLocation finalLocation = path.get(path.size()-1);
        double threshold = 15;
        double fit = 0;
        int currentIndex = 0;
        int counter = currentIndex+1;
        while(!MapLocation.equals(next, finalLocation)){
            ArrayList<MapLocation> straightLine = new ArrayList<MapLocation> ();
            while(counter < path.size()){
                next = path.get(counter);
                straightLine = MappingController.getLineBetweenPoints(currentWaypoint, next);
                ArrayList<MapLocation> pathPart = new ArrayList();
                for(int i = currentIndex; i < counter+1; i++){
                    pathPart.add(path.get(i));
                }
                fit = coumputeFit(straightLine, pathPart, map.getCellSize());
                if (fit > threshold){
                    break;
                }
                counter++;
            }
            currentIndex = counter;
            currentWaypoint = next;
            waypoints.add(map.mapLocation2Position(currentWaypoint));
        }
        
        // Due to how this method works the second last waypoint can be very close to the final target.
        // If this is the case the second last waypoint is removed in order to aviod sending unnecessary commands to the robot
        if(waypoints.size() > 1){
            if(Position.distanceBetween(map.mapLocation2Position(finalLocation), waypoints.get(waypoints.size()-2)) < 10){
                waypoints.remove(waypoints.size()-2);
            }
        }
        return waypoints;
    }
    
    static private double coumputeFit(ArrayList<MapLocation> list1, ArrayList<MapLocation> list2, int cellSize){
        double fit = 0;
        int end = list1.size();
        int index = 0;
        while (index < end){
            fit += MapLocation.distance(list1.get(index), list2.get(index))*cellSize;
            index++;
        }
        return fit;
    }
}