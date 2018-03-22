/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.navigation;


import java.util.ArrayList;
import no.ntnu.et.general.Position;

/**
 * This class contains data for each robot used for navigation. It has flags to
 * see if a robot is in collision management and contains the way points among
 * other things
 * 
 * @author Eirik Thon
 */

public class NavigationRobot {

    private ArrayList<Position> waypoints;

    private int[] priorityCommand;
    
    private boolean hasPriority;

    private Position lastWaypoint;
    
    private boolean inWallCollision;
    
    private boolean inRobotCollision;
            
    public NavigationRobot(Position lastWaypoint) {
        this.lastWaypoint = lastWaypoint;
        waypoints = new ArrayList<Position>();
        priorityCommand = new int[2];
    }
    
    /**
     * Empty constructor
     */
    NavigationRobot() {}
   
    
    public void addWaypoints(ArrayList<Position> newWaypoints) {
        waypoints.addAll(newWaypoints);
    }

    public void setPriorityCommand(int[] newCommand) {
        priorityCommand = newCommand;
        hasPriority = true;
    }

    Position getNextWaypoint() {
        if(waypoints.size() > 0){
            lastWaypoint = waypoints.remove(0);
            return lastWaypoint;
        }
        return null;
    }
    

    int[] getPriorityCommand() {
        if(hasPriority == true){
            hasPriority = false;
            return priorityCommand;
        }
        return null;
    }
    
    void redoLastWaypoint(){
        waypoints.add(0, lastWaypoint);
    }
    
    boolean hasNewPriorityCommand(){
        return hasPriority;
    }

    public Position getLastWaypoint() {
        return lastWaypoint;
    }
    
    public Position getDestinationWaypoint() {
        return waypoints.get(waypoints.size()-1);
    }
    
    void clearWaypoints(){
        waypoints.clear();
    }
    
    ArrayList<Position> getWaypoints(){
        return waypoints;
    }
    
    boolean hasMoreWaypoints(){
        if(waypoints.size()> 0){
            return true;
        }
        return false;
    }
    
    boolean isInCollisionManagement(){
        return inWallCollision || inRobotCollision;
    }
    
    void setInWallCollision(boolean bool){
        inWallCollision = bool;
    }
    
    boolean getInWallCollision(){
        return inWallCollision;
    }
    
    void setInRobotCollision(boolean bool){
        inRobotCollision = bool;
    }
    
    boolean getInRobotCollision(){
        return inRobotCollision;
    }
}
