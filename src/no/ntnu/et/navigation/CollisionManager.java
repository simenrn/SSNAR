/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.navigation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.ConcurrentHashMap;
import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Line;
import no.ntnu.et.general.Position;
import no.ntnu.et.map.GridMap;
import no.ntnu.et.map.MapLocation;
import no.ntnu.et.general.Utilities;
import no.ntnu.et.mapping.MappingController;
import no.ntnu.tem.application.RobotController;

/**
 * This class is used to handle collision avoidance. It detects if a robot is
 * about to collide and sets commands to the corresponding NavigationRobot.
 * 
 * @author Eirik Thon
 */
public class CollisionManager extends Thread {

    private HashMap<String, NavigationRobot> robots;
    
    private ConcurrentHashMap<String, CollisionHandler> handlers;

    private GridMap map;
    
    private boolean paused;
    
    private ArrayList<String> robotNames;
    
    private RobotController robotControl;
    
    private boolean debug = true;
    
    public CollisionManager(GridMap map, RobotController robotController) {
        robotControl = robotController;
        this.map = map;
        robots = new HashMap<String, NavigationRobot>();
        robotNames = new ArrayList();
        paused = true;
        handlers = new ConcurrentHashMap<String, CollisionHandler>();
    }
   
    void addRobot(String name, NavigationRobot navRobot){
        robotNames.add(name);
        robots.put(name, navRobot);
    }
    
    void removeRobot(String name){
        robotNames.remove(name);
        robots.remove(name);
    }
    
    @Override
    public void start(){
        if(!isAlive()){
            super.start();
        }
        paused = false;
    }

    public void pause(){
        paused = true;
        for (ConcurrentHashMap.Entry<String, CollisionHandler> entry : handlers.entrySet()) {
            entry.getValue().pause();
        }
    }
    
    public void unpause(){
        paused = false;
        for (ConcurrentHashMap.Entry<String, CollisionHandler> entry : handlers.entrySet()) {
            entry.getValue().unpause();
        }
    }
    
    public void quit(){
        paused = true;
    }
    
    
    @Override
    public void run() {
        while(true){
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                break;
            }
            if(paused){
                continue;
            }
            for (int i = 0; i < robotNames.size(); i++){
                String robot1Name = robotNames.get(i);
                if(handlers.containsKey(robot1Name)){
                    continue;
                }
                // Robots in manual mode are excluded from collision management
                if (robotControl.getRobot(robot1Name).isInManualMode()) {
                    continue;
                }
                // Check for collisions on current route
                checkWaypoints(robot1Name);
                
                // Check for collisions with walls
                Position robot1Position = new Position(robotControl.getRobot(robot1Name).getPosition());
                MapLocation robotMapLocation = map.findLocationInMap(robot1Position);
                if(map.findCell(robotMapLocation).isRestricted()){
                    CollisionHandler handler = new CollisionHandler();
                    handler.initiateWallCollision(robot1Name);
                    Thread t = new Thread(handler);
                    handlers.put(robot1Name, handler);
                    t.setName("Collision handler "+ Integer.toString(handlers.size()));
                    t.start();
                }
                
                // Check for collisions with orther robots
                else if(robotControl.getRobot(robot1Name).isBusy()){
                    for (int j = 0; j < robotNames.size(); j++){
                        String robot2Name = robotNames.get(j);
                        Position robot2Position = new Position(robotControl.getRobot(robot2Name).getPosition());
                        if(Position.distanceBetween(robot1Position, robot2Position) < 50){
                            double safeDistance = 25;
                            Position robot1Target = robots.get(robot1Name).getLastWaypoint();
                            Line line1 = Line.getLineBetweenPositions(robot1Position, robot1Target);
                            if(line1 != null){
                                if(Utilities.lineCircleIntersection(line1, robot2Position, safeDistance) != 0){
                                    CollisionHandler handler = new CollisionHandler();
                                    if(handlers.containsKey(robot2Name) && handlers.get(robot2Name).getBlockingRobot() == robot1Name){
                                        handler.initiateMutualRobotCollision(robot1Name, robot2Name);
                                    }else{
                                        handler.initiateRobotCollision(robot1Name, robot2Name);
                                    }
                                    Thread t = new Thread(handler);
                                    handlers.put(robot1Name, handler);
                                    t.setName("Collision handler "+ Integer.toString(handlers.size()));
                                    t.start();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    
    private class CollisionHandler implements Runnable{
     
        private boolean paused;
        
        private String name;
        
        private String blockingRobot;
        
        private String type;

        public CollisionHandler() {
            
        }
        
        void pause(){
            paused = true;
        }
        
        void unpause(){
            paused = false;
        }
        
        void initiateWallCollision(String name){
            this.name = name;
            type = "WallCollision";
            blockingRobot = null;
        }
        
        void initiateRobotCollision(String name, String otherName){
            this.name = name;
            blockingRobot = otherName;
            type = "RobotCollision";
        }
        
        void initiateMutualRobotCollision(String name, String otherName){
            this.name = name;
            blockingRobot = otherName;
            type = "MutualRobotCollision";
        }
        
        String getBlockingRobot(){
            return blockingRobot;
        }
        
        @Override
        public void run() {
            if(type == "WallCollision"){
                wallCollision();
            }
            else if (type == "RobotCollision"){
                robotCollision();
            }
            else{
                mutualCollision();
            }
        }
        
        void wallCollision(){
            robots.get(name).setInWallCollision(true);
            robots.get(name).clearWaypoints();
            if(debug){
                System.out.println(name + ": Stuck");
            }
            Position currentPosition = new Position(robotControl.getRobot(name).getPosition());
            Angle currentOrientation = new Angle((double)robotControl.getRobot(name).getRobotOrientation());
            
            int[] command = findWallCollisionCommand(currentPosition, currentOrientation);
            robots.get(name).setPriorityCommand(command);
            
            boolean done = false;
            while(!done){
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    break;
                }
                if(paused){
                    continue;
                }
                try {
                    if(!robotControl.getRobot(name).isBusy()){
                        currentPosition = new Position(robotControl.getRobot(name).getPosition());
                        MapLocation robotLocation = map.findLocationInMap(currentPosition);
                        if(map.findCell(robotLocation).isRestricted()){
                            currentOrientation = new Angle((double)robotControl.getRobot(name).getRobotOrientation());
                            command = findWallCollisionCommand(currentPosition, currentOrientation);
                            robots.get(name).setPriorityCommand(command);
                        }
                        else{
                            if(debug){
                                System.out.println(name + ": No longer stuck");
                            }
                            done = true;
                        }
                    }
                } catch (NullPointerException e) {
                    System.out.println("Exception in Collisionmanager!");
                    System.out.println(e);
                    break;
                }
            }
            robots.get(name).setInWallCollision(false);
            handlers.remove(name);
        }
        
        void robotCollision(){
            //int[] stopCommand = {0, 0};
            // Send the robot's current position to make it stop
            int[] stopCommand = robotControl.getRobot(name).getPosition();
            robots.get(name).setInRobotCollision(true);
            robots.get(name).setPriorityCommand(stopCommand);
            if(debug){
                System.out.println(name + ": Collision course detected. Pausing");
            }
            Position target1 = robots.get(name).getLastWaypoint();
            Position robot1Position = new Position(robotControl.getRobot(name).getPosition());
            Line line1 = Line.getLineBetweenPositions(robot1Position, target1);
            boolean done = false;
            double safeDistance = 25;
            int counter = 0;
            while(!done){
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    break;
                }
                if(paused){
                    continue;
                }
                if(counter > 20){
                    if(debug){
                        System.out.println(name + ": Collision handling timeout! Deleting current task");
                    }
                    robots.get(name).clearWaypoints();
                    int[] destination = {(int)Math.round(robot1Position.getXValue()), (int)Math.round(robot1Position.getYValue())};
                    robotControl.getRobot(name).setDestination(destination);
                    break;
                }
                Position robot2Position = new Position(robotControl.getRobot(blockingRobot).getPosition());
                if(Utilities.lineCircleIntersection(line1, robot2Position, safeDistance) == 0){
                    if(debug){
                        System.out.println(name + ": Collision avoided. Resuming");
                    }
                    robots.get(name).redoLastWaypoint();
                    done = true;
                }
                counter++;
            }
            robots.get(name).setInRobotCollision(false);
            handlers.remove(name);
        }
        
        void mutualCollision(){
            if(debug){
                System.out.println(name + ": Double collision course detected");
            }
            robots.get(name).setInRobotCollision(true);
            Position robot1Position = new Position(robotControl.getRobot(name).getPosition());
            Angle robot1Heading = new Angle((double)robotControl.getRobot(name).getRobotOrientation());
            Position robot2Position = new Position(robotControl.getRobot(blockingRobot).getPosition());
            int[] command = findRobotCollisionCommand(robot1Position, robot1Heading, robot2Position);
            if(command != null){
                if(debug){
                    System.out.println(name + ": Stepping aside");
                }
                robots.get(name).setPriorityCommand(command);
                boolean done = false;
                int counter = 0;
                while(!done){
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        break;
                    }
                    if(paused){
                        continue;
                    }
                    if(counter > 20){
                        if(debug){
                            System.out.println(name + ": Collision handling timeout! Deleting current task");
                        }
                        robots.get(name).clearWaypoints();
                        robot1Position = new Position(robotControl.getRobot(name).getPosition());
                        int[] destination = {(int)Math.round(robot1Position.getXValue()), (int)Math.round(robot1Position.getYValue())};
                        robotControl.getRobot(name).setDestination(destination);
                        break;
                    }
                    if(!handlers.containsKey(blockingRobot)){
                        done = true;
                        if(debug){
                            System.out.println(name + ": Collision avoided. Resuming");
                        }
                        robots.get(name).redoLastWaypoint();
                    }
                    counter++;
                }
            }
            else{
                if(debug){
                    System.out.println(name + ": Unable to step aside. Deleting current task");
                }
                robots.get(name).clearWaypoints();
                robot1Position = new Position(robotControl.getRobot(name).getPosition());
                int[] destination = {(int)Math.round(robot1Position.getXValue()), (int)Math.round(robot1Position.getYValue())};
                robotControl.getRobot(name).setDestination(destination);
            }
            robots.get(name).setInRobotCollision(false);
            handlers.remove(name);
        }
    }
    
    void checkWaypoints(String robotName){
        /*
        Mulig problem med at waypoints forsvinner etterhvert som de blir nådd
        */
        ArrayList<Position> waypoints = robots.get(robotName).getWaypoints();
        for(int i = 0; i < waypoints.size(); i++){
            MapLocation location = null;
            try{
                location = map.findLocationInMap(waypoints.get(i));
            }
            catch(NullPointerException e){
                continue;
            }
            // If a waypoint is restricted, clear all waypoints so that a new target and path is found in the next iteration
            if(map.findCell(location).isRestricted() || map.findCell(location).isOccupied()){
                if(debug){
                    System.out.println(robotName+ ": Found intersection along current path. Stop");
                }
                robots.get(robotName).clearWaypoints();
                //int[] command = {0 , 0};
                int[] command = robotControl.getRobot(robotName).getPosition();
                robots.get(robotName).setPriorityCommand(command);
                robotControl.getRobot(robotName).setDestination(robotControl.getRobot(robotName).getPosition());
                break;
            }
        }
    }
    
    int[] findWallCollisionCommand(Position currentPosition, Angle currentOrientation){
        Position offset = Utilities.polar2cart(currentOrientation, -10);
        Position rearPosition = Position.sum(currentPosition, offset);
        MapLocation rearMapLocation = map.findLocationInMap(rearPosition);
        // Check if it is possible to just reverse 10 cm 
        if(map.findCell(rearMapLocation) != null) {
            if(map.findCell(rearMapLocation).isWeaklyTargetable()){
                //int[] command = {0 , -10};
                int[] command = {(int) rearPosition.getXValue(), (int) rearPosition.getYValue()};
                return command;
            }
        }

        // Send the robot to the nearest unrestricted (and weakly unrestricted) location in the map
        MapLocation robotLocation = map.findLocationInMap(currentPosition);
        MapLocation unrestrictedMapLocation = PathPlanningFunctions.findNearestFreeCell(map, robotLocation);
        Position unrestrictedPosition = map.mapLocation2Position(unrestrictedMapLocation);
        //int[] command = NavigationController.findCommandToTargetPoint(unrestrictedPosition, currentPosition, (int)Math.round(currentOrientation.getValue()));
        int[] command = {(int) unrestrictedPosition.getXValue(), (int) unrestrictedPosition.getYValue() };
        return command;
    }
    
    int[] findRobotCollisionCommand(Position robot1Position, Angle robot1Heading, Position robot2Position){
        Angle angleBetweenRobots = Position.angleBetween(robot1Position, robot2Position);
        MapLocation robot1Location = map.findLocationInMap(robot1Position);
        
        Angle testAngle = Angle.sum(angleBetweenRobots, new Angle(90));
        Position offset = Utilities.polar2cart(testAngle, 40);
        Position testPosition = Position.sum(robot1Position, offset);
        MapLocation testMapLocation = map.findLocationInMap(testPosition);
        ArrayList<MapLocation> line = MappingController.getLineBetweenPoints(robot1Location, testMapLocation);
        boolean success = true;
        for(MapLocation location: line){
            if(map.findCell(location) != null){
                if(map.findCell(location).isRestricted()) {
                    success = false;
                }
            }
        }
        if(success){
            //int[] command = {(int)Math.round(Angle.difference(testAngle, robot1Heading)), 40};
            int[] command = {(int) testPosition.getXValue(), (int) testPosition.getYValue()};
            return command;
        }
        
        testAngle = Angle.sum(angleBetweenRobots, new Angle(-90));
        offset = Utilities.polar2cart(testAngle, 40);
        testPosition = Position.sum(robot1Position, offset);
        testMapLocation = map.findLocationInMap(testPosition);
        for(MapLocation location: line){
            if(map.findCell(location) != null){
                if(map.findCell(location).isRestricted()) {
                    success = false;
                }
            }
        }
        if(success){
            //int[] command = {-(int)Math.round(Angle.difference(testAngle, robot1Heading)), 40};
            int[] command = {(int) testPosition.getXValue(), (int) testPosition.getYValue()};
            return command;
        }
        
        return null;
    }
}
