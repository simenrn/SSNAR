package no.ntnu.hkm.particlefilter;

import no.ntnu.et.general.*;
import no.ntnu.et.map.*;
import java.util.*;
import static no.ntnu.et.mapping.MappingController.getLineBetweenPoints;
import no.ntnu.tem.robot.Measurement;
import no.ntnu.tem.robot.*;

/**
 * Each particle represents a pose and path hypothesis for the robots.
 * @author Henrik
 */
public class Particle {
    
    private Pose pose;
    private double weight;
    private final Vector<Pose> history;
    private final HashMap<String, Integer> GlobalMap;
    private final HashMap<String, Integer> LocalMap; 
    private final int cellSize;
    private final double convergence;
   
    public Particle(Pose pose, double weight, Robot robot, int cellSize, double convergence){
        this.pose = new Pose(pose);
        this.weight = weight;
        this.history = new Vector<>();
        this.cellSize = cellSize;
        this.GlobalMap = new HashMap<>();
        this.LocalMap = new HashMap<>();
        this.convergence = convergence;
    }
    
    public Particle(Particle p, Robot robot,double convergence){
        this.pose = new Pose(p.getPose());
        this.weight = p.getWeight();
        this.history = p.getHistory();
        this.GlobalMap = p.getGlobalMap();
        this.LocalMap = p.getLocalMap();
        //this.updateQueue = new LinkedList(updateQueue);
        this.cellSize = p.getCellSize();
        this.convergence = convergence;
    }
    
    public double getWeight(){
        return weight;
    }
    
    public void setWeight(double weight){
        this.weight = weight;
    }
    
    public Pose getPose(){
        return pose;
    }
    
    public Position getPosition(){
        return pose.getPosition();
    }
    
    public int getCellSize(){
        return this.cellSize;
    }
    
    public HashMap<String, Integer>  getGlobalMap(){
        HashMap<String, Integer> newMap = new HashMap<>(GlobalMap);
        return newMap;
    } 

    public HashMap<String, Integer> getLocalMap(){
        HashMap<String, Integer> newMap = new HashMap<>(LocalMap);
        return newMap;
    } 

    /**
     * Adds Pose to history if the robot has moved at least 2 cm since last time.
     */
    public void addHistory(){
        Pose newpose = new Pose(pose);
        if(history.isEmpty()){
            history.add(newpose);
        }
        else if((Math.abs(history.lastElement().getPosition().getXValue() - newpose.getPosition().getXValue()) > 10)
               || (Math.abs(history.lastElement().getPosition().getYValue() - newpose.getPosition().getYValue()) > 10)){
            history.add(newpose);
        }
    }
        
    public Vector<Pose> getHistory(){
        Vector<Pose> clonevec = (Vector)history.clone();
        return clonevec;
    }
    
    /**
     * Method called by Pariclefilter to add sensor data to the local map.
     * The position must be translated to the particles point of view.
     * Data is stored in a queue to be integrated into the global map later.
     * @param currentMeasurement 
     */
    public void updateLocalMap(Measurement currentMeasurement){        
        // Update sensor data
        int[] irData = currentMeasurement.getIRdata();
        int[] irheading = currentMeasurement.getIRHeading();        
        
        for (int i = 0; i < 4; i++) {
            int measurementDistance = irData[i];
            boolean trueMeasurement = true;
            if (measurementDistance == 0 || measurementDistance >  40) {
                measurementDistance = 40;
                trueMeasurement = false;                
            }
            
            Angle towerAngle  = new Angle((double)irheading[i]);
            Angle sensorAngle = Angle.sum(towerAngle, pose.getHeading());
            double xOffset = measurementDistance * Math.cos(Math.toRadians(sensorAngle.getValue()));
            double yOffset = measurementDistance * Math.sin(Math.toRadians(sensorAngle.getValue()));
            Position measurementPosition = Position.sum(pose.getPosition(), new Position(xOffset, yOffset));
            MapLocation measurementLocation = findLocationInMap(measurementPosition);
            MapLocation robotlocation = findLocationInMap(pose.getPosition());
            String coord = Integer.toString(measurementLocation.getColumn())+","+Integer.toString(measurementLocation.getRow());

            if(trueMeasurement){
                LocalMap.put(coord,1); //1 is true and -1 is false
            }
            else{
                LocalMap.put(coord,-1);
            }
            // Create a measurements indicating no obstacle in the sensors line of sight
            ArrayList<MapLocation> lineOfSight = getLineBetweenPoints(robotlocation, measurementLocation);
            for (MapLocation location : lineOfSight) {
                coord = Integer.toString(location.getColumn())+","+Integer.toString(location.getRow());
                LocalMap.put(coord,-1);
            }
        }
    }
    
    /**
     * Updates the global map.
     * Called after the robot has moved a given length.
     */
    public void updateGlobalMap(){
        GlobalMap.putAll(LocalMap);
        LocalMap.clear();
    }
 
    /**
     * Compares two maps against each other an returns a comparison score.
     */
    public void MapMatch(){
        //Change types
        MapMatching match = new MapMatching(GlobalMap,LocalMap, convergence); 
        weight = match.correlationScore();
    }

    /**
     * Finds the location in the map grid based on position.
     * Same functionality as found in the GridMap class.
     * @param position
     * @return 
     */
    public MapLocation findLocationInMap(Position position) {
        int row = 0;
        if(position.getYValue() >= 0){
            row = (int)(position.getYValue()/cellSize);
        }else{
            if(position.getYValue()%cellSize == 0){
                row = (int)(position.getYValue()/cellSize);
            }else{
                row = (int)(position.getYValue()/cellSize)-1;
            }
        }
        int column = 0;
        if(position.getXValue() >= 0){
            column = (int)(position.getXValue()/cellSize);
        }else{
            if(position.getXValue()%cellSize == 0){
                column = (int)(position.getXValue()/cellSize);
            }else{
                column = (int)(position.getXValue()/cellSize)-1;
            }
        }
        return new MapLocation(row, column);
    }
    
}
