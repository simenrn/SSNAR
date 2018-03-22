/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.simulator;

import no.ntnu.et.general.Utilities;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Position;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;
import no.ntnu.et.general.Line;

/**
 * This class represents the environment of the simulator. It contains all the
 * features and all the robots. The scale of the world is in cm, meaning that
 * if the width is 600, it represents an environment that is 6 meters across
 * 
 * @author Eirik Thon
 */
public class SimWorld {
    private int width;
    private int height;
    ArrayList<Feature> features;
    ArrayList<Integer> robotIDs;
    private ArrayList<String[]> possibleRobotNames;
    private int addressCounter = 0;
    HashMap<Integer, SimRobot> robots;
    private Integer ArduinoCounter = 0;
    private Integer NXTCounter = 0;
    private Integer AVRCounter = 0;
    private boolean SLAMadded = false;
    private boolean Droneadded = false;

    /**
     * Default constructor
     */
    public SimWorld() {
        width = 0;
        height = 0;
        features = new ArrayList<Feature>(0);
        robots = new HashMap<>();
        robotIDs = new ArrayList<Integer> (0);
    }
    
    /**
     * Returns the height of the map
     * @return height
     */
    int getMapHeight() {
        return height;
    }
    
    /**
     * Returns the width of the map
     * @return width
     */
    int getMapWidth() {
        return width;
    }
    
    
    /**
    * Creates and returns a copy of the features in the SimWorld.
    * This function is safe to call concurrently.
    * @return ArrayList<String> robotNamesCopy
    */
    ArrayList<Feature> getFeatures() {
        return features;   
    }
    
    ArrayList<SimRobot> getRobots() {
        ArrayList<SimRobot> robotlist = new ArrayList();
        robotlist.addAll(robots.values());
        return robotlist;
    }

    SimRobot createRobot(Pose initialPose, int RobotType) {
       String[] identification = null;
        if(!Droneadded && RobotType == 1){
            identification = new String[]{"1", "Drone"};
            Droneadded = true;
        }
        else if(Droneadded && RobotType == 1){
            System.out.println("Only one Drone supported");
            return null;
        }
        if(RobotType == 2){
            ArduinoCounter++;
            Integer id = (ArduinoCounter < 2) ? 2 : 5; // 2 5 and 8
            if(ArduinoCounter == 3)id =8;
            if(ArduinoCounter > 3)id = 11;
            identification = new String[]{id.toString() , "Arduino"+ArduinoCounter.toString()};
        }
        if(RobotType == 3){
            NXTCounter++;
            Integer id = 3*NXTCounter; //3 6 and 9
            identification = new String[]{id.toString(), "NXT"+NXTCounter.toString()};
        }
        if(RobotType == 4){
            AVRCounter++;
            Integer id = 4*AVRCounter; //4 and 8
            identification = new String[]{id.toString(), "AVR"+AVRCounter.toString()};
        }
        if(!SLAMadded && RobotType == 5){
            identification = new String[]{"7", "SLAM"};
            SLAMadded = true;
        }
        else if(SLAMadded && RobotType == 5){
            System.out.println("Only one SLAMrobot supported");
            return null;
        }
        int id = Integer.parseInt(identification[0]);
        if(id > 9){
            System.out.println("Cant have more of this unit!");
            return null;
        }
        String name = identification[1];
        SimRobot newRobot;
        if(name.equals("Drone")){
            newRobot = new Drone(this, initialPose, "Drone", id, addressCounter++);
            System.out.println("Drone created");
        } else if (name.equals("SLAM")) {
            newRobot = new SlamRobot(this, initialPose, "SLAM", id, addressCounter++);
            System.out.println("SLAMrobot created");
        } else {
            newRobot = new SimRobot(this, initialPose, name, id, addressCounter++);
            System.out.println("SimRobot created");
        }
        robotIDs.add(id);
        robots.put(id,newRobot);  
        return newRobot;
    }
    
    
    boolean checkIfPositionIsFree(Position center, int ignoredRobotId) {
        double collisionSize = 5; // 5 cm
        for(int i = 0; i < features.size(); i++){
            // Create vector from feature start to position
            Feature feature = features.get(i);
            Position projection = Utilities.getOrthogonalProjection(center, Line.convertFeatureToLine(feature));
            if(projection == null){
                continue;
            }
            // Check for collision
            if (Position.distanceBetween(center, projection) < collisionSize) {
                return false;
            }
        }
        for (int i = 0; i < robotIDs.size(); i++){
            if(robotIDs.get(i) == ignoredRobotId){
                continue;
            }
            Position robotPosition = robots.get(robotIDs.get(i)).getPose().getPosition();
            if(Position.distanceBetween(center, robotPosition) < collisionSize*2){
                return false;
            }
        }
        return true;
    }
    
    double findNearestIntersection(Line line, int ignoredRobotId){
        double shortestMeasurement = Double.POSITIVE_INFINITY;
        for(int i = 0; i < features.size(); i++){
            Line featureLine = Line.convertFeatureToLine(features.get(i));
            double newIRMeasurement = Utilities.lineLineIntersection(line, featureLine);
            // Add only the measurement of the nearest feature
            if (newIRMeasurement != 0 && newIRMeasurement < shortestMeasurement) {
                shortestMeasurement = newIRMeasurement;
            }
        }
        for (int i = 0; i < robotIDs.size(); i++){
            if(robotIDs.get(i) == ignoredRobotId){
                continue;
            }
            double newIRMeasurement = Utilities.lineCircleIntersection(line, robots.get( robotIDs.get(i) ).getPose().getPosition(), 5);
            // Add only the measurement of the nearest feature
            if (newIRMeasurement != 0 && newIRMeasurement < shortestMeasurement) {
                shortestMeasurement = newIRMeasurement;
            }
        }
        if(shortestMeasurement == Double.POSITIVE_INFINITY){
            shortestMeasurement = 0;
        }
        return shortestMeasurement;
    }
    
    SimRobot getRobot(int id) {
        return robots.get(id);
    }
    SimRobot getRobot(String name) {
        for (Map.Entry<Integer, SimRobot> e : robots.entrySet()) {
            if(e.getValue().getName().equals(name)) return e.getValue();
        }    
        return null;
    }
    
    /**
     * Creates and returns a copy of the robot names stored in the SimWorld.
     * This function is safe to call concurrently.
     * @return ArrayList<String> robotNamesCopy
     */
    ArrayList<Integer> getRobotIDs() {
        return robotIDs;
    }
    
    /**
     * Sets the map size and adds all features in the map based upon data found
     * in the text file given by the input parameter filename
     * @param filename The string filename specifies the map file
     */
    void initMap(String filename){
        features = new ArrayList<Feature>();
        try{
            File map = new File(filename);
            Scanner sc = new Scanner(map);
            
            initWorldSize(sc.next());
            while(sc.hasNext()){
                String featurePositionString = sc.next();
                String[] stringParts = featurePositionString.split(":");
                Position start = Utilities.string2Position(stringParts[0]);
                Position end = Utilities.string2Position(stringParts[1]);
                Feature newFeature = new Feature(start, end);
                features.add(newFeature);
            }
            sc.close();
            addBorderFeatures();
        }
        catch(FileNotFoundException e){
            System.out.println("File not found");
        }        
    }
    
    /**
     * Reads the contents of the input parameter size and updates the width
     * and height of the world based upon this
     * @param size Sting on the format "width:height"
     */
    private void initWorldSize(String size){
        String[] stringParts = size.split(":");
        width = Integer.parseInt(stringParts[0]);
        height = Integer.parseInt(stringParts[1]);
    }
    
    /**
     * Creates features that lies along the edges of the map and adds them to
     * the other features
     */
    private void addBorderFeatures(){
        Feature leftBorder = new Feature(0,0,0,height);
        features.add(leftBorder);
        Feature rightBorder = new Feature(width,0,width,height);
        features.add(rightBorder);
        Feature topBorder = new Feature(0,0,width,0);
        features.add(topBorder);
        Feature bottomBorder = new Feature(0,height,width,height);
        features.add(bottomBorder);
    }
}
