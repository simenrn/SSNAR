/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package no.ntnu.tem.application;

import no.ntnu.tem.robot.Robot;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;

/**
 * This class holds a List over all the robots in the system. It holds methods
 * for adding and removing robots as well as searching for special robots.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class RobotController {

    private final ObservableList<Robot> robotList;
    private final boolean debug = false;
    private int idCounter = 0;

    /**
     * Constructor of the class RobotController, starts off with an empty list
     * of robots.
     */
    public RobotController() {
        this.robotList = FXCollections.observableArrayList();
    }

    /**
     * Method that returns the robotlist containing the connected robots
     *
     * @return the list
     */
    public ObservableList<Robot> getRobotList() {
        return robotList;
    }

    /**
     * Method that adds a robot to the system
     *
     * @param address The robots address
     * @param name The robots name
     * @param width The robots physical width (cm)
     * @param length The robots physical length (cm)
     * @param messageDeadline The robots message deadline (min update rate[ms])
     * @param axleOffset Axle offset lengthwise
     * @param towerOffset Tower offset [a,b] a-lengthwise, b-crosswise
     * @param sensorOffset Sensor offset [a,b,c,d,..] from centre (radius)
     * @param irHeading IR heading relative to 0 deg (straight forward)
     * @return true if successful
     */
    public boolean addRobot(int address, String name, int width, int length, int messageDeadline,
            int axleOffset, int[] towerOffset, int[] sensorOffset, int[] irHeading) {
        if (getRobotFromAddress(address) == null) {
            try {
                Robot robot = new Robot(idCounter++, address, name, width, length, messageDeadline,
                        axleOffset, towerOffset, sensorOffset, irHeading);
                robotList.add(robot);
                if (debug) {
                    System.out.println("Robot <" + robot.getName() + "> created!");
                }
                return true;
            } catch (Exception e) {
                e.printStackTrace();
                return false;
            }
        }
        return false;
    }

    public void addRobot(Robot robot) {
        robotList.add(robot);
    }

    /**
     * Method that adds a measurement to a robot
     *
     * @param address the address of the robot
     * @param measuredOrientation measured orientation
     * @param measuredPosition measured position
     * @param irHeading angle of the robot tower
     * @param irData data from IR sensors
     * @return returns true if everything is ok.
     */
    public boolean addMeasurment(int address, int measuredOrientation, int[] measuredPosition, int irHeading, int[] irData) {
        Robot robot = getRobotFromAddress(address);
        if (robot == null) {
            return false;
        }
        if (debug) {
            System.out.println("Robot <" + robot.getName() + "> updated!");
        }
        return robot.addMeasurement(measuredOrientation, measuredPosition, irHeading, irData);
    }

    /**
     * Method that adds a measurement to a robot
     *
     * @param address the address of the robot
     * @param measuredOrientation measured orientation
     * @param measuredPosition measured position
     * @param line Found line
     * @return returns true if everything is ok.
     */
    public boolean addDroneMeasurment(int address, int measuredOrientation, int[] measuredPosition, int[] line) {
        Robot robot = getRobotFromAddress(address);
        if (robot == null) {
            return false;
        }
        if (debug) {
            System.out.println("Robot <" + robot.getName() + "> updated!");
        }
        return robot.addMeasurement(measuredOrientation, measuredPosition, 0, line);
    }
    
            /**
     * Mathod for updating the current battery level of a robot
     * 
     * @param name  of th robot
     * @param level current battery level
     */
    
    public void updateBattery(int adress, int level) {
        Robot robot = getRobotFromAddress(adress);
        
        if (debug) {
            System.out.println("Robot <" + robot.getName() + "> Battery updatedet! to ->" + level);
        }
        robot.updateBattery(level);
    }

    /**
     * Method that updates a robots status to idle
     *
     * @param address the robots
     */
    public void setIdle(int address) {
        Robot robot = getRobotFromAddress(address);
        if (robot != null) {
            robot.setBusy(false);
        }
    }
    


    /**
     * Method that returns the robot, if existing, with the given name
     *
     * @param name the robot name
     * @return the first robot with the given name
     */
    public Robot getRobot(String name) {
        for (Robot r : robotList) {
            if (r.getName().equalsIgnoreCase(name)) {
                return r;
            }
        }
        return null;
    }

    /**
     * Method that returns the robot, if existing, with the given robot id
     *
     * @param robotID the robot id
     * @return the first robot with the given id
     */
    public Robot getRobot(int robotID) {
        for (Robot r : robotList) {
            if (r.getId() == robotID) {
                return r;
            }
        }
        return null;
    }

    public Robot getRobotFromAddress(int address) {
        for (Robot r : robotList) {
            if (r.getAddress() == address) {
                return r;
            }
        }
        return null;
    }

    /**
     * Method that removes the robot, if existing, with the given name
     *
     * @param robotID the robot id.
     * @return true if successful
     */
    public boolean removeRobot(int robotID) {
        for (Robot r : robotList) {
            if (r.getId() == robotID) {
                robotList.remove(r);
                return true;
            }
        }
        return false;
    }

    /**
     * Method that removes the robot, if existing, with the given name
     *
     * @param name the robot name
     * @return true if successful
     */
    public boolean removeRobot(String name) {
        for (Robot r : robotList) {
            if (r.getName().equalsIgnoreCase(name)) {
                robotList.remove(r);
                return true;
            }
        }
        return false;
    }

    /**
     * Mathod for updating the current battery level of a robot
     *
     * @param name of th robot
     * @param level current battery level
     */
    public void updateBattery(String name, int level) {
        Robot robot = getRobot(name);
        if (debug) {
            System.out.println("Robot <" + robot.getName() + "> Battery updatedet!");
        }
        robot.updateBattery(level);
    }
}
