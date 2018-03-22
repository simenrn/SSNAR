/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.robot;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;
import no.ntnu.et.general.Position;
import no.ntnu.hkm.particlefilter.Particlefilter;
import org.ejml.simple.SimpleMatrix;

/**
 * This class represents each robot. It holds the robot's parameters and
 * characteristics. The class holds two different objects, IR and a
 * concurrentLinkedQueue of Measurements.
 *
 * The concurrentLinkedQueue "measurements" holds all the Measurements, done by
 * the robot, that is not yet processed by the program.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class Robot {

    private final int id;
    private final String name;
    private final int width, length, axleOffset;
    private final int messageDeadline;
    private final int[] towerOffset, sensorOffset;
    private final IR irSensors;
    private final ConcurrentLinkedQueue<Measurement> measurements;
    private final ConcurrentLinkedQueue<Measurement> slamMeasurements;

    private int[] initialPosition;
    private int[] estimatedPosition;
    private int robotOrientation;
    private boolean homeFlag;
    private final Object homeLock = new Object();
    private boolean goingHomeFlag;
    private final Object goingHomeLock = new Object();
    private boolean busyFlag;
    private final Object busyLock = new Object();
    private int[] destination;
    private int messageCorruptCount = 0;
    private int ValueCorruptCount = 0;
    private int address;
    private boolean connected = false;
    //Particle filter
    Particlefilter p = null;

    //  DOCKING added LMS
    private boolean rangeScanBase;
    private boolean stuckFlag;
    private boolean baseFlag;
    private boolean dockReady;
    private final Object baseLock = new Object();
    private boolean confirmPose;
    private boolean robotAligned;
    public ArrayList<Position> S_REF = new ArrayList<Position>();
    public ArrayList<Position> S_NEW = new ArrayList<Position>();
    private SimpleMatrix realPose;
    private int adjustRobot;
    private int adjustDirection;
    private int backUpDist;
    public SimpleMatrix transMatrix = SimpleMatrix.identity(3);

    private int batteryLevel;
    private final int[] basePosition;
    
    private boolean manualMode = false;

    /**
     * Constructor of the class Robot
     *
     * @param robotID The robots ID
     * @param name The robots name
     * @param width The robots physical width (cm)
     * @param length The robots physical length (cm)
     * @param messageDeadline The robots message deadline (min update rate[ms])
     * @param axleOffset Axle offset lengthwise
     * @param towerOffset Tower offset [a,b] a-lengthwise, b-crosswise
     * @param sensorOffset Sensor offset [a,b,c,d,..] from center (radius)
     * @param irHeading IR heading relative to 0 degrees (straight forward)
     */
    public Robot(int robotID, int address, String name, int width, int length, int messageDeadline,
            int axleOffset, int[] towerOffset, int[] sensorOffset, int[] irHeading) {
        this.id = robotID;
        this.name = name;
        this.width = width;
        this.length = length;
        this.messageDeadline = messageDeadline;
        this.axleOffset = axleOffset;
        this.towerOffset = towerOffset;
        this.sensorOffset = sensorOffset;
        this.irSensors = new IR(irHeading);
        this.batteryLevel = 1023;
        this.measurements = new ConcurrentLinkedQueue<>();
        this.slamMeasurements = new ConcurrentLinkedQueue<>();
        this.address = address;
        this.initialPosition = new int[]{0, 0, 0};
        this.estimatedPosition = new int[]{0, 0};
        this.destination = new int[]{0, 0};

        // Added LMS
        this.goingHomeFlag = false;
        this.baseFlag = false;
        this.dockReady = true;
        this.rangeScanBase = false;
        this.homeFlag = true;
        this.robotAligned = false;
        this.confirmPose = false;
        this.adjustRobot = -1;
        this.adjustDirection = 0;
        this.backUpDist = 0;
        this.basePosition = new int[]{0, 35, 0};
    }

    public void setManualMode(boolean b) {
        manualMode = b;
    }
    
    public boolean isInManualMode() {
        return manualMode;
    }
    
    public boolean isConnected() {
        return connected;
    }

    public void setConnected(boolean connected) {
        this.connected = connected;
    }

    public int getAddress() {
        return address;
    }

    /**
     * Method that sets the robots initial position
     *
     * @param x the robots x-value
     * @param y the robots y-value
     * @param orientation the robots orientation
     */
    public void setInitialPosition(int x, int y, int orientation) {
        initialPosition = new int[]{x, y, orientation};
    }

    /**
     * Method that returns the robots initial position
     *
     * @return the robots initial x, y and orientation
     */
    public int[] getInitialPosition() {
        return this.initialPosition;
    }

    /**
     * Puts a Measurement at the end of the measurement queue. This method is
     * thread safe and will never block. It can throw a NullPointerExeption if
     * some of the values are wrong or null, if not it will return true.
     *
     * @param measuredOrientation Measured theta
     * @param measuredPosition Measured position as an int[] x first, then y
     * @param towerHeading the heading of the first sensor in the ir tower
     * @param irData the ir data that where taken at the same time.
     * @return true if successful, it may throw an exception if some parameters
     * are wrong or null
     */
    public boolean addMeasurement(int measuredOrientation, int[] measuredPosition, int towerHeading, int[] irData) {
        int[] irHeading = new int[irData.length];
        for (int i = 0; i < irData.length; i++) {
            irHeading[i] = (towerHeading + irSensors.getSpreading()[i]) % 360;
        }
        Measurement measurment = new Measurement(measuredOrientation, measuredPosition, irHeading, irData);
        slamMeasurements.offer(measurment); // Separate queue for the boundary robot
        return measurements.offer(measurment);
    }

    /**
     * Returns and removes the oldest measurement done by the robot, this method
     * is thread safe and will return null if there are no more measurements
     * left.
     *
     * @return the oldest measurement done by this robot or null if there are no
     * measurements in the queue.
     */
    public Measurement getMeasurement() {
        Measurement m = measurements.poll();
        //Update particle filter
        if (p != null && m != null) {
            p.updateMeasurement(m);
        }
        return m;
    }
    
    public Measurement getSlamMeasurement() {
        Measurement m = slamMeasurements.poll();
        return m;
    }

    /**
     * Method that returns the robots id
     *
     * @return the id
     */
    public int getId() {
        return id;
    }

    /**
     * Method that returns the robots name
     *
     * @return the name
     */
    public String getName() {
        return name;
    }

    /**
     * Method that returns the robots position
     *
     * @return the position
     */
    public int[] getPosition() {
        int[] position = {estimatedPosition[0], estimatedPosition[1]};
        return position;
    }

    public void setPosition(int[] position) {
        this.estimatedPosition = position;
    }

    /**
     * Method that returns the robots orientation
     *
     * @return the orientation
     */
    public int getRobotOrientation() {
        return robotOrientation;
    }

    public void setRobotOrientation(int robotOrientation) {
        this.robotOrientation = robotOrientation;
    }

    /**
     * Method that returns the robots physical width
     *
     * @return the width
     */
    public int getWidth() {
        return width;
    }

    /**
     * Method that returns the robots physical length
     *
     * @return the length
     */
    public int getLength() {
        return length;
    }

    /**
     * Method that returns the robots tower offset
     *
     * @return the offset
     */
    public int[] getTowerOffset() {
        return towerOffset;
    }

    /**
     * Method that returns the robots sensor offset
     *
     * @return the sensor offset
     */
    public int[] getSensorOffset() {
        return sensorOffset;
    }

    /**
     * Method that returns the robots axle offset
     *
     * @return the axle offset
     */
    public int getAxleOffset() {
        return axleOffset;
    }

    /**
     * Method that returns the robots ir sensors
     *
     * @return the sensors
     */
    public IR getIRSensors() {
        return irSensors;
    }

    /**
     * Method that returns the robots busyflag
     *
     * @return true if the robot is busy
     */
    public boolean isBusy() {
        synchronized (busyLock) {
            return busyFlag;
        }
    }

    /**
     * Method that sets the robots status to busy
     *
     * @param busyFlag true if robot is busy
     */
    public void setBusy(boolean busyFlag) {
        synchronized (busyLock) {
            this.busyFlag = busyFlag;
        }
    }

    /**
     * Method that returns the robots goingHomeFlag
     *
     * @return true if the robot is going home
     */
    public boolean isGoingHome() {
        synchronized (goingHomeLock) {
            return goingHomeFlag;
        }
    }

    /**
     * Method that sets the robot to status going home
     *
     * @param goingHomeFlag true if the robot is going home
     */
    public void setGoingHome(boolean goingHomeFlag) {
        synchronized (goingHomeLock) {
            this.goingHomeFlag = goingHomeFlag;
        }
    }

    public boolean isHome() {
        synchronized (homeLock) {
            return homeFlag;
        }
    }

    /**
     * Method that sets the robot to status going home
     *
     * @param homeFlag true if the robot is going home
     */
    public void setHome(boolean homeFlag) {
        synchronized (homeLock) {
            this.homeFlag = homeFlag;
        }
    }

    /**
     * Method that returns the robots destination
     *
     * @return the destination
     */
    public int[] getDestination() {
        return destination;
    }

    /**
     * Method that updates the robots destination
     *
     * @param destination the robots destination (x,y)
     */
    public void setDestination(int[] destination) {
        this.destination = destination;
    }

    /**
     * Method that increment the message corrupt counter
     *
     */
    public void addMessageCorruptCount() {
        this.messageCorruptCount++;
    }

    /**
     * Method that increment the value corrupt counter
     *
     */
    public void addValueCorruptCount() {
        this.ValueCorruptCount++;
    }

    /**
     * Method that returns the robots number of received corrupt messages/values
     *
     * @return the counter
     */
    public int getCorruptCount() {
        return messageCorruptCount + ValueCorruptCount;
    }

    /**
     * Particlefilter functionality.
     *
     */
    public void setParticleFilter(Particlefilter p) {
        this.p = p;
    }

    /**
     * OBS! Can return null!
     *
     * @return
     */
    public Particlefilter getParticleFilter() {
        return p;
    }

    // Added LMS
    public void setDock(boolean dock) {
        this.dockReady = dock;
    }

    /*
    * Method that sets robot ready for docking
    *
     */
    public boolean isDockReady() {
        return dockReady;
    }

    public void setConfirmPose(boolean confirm) {
        this.confirmPose = confirm;
    }

    public boolean confirmPose() {
        return confirmPose;
    }

    /*Update the battery level*/
    public void updateBattery(int level) {
        this.batteryLevel = level;
        //System.out.println(level);
    }

    public int getBat() {
        return this.batteryLevel;
    }

    /*  Check battery   */
    public void checkBattery() {
        if (this.batteryLevel < 900) {
            this.setGoingHome(true);
            this.setAtBase(false);
            this.setDock(true);
            System.out.println("Low on power, heading home!");
        }
    }

    public boolean isAtBase() {
        synchronized (baseLock) {
            return baseFlag;
        }
    }

    public void setAtBase(boolean baseFlag) {
        synchronized (baseLock) {
            this.baseFlag = baseFlag;
        }
    }

    public boolean isStuck() {
        return this.stuckFlag;
    }

    public void setStuck(boolean stuckFlag) {
        this.stuckFlag = stuckFlag;
    }

    public void setAdjustRobot(int adjustRobot) {
        this.adjustRobot = adjustRobot;
    }

    public int getAdjustRobot() {
        return adjustRobot;
    }

    public void setBackUpDist(int backUpDist) {
        this.backUpDist = backUpDist;
    }

    public int getBackUpDist() {
        return backUpDist;
    }

    public int getAdjustDirection() {
        return adjustDirection;
    }

    public void setAdjustDirection(int adjustDirection) {
        this.adjustDirection = adjustDirection;
    }

    public boolean isRobotAligned() {
        return robotAligned;
    }

    public void setRobotAligned(boolean robotAligned) {
        this.robotAligned = robotAligned;
    }

    public boolean isRangeScanBase() {
        return rangeScanBase;
    }

    public void setRangeScanBase(boolean rangeScanBase) {
        this.rangeScanBase = rangeScanBase;
    }

    public void addToREF(Position in) {
        this.S_REF.add(in);
    }

    public void addToNEW(Position in) {
        this.S_NEW.add(in);
    }

    public void resetNew() {
        System.out.println("NEW-list is cleared");
        this.S_NEW.clear();
    }

    public void resetRef() {
        System.out.println("REF-list is cleared");
        this.S_REF.clear();
    }

    public void setRealPose(SimpleMatrix realPose) {
        this.realPose = realPose;
    }

    public double getRealPose(int row, int col) {
        return realPose.get(row, col);
    }

    public int[] getBasePosition() {
        return this.basePosition;
    }

    public SimpleMatrix getTransMatrix() {
        return transMatrix;
    }

    public void setTransMatrix(SimpleMatrix transMatrix) {
        this.transMatrix = transMatrix;
    }

    public double Bat2Screen() {
        double batLvl = getBat();
        double ToDisp = Math.round(batLvl * 100.0) / 100.0;
        if (!isGoingHome()) {
            if (batLvl < 1023) {
                batLvl = ((getBat() * 2.56) / 1023) * 4.5;
                ToDisp = Math.round(batLvl * 100.0) / 100.0;
            } else {
                if (batLvl < ToDisp) {
                    ToDisp = Math.round(batLvl * 100.0) / 100.0;
                }

            }
        }
        return batLvl;
    }
}
