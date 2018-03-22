/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.simulator;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import no.ntnu.et.general.Utilities;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Position;
import no.ntnu.et.general.Line;
import no.ntnu.tem.communication.DroneUpdateMessage;
import no.ntnu.tem.communication.HandshakeMessage;
import no.ntnu.tem.communication.UpdateMessage;

/**
 * This class represents the virtual robots in the simulator. It has private
 * variables to represent the state of the robots such as the pose The robots
 * behavior is created by calling methods in this class such as moveRobot().
 *
 * @author Eirik Thon
 */
public class SimRobot {

    private SimWorld world;
    public Pose pose;
    public Pose estimatedPose;
    final private Pose initialPose;
    private String name;
    private int id;
    public double measuredDistance;
    public double measuredRotation;
    public double targetDistance;
    public double targetRotation;
    public int rotationDirection;
    public int movementDirection;
    public boolean rotationFinished;
    public boolean translationFinished;
    private Angle towerAngle;
    final private double turnSpeed = 0.5;
    //final private double turnSpeed = 1;
    private double moveSpeed;
    final private double towerSpeed = 0.25; // = 5 deg resolution
    //final private double towerSpeed = 0.05; // = 1 deg resolution
    private int towerDirection;
    final private Object movementLock = new Object();
    final private double maxVisualLength = 80;
    final private double minVisualLength = 10;
    private double[] lastIrMeasurement;
    private Position targetPosition;
    private int diameter = 10;
    private final int address;
    
    private final double[] distances;

    /**
     * Constructor for Robot.
     *
     * @param initialPose
     * @param name
     * @param id
     */
    public SimRobot(SimWorld world, Pose initialPose, String name, int id, int address) {
        this.address = address;
        this.world = world;
        towerDirection = 1;
        towerAngle = new Angle(0);
        this.name = name;
        this.id = id;
        pose = initialPose;
        this.initialPose = Pose.copy(initialPose);
        estimatedPose = new Pose(0, 0, 0);
        targetDistance = 0;
        targetRotation = 0;
        measuredDistance = 0;
        measuredRotation = 0;
        lastIrMeasurement = new double[4];
        rotationDirection = 1;
        movementDirection = 1;
        rotationFinished = true;
        translationFinished = true;
        targetPosition = Position.copy(pose.getPosition());
        
        distances = new double[360];
        for (int i = 0; i < distances.length; i++) {
            distances[i] = Double.POSITIVE_INFINITY;
        }
        
        if (name.equals("SLAM")) {
            moveSpeed = 0.05;
        } else {
            moveSpeed = 0.1;
        }
    }

    void updateDistances() {
        double towerHeading = towerAngle.getValue();
        double robotHeading = estimatedPose.getHeading().getValue();
        for (int i = 0; i < lastIrMeasurement.length; i++) {
            int currentAngle = (int) (robotHeading + towerHeading + (i-1) * 90);
            if (currentAngle >= 360) {
                currentAngle -= 360;
            } else if (currentAngle < 0) {
                currentAngle += 360;
            }
            if (lastIrMeasurement[i] >= 0) {
                distances[currentAngle] = Double.POSITIVE_INFINITY;
            } else {
                distances[currentAngle] = lastIrMeasurement[i];
            }
        }
    }
    
    void stop() {
        setMovement(0, 0);
    }
    
    double[] getDistances() {
        return distances;
    }
    
    double getMaxSensorDistance() {
        return maxVisualLength;
    }

    Angle getTowerAngle() {
        return towerAngle;
    }

    /**
     * Returns the ID of the robot
     *
     * @return integer
     */
    int getId() {
        return id;
    }

    int getAddress() {
        return address;
    }

    /**
     * Returns the name of the robot
     *
     * @return String
     */
    String getName() {
        return name;
    }
    
    /**
     * Sets the target rotation and target heading. Also resets the measured
     * rotation and distance.
     *
     * @param x
     * @param y
     */
    void setTarget(double x, double y) {
        synchronized (movementLock) {
            targetPosition = new Position(x, y);
            targetDistance = Position.distanceBetween(targetPosition, pose.getPosition()); //cm
            
            Angle targetAngle = Position.angleBetween(pose.getPosition(), targetPosition); //deg
            targetRotation = Angle.difference(pose.getHeading(), targetAngle); //deg
            // Determine if robot should rotate left or right
            if ((pose.getHeading().getValue() - targetAngle.getValue() + 360) % 360 > 180) {
                rotationDirection = 1;
            } else {
                rotationDirection = -1;
            }
            
            measuredRotation = 0;
            measuredDistance = 0;
            movementDirection = 1; // cludged
            rotationFinished = false;
            translationFinished = false;
        }
    }
    
    /**
     * Specify an angle for the robot to rotate and a distance to translate
     * 
     * @param theta number of degrees to rotate
     * @param distance number of cm to translate
     */
    void setMovement(int thetaTarget, double distance) {
        synchronized (movementLock) {
            Angle targetAngle = new Angle(thetaTarget);
            Position offset = Utilities.polar2cart(targetAngle, distance);
            targetRotation = Angle.difference(pose.getHeading(), targetAngle); //deg
            targetDistance = distance;
            measuredRotation = 0;
            measuredDistance = 0;
            rotationDirection = (int) Math.signum(thetaTarget);
            movementDirection = (int) Math.signum(distance);
            
            targetPosition = Position.sum(pose.getPosition(), offset);
            rotationFinished = false;
            translationFinished = false;
        }
    }

    boolean isTranslationFinished() {
        return translationFinished;
    }
    
    /**
     * Creates and returns a copy of the pose
     *
     * @return
     */
    Pose getPose() {
        return pose;
    }

    Pose getInitialPose() {
        return initialPose;
    }

    /**
     * Creates and returns a copy of the estimated pose
     *
     * @return
     */
    Pose getEstimatedPose() {
        return estimatedPose;
    }

    Position getTargetPosition() {
        return targetPosition;
    }

    /**
     * Turns the tower 5 degrees within a 90 degree angle.
     */
    void turnTower() {
        int clockWise = -1;
        int counterClockWise = 1;
        if (towerAngle.getValue() < towerSpeed && towerDirection == clockWise) {
            towerDirection = counterClockWise;
        } else if (towerAngle.getValue() >= 90 && towerDirection == counterClockWise) {
            towerDirection = clockWise;
        }
        towerAngle.add(towerDirection * towerSpeed);
    }

    /**
     * Creates a package of all measurements with a timestamp. The package is
     * sent to the application and also stored in the measurement history of the
     * robot.
     */
    int[] createMeasurement() {
        int[] measurement = new int[8];
        measurement[0] = (int) Math.round(estimatedPose.getPosition().getXValue());
        measurement[1] = (int) Math.round(estimatedPose.getPosition().getYValue());
        measurement[2] = (int) Math.round(estimatedPose.getHeading().getValue());
        measurement[3] = (int) Math.round(towerAngle.getValue());
        //System.out.println("Angle: " + measurement[3]);
        for (int i = 4; i < 8; i++) {
            measurement[i] = (int) Math.round(lastIrMeasurement[i - 4]);
        }
        return measurement;
    }

    /**
     * Rotate or move the robot a little distance towards the target position.
     * The robot will not move if the movement leads to a collision between any
     * of the features in "allFeatures". If the destination is reached the robot
     * will not move. The estimated pose is also updated and the value of
     * "noise" is added to the estimate.
     *
     * @param allFeatures ArrayList<Feature>
     * @param noise double
     */
    boolean moveRobot(double noise) {
        synchronized (movementLock) {
            if (!rotationFinished) {
                if (Math.abs(measuredRotation) >= Math.abs(targetRotation)) {
                    measuredRotation = 0;
                    rotationFinished = true;
                }
                pose.rotate(new Angle(turnSpeed * rotationDirection));
                estimatedPose.rotate(new Angle((turnSpeed + noise) * rotationDirection));
                measuredRotation += (turnSpeed + noise) * rotationDirection;
            } else if (!translationFinished) {
                if (Math.abs(measuredDistance) >= Math.abs(targetDistance)) {
                    measuredDistance = 0;
                    translationFinished = true;
                    return true;
                }
                Pose testPose = Pose.copy(pose);
                testPose.move(moveSpeed * movementDirection);
                estimatedPose.move((moveSpeed + noise) * movementDirection);
                measuredDistance += (moveSpeed + noise) * movementDirection;
                if (world.checkIfPositionIsFree(testPose.getPosition(), id)) {
                    pose.move(moveSpeed * movementDirection);
                }
            }
            //System.out.println("SimRobot orientation: " + pose.getHeading().getValue());
            return false;
        }
    }

    /**
     * Compute the intersection between each sensors line of sight and any of
     * the features, and add the distance to the intersection as a new
     * measurement. The value given in "noise" is added to the measurement
     *
     * @param features ArrayList<Feature>
     * @param sensorNoise double
     */
    void measureIR(double sensorNoise) {
        Position lineOfSightStart = pose.getPosition();
        Angle lineOfSightAngle = Angle.sum(towerAngle, pose.getHeading());
        for (int i = 0; i < 4; i++) {
            lastIrMeasurement[i] = 0;

            // Create a feature along the line of sight for each sensor
            if (i > 0) {
                lineOfSightAngle.add(90);
            }
            Position offset = Utilities.polar2cart(lineOfSightAngle, maxVisualLength);
            Position lineOfSightEnd = Position.sum(lineOfSightStart, offset);
            Line lineOfSight = Line.convertFeatureToLine(new Feature(lineOfSightStart, lineOfSightEnd));
            lastIrMeasurement[i] = world.findNearestIntersection(lineOfSight, id);
            if (lastIrMeasurement[i] != 0) {
                lastIrMeasurement[i] += sensorNoise;
            }
        }
    }

    static DroneUpdateMessage generateDroneUpdate(int xPos, int yPos, int robotHeading, int startX, int startY, int stopX, int stopY) {

        ByteBuffer msg = ByteBuffer.allocate(14);
        msg.order(ByteOrder.LITTLE_ENDIAN);
        DroneUpdateMessage um = null;
        try {
            msg.putShort((short) xPos);
            msg.putShort((short) yPos);
            msg.putShort((short) robotHeading);
            msg.putShort((short) startX);
            msg.putShort((short) startY);
            msg.putShort((short) stopX);
            msg.putShort((short) stopY);

            msg.rewind();
            byte[] data = new byte[14];
            msg.get(data);
            um = new DroneUpdateMessage(data);
        } catch (Exception e) {
        }
        return um;
    }

    static UpdateMessage generateUpdate(int xPos, int yPos, int robotHeading, int towerHeading, int s1, int s2, int s3, int s4) {

        ByteBuffer msg = ByteBuffer.allocate(12);
        msg.order(ByteOrder.LITTLE_ENDIAN);
        UpdateMessage um = null;
        try {
            msg.putShort((short) xPos);
            msg.putShort((short) yPos);
            msg.putShort((short) robotHeading);
            msg.putShort((short) towerHeading);
            //A byte is to small to hold an integer, causes problems for Drone!
            //Is not in compliance with com protocol
            msg.put((byte) s1);
            msg.put((byte) s2);
            msg.put((byte) s3);
            msg.put((byte) s4);
            msg.rewind();
            byte[] data = new byte[12];
            msg.get(data);
            um = new UpdateMessage(data);
        } catch (Exception e) {
        }
        return um;
    }

    HandshakeMessage generateHandshake() {
        ByteBuffer msg = ByteBuffer.allocate(HandshakeMessage.BASE_LENGTH + name.length());
        msg.order(ByteOrder.LITTLE_ENDIAN);
        int robotWidth = 40;
        int robotLength = 60;
        int toweroffset[] = {30, 40};
        int axleoffset = 0;
        int[] sensoroffset = {5, 5, 5, 5};
        int[] irheading = {0, 90, 180, 270};
        int messageDeadline = 400;

        HandshakeMessage hm = null;
        try {
            msg.put((byte) name.length());
            msg.put(name.getBytes());
            msg.putShort((short) robotWidth);
            msg.putShort((short) robotLength);
            int i;
            for (i = 0; i < 2; i++) {
                msg.put((byte) toweroffset[i]);
            }
            msg.put((byte) axleoffset);
            for (i = 0; i < 4; i++) {
                msg.put((byte) sensoroffset[i]);
            }
            for (i = 0; i < 4; i++) {
                msg.putShort((short) irheading[i]);
            }
            msg.putShort((short) messageDeadline);
            byte[] data = new byte[HandshakeMessage.BASE_LENGTH + name.length()];
            msg.rewind();
            msg.get(data);
            hm = new HandshakeMessage(data);
        } catch (Exception e) {

        }
        return hm;
    }
}
