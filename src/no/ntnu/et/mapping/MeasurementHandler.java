/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.mapping;

import java.util.ArrayList;
import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Position;
import no.ntnu.et.general.Pose;
import no.ntnu.tem.robot.Measurement;
import no.ntnu.tem.robot.Robot;

/**
 * This class is used to find the location of the robot and each IR-measurement
 * in the map.
 *
 * @author Eirik Thon
 */
public class MeasurementHandler {

    final private Pose initialPose;
    private int sensorRange;
    private Measurement currentMeasurement;
    private Pose robotPose;
    private Sensor[] sensors;
    private Robot robot;

    public MeasurementHandler(Robot robot, Pose initialPose) {
        this.initialPose = initialPose;
        this.robot = robot;
        sensorRange = 40;
        sensors = new Sensor[4];
        for (int i = 0; i < 4; i++) {
            sensors[i] = new Sensor();
        }
    }

    boolean updateMeasurement() {
        currentMeasurement = robot.getMeasurement();
        if (currentMeasurement == null) {
            return false;
        }
        Position robotPosition = new Position((double) currentMeasurement.getxPos(), (double) currentMeasurement.getyPos());
        Angle robotAngle = new Angle(currentMeasurement.getTheta());
        robotPose = new Pose(robotPosition, robotAngle);
        robotPose.transform(this.initialPose);

        // Update sensor data
        int[] irData = currentMeasurement.getIRdata();
        int[] irheading = currentMeasurement.getIRHeading();

        //Drone data handling
        if (robot.getName().equals("Drone")) {
            sensors[0].setPosition(new Position(irData[0], irData[1]));
            sensors[1].setPosition(new Position(irData[2], irData[3]));
            return true;
        }

        for (int i = 0; i < 4; i++) {
            int measurementDistance = irData[i];
            if (measurementDistance == 0 || measurementDistance > sensorRange) {
                sensors[i].setMeasurement(false);
                measurementDistance = sensorRange;
            } else {
                sensors[i].setMeasurement(true);
            }
            Angle towerAngle = new Angle((double) irheading[i]);
            Angle sensorAngle = Angle.sum(towerAngle, robotPose.getHeading());
            double xOffset = measurementDistance * Math.cos(Math.toRadians(sensorAngle.getValue()));
            double yOffset = measurementDistance * Math.sin(Math.toRadians(sensorAngle.getValue()));
            Position measurementPosition = Position.sum(robotPose.getPosition(), new Position(xOffset, yOffset));
            sensors[i].setPosition(measurementPosition);
        }
        return true;
    }

    Position getRobotPosition() {
        return robotPose.getPosition();
    }

    Angle getRobotHeading() {
        return robotPose.getHeading();
    }

    Sensor[] getIRSensorData() {
        return sensors;
    }

    int[] getSensorAngel() {
        return currentMeasurement.getIRHeading();
    }

    public Measurement getCurrentMeasurement() {
        return currentMeasurement;
    }
}
