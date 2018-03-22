/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.robot;

/**
 * This class holds the parameters and characteristics of the IR sensor tower.
 *
 * @author Thor Eivind and Mats (Master 2016 NTNU)
 */
public class IR {

    private int numberOfSensors;
    private final int[] heading, spreading;

    /**
     * Constructor of the class IR
     *
     * @param sensors array that contains the heading of the sensors
     */
    public IR(int[] sensors) {
        numberOfSensors = sensors.length;
        spreading = new int[numberOfSensors];
        heading = sensors;

        // Calculates the spreading between the sensors
        for (int i = 0; i < numberOfSensors; i++) {
            spreading[i] = sensors[i] - sensors[0];
        }
    }

    /**
     * Method that returns the number of sensors
     *
     * @return the number of sensors
     */
    public int getNumberOfSensors() {
        return numberOfSensors;
    }

    /**
     * Method that sets the number of sensors
     *
     * @param numberOfSensors the number of sensors
     */
    public void setNumberOfSensors(int numberOfSensors) {
        this.numberOfSensors = numberOfSensors;
    }

    /**
     * Method that returns the heading of the sensors
     *
     * @return heading of every IR sensor on the robot
     */
    public int[] getHeading() {
        return heading;
    }

    /**
     * Method that returns the spreading between the sensors
     *
     * @return the spreading between every IR sensor on the robot
     */
    public int[] getSpreading() {
        return spreading;
    }
}
