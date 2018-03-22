/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.robot;

/**
 * This class represents one measurement made by a robot. It holds the robots
 * pose and the ir sensor readings.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class Measurement {

    private final int xPos, yPos, theta;
    private final int[] IRdata, IRheading;

    /**
     * The constructor of the class Measurement
     *
     * @param measuredOrientation measured orientation
     * @param measuredPosition measured position
     * @param irHeading the heading of the first sensor in the ir tower
     * @param irData measured IR data
     */
    Measurement(int measuredOrientation, int[] measuredPosition, int[] irHeading, int[] irData) {
        this.theta = measuredOrientation;
        this.xPos = measuredPosition[0];
        this.yPos = measuredPosition[1];
        this.IRheading = irHeading;
        this.IRdata = irData;
    }
    
    /**
     * Copy constructor
     * @param copy 
     */
    public Measurement(Measurement copy) {
        this.theta = copy.getTheta();
        this.xPos = copy.getxPos();
        this.yPos = copy.getyPos();
        int[] copyIRHeading = new int[copy.getIRHeading().length];
        System.arraycopy(copy.getIRHeading(), 0, copyIRHeading, 0, copy.getIRHeading().length);
        this.IRheading = copyIRHeading;
        int[] copyIRdata = new int[copy.getIRHeading().length];
        System.arraycopy(copy.getIRdata(), 0, copyIRdata, 0, copy.getIRdata().length);
        this.IRdata = copy.getIRdata();
    }
    
    /**
     * Method that returns the robots x position
     *
     * @return the position
     */
    public int getxPos() {
        return xPos;
    }

    /**
     * Method that returns the robots y position
     *
     * @return the position
     */
    public int getyPos() {
        return yPos;
    }

    /**
     * Method that returns the robots orientation
     *
     * @return the orientation
     */
    public int getTheta() {
        return theta;
    }

    /**
     * Method that returns the heading of the first sensor in the ir tower
     *
     * @return the heading
     */
    public int[] getIRHeading() {
        return IRheading;
    }

    /**
     * Method that returns the measured ir data
     *
     * @return the ir data
     */
    public int[] getIRdata() {
        return IRdata;
    }
}
