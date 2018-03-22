/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.general;

import java.awt.Graphics2D;

/**
 * This class is used to represent angles. The angle is represented as a double
 * in the range [0,360). All functions in this class are implemented so that the 
 * value stays within these bounds.
 * 
 * @author Eirik Thon
 */
public class Angle {
    
    private double value;

    /**
     * Constructs a new Angle with value 0
     * @param angle value of initial angle
     */
    public Angle() {
        value = 0;
    }
    
    /**
     * Constructs a new Angle with value specified by the input parameter
     * @param angle value of the angle
     */
    public Angle(double value) {
        this.value = value % 360;
        if (value < 0) {
            value += 360;
        }
    }
    
    /**
     * Returns the value of the angle
     * @return value of Angle
     */
    public double getValue() {
        return value;
    }
    
    /**
     * Creates a new Angle with the same value as the input Angle
     * @param angle Angle to copy
     * @return new Angle equal to the input parameter
     */
    public static Angle copy(Angle angle) {
        return new Angle(angle.value);
    }
    
    /**
     * Adds the value input Angle to the value of the Angle. The
     * @param addedAngle Angle to be added
     * @return 
     */
    public void add(Angle addedAngle){
        value = (value + addedAngle.value) % 360;
    }
    
    /**
     * Adds the input value to the angle
     * @param addedAngle value to be added
     */
    public void add(double addedAngle){
        value = (value + (addedAngle % 360)) % 360;
        if(value < 0) {
            value += 360;
        }
    } 
    
    /**
     * Returns a new Angle that is the sum of the two input Angles
     * @param angl1
     * @param angl2
     * @return 
     */
    public static Angle sum(Angle angl1, Angle angl2) {
        double result = (angl1.value + angl2.value)%360;
        return new Angle(result);
    }
    
    /**
     * Computes the lowest difference between two angles
     * @param angl1
     * @param angl2
     * @return double
     */
    public static double difference(Angle angl1, Angle angl2) {
        double result = Math.abs(angl1.value - angl2.value);
        if(result <= 180){
            return result;
        }else{
            return 360 - result;
        }
    }
    
    /**
     * Prints the value of the angle
     */
    public void print() {
        System.out.println("Angle: " + value);
    }
    
    /**
     * Draws a line onto g2D starting in "start", with length "length" in the
     * direction of the angle
     * @param g2D
     * @param start
     * @param length 
     */
    public void drawAngle (Graphics2D g2D, double xStart, double yStart, int length, double scale) {
        Position headingPoint = Utilities.polar2cart(this, length);
        g2D.drawLine((int)(Math.round(xStart)*scale), (int)(Math.round(yStart)*scale),
                (int)(Math.round(xStart+headingPoint.getXValue())*scale), (int)(Math.round(yStart+headingPoint.getYValue())*scale));
    }
}
