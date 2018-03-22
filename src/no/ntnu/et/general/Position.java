/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.general;
import java.awt.Graphics2D;

/**
 * This class represents a position using an x-value, an a y-value
 * 
 * @author Eirik Thon
 */
public class Position {
    
    private double xValue;
    
    private double yValue;
    
    /**
     * Constructor. Initializes to zero
     */
    public Position() {
        xValue = 0;
        yValue = 0;
    }

    /**
     * Creates a new Position object
     * @param xPosition double
     * @param yPosition double
     */
    public Position(double xPosition, double yPosition) {
        this.xValue = xPosition;
        this.yValue = yPosition;
    }
    
    /**
     * Creates a new Position object
     * @param position 
     */
    public Position(int[] position) {
        this.xValue = position[0];
        this.yValue = position[1];
    }
    
    /**
     * Returns the x-value of the Position object
     * @return 
     */
    public double getXValue() {
        return xValue;
    }

    /**
     * Returns the x-value of the Position object
     * @return 
     */
    public double getYValue() {
        return yValue;
    }
    
    /**
     * 
     * @param position
     * @return 
     */
    public static Position copy(Position position) {
        return new Position(position.xValue, position.yValue);
    }
 
    /**
     * prints the x-, y-values of the Position object
     */
    public void print() {
        System.out.println("x-position " + xValue + ", y-position " + yValue);
    }
    
    /**
     * Draws a circle centered at the position onto a Graphics2D object. The 
     * scale parameter can be used o mace the circle bigger and smaller
     * @param g2D Graphics2D
     * @param diameter double
     */
    public void drawCircle(Graphics2D g2D, int diameter, double scale) {
        g2D.drawOval((int)((Math.round(xValue)-diameter/2)*scale),(int)((Math.round(yValue)-diameter/2)*scale), (int)((double)diameter*scale), (int)((double)diameter*scale));
    }
    
    /**
     * Draws a cross centered at the position onto a Graphics2D object. Size
     * determines the distance from the center of the cross to the end points
     * @param g2D Graphics2D
     * @param size integer
     */
    public void drawCross(Graphics2D g2D, int size, double scale) {
        g2D.drawLine((int)(Math.round(xValue-size)*scale), (int)(Math.round(yValue-size)*scale),
                                (int)(Math.round(xValue+size)*scale), (int)(Math.round(yValue+size)*scale));
        g2D.drawLine((int)(Math.round(xValue-size)*scale), (int)(Math.round(yValue+size)*scale),
                (int)(Math.round(xValue+size)*scale), (int)(Math.round(yValue-size)*scale));
    }
    
    /**
     * Adds the values of a Position object to the Position object
     * @param other 
     */
    public void add(Position other) {
        xValue += other.xValue;
        yValue += other.yValue;
    }
    
    /**
     * Creates a new position that is the sum of two positions
     * @param pos1
     * @param pos2
     * @return 
     */
    public static Position sum(Position pos1, Position pos2) {
        Position sum = new Position(0, 0);
        sum.xValue = pos1.xValue + pos2.xValue;
        sum.yValue = pos1.yValue + pos2.yValue;
        return sum;
    }
    
    /**
     * Returns the distance between two positions
     * @param pos1
     * @param pos2
     * @return 
     */
    public static double distanceBetween(Position pos1, Position pos2){
        return Math.sqrt(Math.pow(pos1.xValue - pos2.xValue, 2) + Math.pow(pos1.yValue - pos2.yValue, 2));
    }
    
    /**
     * Returns the angle of a line between two positions
     * @param pos1
     * @param pos2
     * @return 
     */
    public static Angle angleBetween(Position pos1, Position pos2) {
        return new Angle(Math.toDegrees(Math.atan2(pos2.yValue - pos1.yValue, pos2.xValue - pos1.xValue)));
    }
    
    /**
     * Applies a rotational matrix to the position rotating the position about
     * origo
     * @param rotation 
     */
    public void transform(Angle rotation){
        double rad = Math.toRadians(rotation.getValue());
        double xTemp = xValue;
        double yTemp = yValue;
        xValue = Math.cos(rad)*xTemp+-Math.sin(rad)*yTemp;
        yValue = Math.sin(rad)*xTemp+Math.cos(rad)*yTemp;
    }
}
