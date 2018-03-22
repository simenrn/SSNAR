/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.simulator;

import no.ntnu.et.general.Position;
import java.awt.Graphics2D;


/**
 * This class represent a wall or obstacle in the map. The feature must be a
 * straight line, and it is parameterized by a start and end position.
 * 
 * @author Eirik Thon
 */
public class Feature {
    private Position start;
    private Position end;
    
    /**
     * Constructor
     * @param start Position
     * @param end Position
     */
    public Feature(Position start, Position end) {
        this.start = start;
        this.end = end;
    }
    
    /**
     * Copy constructor. Creates a copy of otherFeature
     * @param otherFeature 
     */
    static Feature copy(Feature feature){
        return new Feature(Position.copy(feature.start), Position.copy(feature.end));
    }
    
    /**
     * Constructor
     * @param xStart
     * @param yStart
     * @param xEnd
     * @param yEnd 
     */
    public Feature(double xStart, double yStart, double xEnd, double yEnd) {
        this.start = new Position(xStart, yStart);
        this.end = new Position(xEnd, yEnd);
    }

    
    
    public Position getStartPosition() {
        return start;
    }

    public Position getEndPosition() {
        return end;
    }
    
    
    
    /**
     * Prints the position of the feature using System.out
     */
    void print(){
        start.print();
        end.print();
    }
    
    /**
     * Paints a line between the start and end position of the feature onto
     * "g2D".
     * @param g2D Graphics2D
     */
    void paint(Graphics2D g2D, double scale) {
       // Draw line between start and end
       g2D.drawLine((int)(Math.round(start.getXValue())*scale), (int)(Math.round(start.getYValue())*scale),
                   (int)(Math.round(end.getXValue())*scale), (int)(Math.round(end.getYValue())*scale));        
    }
    
    static double[] feature2Vector(Feature feature) {
        double x = feature.end.getXValue()-feature.start.getXValue();
        double y = feature.end.getYValue()-feature.start.getYValue();
        double distance = feature.getLength();
        double[] vector = {x/distance, y/distance};
        return vector;
    }
    
    public double getLength() {
        return Position.distanceBetween(start, end);
    }
}
