/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.general;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import no.ntnu.et.map.MapLocation;
import no.ntnu.et.simulator.Feature;

/**
 * This class is used to represent lines by specifying the start point of the
 * line, its direction and its length.'
 * 
 * @author Eirik Thon
 */
public class Line {
    
    private double[] start;
    
    private double[] direction;
    
    private double length;
    
    private Position a;
    private Position b;

    /**
     * Creates a new Line object
     * @param start
     * @param direction
     * @param length 
     */
    public Line(double[] start, double[] direction, double length) {
        this.start = start;
        this.direction = direction;
        this.length = length;
    }
    
    /**
     * Constructor Line object used in line merge
     * @param a
     * @param b 
     */
    public Line(Position a, Position b) {
        this.a = a;
        this.b = b;
        this.length = Math.sqrt( Math.pow(b.getXValue() - a.getXValue(), 2) + Math.pow(b.getYValue() - a.getYValue(), 2) );
    }
    
    /**
     * Empty constructor
     */
    private Line() {};

    /**
     * Returns the start position of the line
     * @return double[] start position 
     */
    public double[] getStart() {
        return start;
    }

    /**
     * Returns the two values specifying the direction of the line
     * @return double[]
     */
    public double[] getDirection() {
        return direction;
    }

    /**
     * Returns the length of the line
     * @return double
     */
    public double getLength() {
        return length;
    }
    
    private double getSlope() {
        return (b.getYValue() - a.getYValue()) / (b.getXValue() - a.getXValue());
    }
    
    public Position getA() {
        return a;
    }
    
    public Position getB() {
        return b;
    }
    
    /**
     * Creates a new Line object similar to the input Feature object. The Line
     * object starts in the start position of the feature and ends in its end 
     * position
     * @param feature Feature
     * @return 
     */
    public static Line convertFeatureToLine(Feature feature){
        double[] start = new double[2];
        start[0] = feature.getStartPosition().getXValue();
        start[1] = feature.getStartPosition().getYValue();
        double length = feature.getLength();
        double xDiff = feature.getEndPosition().getXValue()-feature.getStartPosition().getXValue();
        double yDiff = feature.getEndPosition().getYValue()-feature.getStartPosition().getYValue();
        double[] vector = {xDiff/length, yDiff/length};
        return new Line(start, vector, length);
    }
    
    /**
     * Creates a new Line object between the two Position given in the input
     * parameters
     * @param startPos Position
     * @param endPos Position
     * @return Line
     */
    public static Line getLineBetweenPositions(Position startPos, Position endPos){
        double length = Position.distanceBetween(startPos, endPos);
        if(length == 0){
            return null;
        }
        double[] start = new double[2];
        start[0] = startPos.getXValue();
        start[1] = startPos.getYValue();
        double xDiff = endPos.getXValue()-startPos.getXValue();
        double yDiff = endPos.getYValue()-startPos.getYValue();
        double[] vector = {xDiff/length, yDiff/length};
        return new Line(start, vector, length);
    }
    
    public static void lineCreate(ArrayList<Position> pointBuffer, ArrayList<Line> lineBuffer) {
        if (pointBuffer.size() <= 1) {
            pointBuffer.clear();
            return;
        } else if (pointBuffer.size() == 2) {
            Line line = new Line(pointBuffer.get(0), pointBuffer.get(1));
            lineBuffer.add(line);
            pointBuffer.clear();
            return;
        }
        
        Position a = pointBuffer.get(0);
        Position b = pointBuffer.get(1);
        
        int i = 2;
        while (i < pointBuffer.size()) {
            Line line;
            if (i == pointBuffer.size() - 1) {
                if (!isCollinear(a, b, pointBuffer.get(i))) {
                    line = new Line(a, pointBuffer.get(i-1));
                } else {
                    line = new Line(a, pointBuffer.get(i));
                }
                i++;
            } else if (i == pointBuffer.size() - 2) {
                if (!isCollinear(a, b, pointBuffer.get(i))) {
                    line = new Line(a, pointBuffer.get(i-1));
                    a = pointBuffer.get(i);
                    b = pointBuffer.get(i+1);
                    i++;
                } else {
                    i++;
                    break;
                }
            } else {
                if (!isCollinear(a, b, pointBuffer.get(i))) {
                    line = new Line(a, pointBuffer.get(i-1));
                    a = pointBuffer.get(i);
                    b = pointBuffer.get(i+1);
                    i += 2;
                } else {
                    i++;
                    break;
                }
            }
            
            // Discard lines that are too long
            if (line.getLength() < 20) {
                lineBuffer.add(line);
            }
        }
        pointBuffer.clear();
    }
    
    public static void lineMerge(ArrayList<Line> lineBuffer, List<Line> lineRepository) {
        // Array size check here

        if (lineRepository.isEmpty()) {
            synchronized (lineRepository) {
                for (Line bufferLine : lineBuffer) {
                    lineRepository.add(bufferLine);
                }
            }
            lineBuffer.clear();
            return;
        }
        
        double u = 20; // slope tolerance
        double d = 20; // distance tolerance
        
        ArrayList<Line> toAdd = new ArrayList<Line>();
        ListIterator<Line> iter1 = lineBuffer.listIterator();
        while (iter1.hasNext()) {
            Line bufferLine = iter1.next();
            double m1 = bufferLine.getSlope();
            synchronized (lineRepository) {
                ListIterator<Line> iter2 = lineBuffer.listIterator();
                while (iter2.hasNext()) {
                    Line line = iter2.next();
                    double m2 = line.getSlope();
                    if (!(Math.abs(m1 - m2) <= u)) {
                        break;
                    }
                    double dist1 = Position.distanceBetween(bufferLine.getA(), line.getA());
                    double dist2 = Position.distanceBetween(bufferLine.getA(), line.getB());
                    double dist3 = Position.distanceBetween(bufferLine.getB(), line.getA());
                    double dist4 = Position.distanceBetween(bufferLine.getB(), line.getB());
                    if (dist1 <= d || dist4 <= d) {
                        Line newLine = bufferLine;
                        iter2.set(newLine);
                    } else if (dist2 <= d) {
                        Line newLine = new Line(line.getA(), bufferLine.getB());
                        iter2.set(newLine);
                    } else if (dist3 <= d) {
                        Line newLine = new Line(bufferLine.getA(), line.getB());
                        iter2.set(newLine);
                    } else {
                        break;
                    }
                }
                if (!iter2.hasNext()) {
                    toAdd.add(bufferLine);
                }
            }
        }
        
        // Add non-merged lines to end of lineRepository
        synchronized (lineRepository) {
            for (Line bufferLine : toAdd) {
                lineRepository.add(bufferLine);
            }
        }
        lineBuffer.clear();
    }
    
    private static boolean isMergeable(Line line1, Line line2, double u, double d) {
        double m1 = line1.getSlope();
        double m2 = line2.getSlope();
        double m = Math.abs(m1 - m2);
        double dist1 = Position.distanceBetween(line1.getA(), line2.getA());
        double dist2 = Position.distanceBetween(line1.getA(), line2.getB());
        double dist3 = Position.distanceBetween(line1.getB(), line2.getA());
        double dist4 = Position.distanceBetween(line1.getB(), line2.getB());
        return ( (m <= u) && (dist1 <= d || dist2 <= d || dist3 <= d || dist4 <= d) );
    }
    
    /**
     * Determines if 3 given points are collinear
     * 
     * @param a
     * @param b
     * @param c
     * @return 
     */
    private static boolean isCollinear(Position a, Position b, Position c) {
        double x1 = a.getXValue();
        double y1 = a.getYValue();
        double x2 = b.getXValue();
        double y2 = b.getYValue();
        double x3 = c.getXValue();
        double y3 = c.getYValue();
        return Math.abs((y1 - y2) * (x1 - x3) - (y1 - y3) * (x1 - x2)) <= 1e-9; // epsilon because of float comparison
    }
    
    /**
     * Prints all the values of the Line object
     */
    public void print(){
        System.out.println("Start: " + start[0] + ", " + start[1]);
        System.out.println("Direction: " + direction[0] + ", " + direction[1]);
        System.out.println("Length: " + length);
    }
}
