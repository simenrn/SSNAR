/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.general;

import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Position;
import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import no.ntnu.et.general.Line;

/**
 * This class contains methods useful to other classes.
 * 
 * @author Eirik Thon
 */
public class Utilities {
    
    /**
     * Create s new Position object with values given in Cartesian coordinates.
     * @param theta Angle object
     * @param r double
     * @return a new Position object
     */
    public static Position polar2cart(Angle theta, double r){
        double thetaRad = Math.toRadians(theta.getValue());
        double x = r*Math.cos(thetaRad);
        double y = r*Math.sin(thetaRad);
        Position cart = new Position(x,y);
        return cart;
    }
    
    /**
     * Creates a new Position from data in "string". "string" should be on the
     * format "x,y".
     * @param string
     * @return 
     */
    public static Position string2Position(String string){
        String[] stringParts = string.split(",");
        double x = Integer.parseInt(stringParts[0]);
        double y = Integer.parseInt(stringParts[1]);
        Position position = new Position(x,y);
        return position;
    }
    
    /**
     * Computes and returns the end position of a vector. The vector is
     * parameterized by a starting point, unit vector, and a distance along
     * the unit vector
     * @param start Position
     * @param unitVector double[2]
     * @param distance double
     * @return 
     */
    public static Position findPositionAlongLine(double[] start, double[] unitVector, double distance) {
        double xValue = start[0] + unitVector[0]*distance;
        double yValue = start[1] + unitVector[1]*distance;
        return new Position(xValue, yValue);
    }
    
    /**
     * Selects a color within a predefined set of colors. The
     * parameter "i" is used to select a color from the set. Modulo is used if
     * i is larger than the size of the set
     * @param g Graphics2D
     * @param i integer
     * @return Color
     */
    public static Color selectColor(int i) {
        switch (i % 10) {
            case 0:
                return Color.blue;
            case 1:
                return Color.red;
            case 2:
                return Color.green;
            case 3:
                return Color.magenta;
            case 4:
                return Color.orange;
            case 5:
                return Color.cyan;
            case 6:
                return Color.pink;
            case 7:  
                return Color.lightGray;
            case 8:
                return Color.darkGray;
            case 9:
                return Color.yellow;
            default:
                return Color.black;
        }
    }
    
    /**
     * Finds the intersection points of  Line object and a circle. The method
     * returns the distance to the nearest intersection point along the line.
     * If no intersection exists the method returns 0. 
     * @param line
     * @param circleCenter
     * @param circleRadius
     * @return 
     */
    public static double lineCircleIntersection(Line line, Position circleCenter, double circleRadius) {
        double v_x = line.getDirection()[0];
        double v_y = line.getDirection()[1];
        double p_x = line.getStart()[0];
        double p_y = line.getStart()[1];
        double lineLength = line.getLength();
        double x_c = circleCenter.getXValue();
        double y_c = circleCenter.getYValue();
        double r = circleRadius;
        // See Thon 2016 on where this monstrous equation comes from
        double t1 = (Math.sqrt(-Math.pow(p_x*v_y,2) + 2*p_x*p_y*v_x*v_y - 2*p_x*v_x*v_y*y_c + 2*p_x*Math.pow(v_y,2)*x_c - Math.pow(p_y*v_x,2) + 2*p_y*Math.pow(v_x,2)*y_c - 2*p_y*v_x*v_y*x_c + Math.pow(r*v_x,2) + Math.pow(r*v_y,2) - Math.pow(v_x*y_c,2) + 2*v_x*v_y*x_c*y_c - Math.pow(v_y*x_c,2)) - p_x*v_x - p_y*v_y + v_x*x_c + v_y*y_c)/(Math.pow(v_x,2) + Math.pow(v_y,2));
        double t2 = -(Math.sqrt(-Math.pow(p_x*v_y,2) + 2*p_x*p_y*v_x*v_y - 2*p_x*v_x*v_y*y_c + 2*p_x*Math.pow(v_y,2)*x_c - Math.pow(p_y*v_x,2) + 2*p_y*Math.pow(v_x,2)*y_c - 2*p_y*v_x*v_y*x_c + Math.pow(r*v_x,2) + Math.pow(r*v_y,2) - Math.pow(v_x*y_c,2) + 2*v_x*v_y*x_c*y_c - Math.pow(v_y*x_c,2)) + p_x*v_x + p_y*v_y - v_x*x_c - v_y*y_c)/(Math.pow(v_x,2) + Math.pow(v_y,2));
        if(t1 != Double.NaN && t2 != Double.NaN){
            if(t1 < t2 && t1 > 0 && t1 < lineLength){
                return t1;
            }
            if(t2 < t1 && t2 > 0 && t2 < lineLength){
                return t2;
            }
        }
        return 0;
    }
    
    
    /**
     * Finds the intersection point between two lines. If no intersection exists
     * the method returns 0.
     * @param line1
     * @param line2
     * @return The distance to the intersection along the first line.
     */
    public static double lineLineIntersection(Line line1, Line line2){//double[] line1Start, double[] line1Vector, double line1Length, double[] line2Start, double[] line2Vector, double line2Length) {
        double s1_x = line1.getStart()[0];
        double s1_y = line1.getStart()[1];
        double s2_x = line2.getStart()[0];
        double s2_y = line2.getStart()[1];
        double v1_x = line1.getDirection()[0];
        double v1_y = line1.getDirection()[1];
        double v2_x = line2.getDirection()[0];
        double v2_y = line2.getDirection()[1];
        double l1 = line1.getLength();
        double l2 = line2.getLength();
        
        double div = v2_y*v1_x - v2_x*v1_y;
        if (div == 0) {
            return 0;
        }
        double t = ((s2_x-s1_x)*v2_y+(s1_y-s2_y)*v2_x)/div;
        double u = ((s1_y-s2_y)*v1_x+(s2_x-s1_x)*v1_y)/div;
        
        if (t > 0 && u > 0 && t < l1 && u < l2) {
            return t;
        }
        else
            return 0;
    }
    
    /**
     * Calculates the global headings of the IR-tower
     * 
     * @param robotHeading the heading of the robot
     * @param towerHeadings array with the angles of each sensor tower, relative
     * to the robot heading.
     * @return int[4] newHeadings
     */
    public static int[] getMeasurementHeadings(int[] towerHeadings) {
        int[] newHeadings = new int[4];
        for (int i = 0; i < towerHeadings.length; i++) {
            int angle = (i-1)*90 + towerHeadings[i];
            if (angle >= 360) {
                angle -= 360;
            } else if (angle < 0) {
                angle += 360;
            }
            newHeadings[i] = angle;
        }
        return newHeadings;
    }
    
    /**
     * The method projects the input position onto the Line object and returns
     * the projected Position.
     * @param point
     * @param line
     * @return 
     */
    public static Position getOrthogonalProjection(Position point, Line line){
        double xDiff = point.getXValue() - line.getStart()[0];
        double yDiff = point.getYValue() - line.getStart()[1];

        // Create unitvector from feature
        double[] dir = line.getDirection();

        // Project a onto b
        double projectionLength = xDiff*dir[0] + yDiff*dir[1];

        if (projectionLength < line.getLength() && projectionLength > 0) {
            return findPositionAlongLine(line.getStart(), line.getDirection(), projectionLength);
        }

        return null;
    }
    
    /*
    public static void createLines(ArrayList<Position> pointBuffer, ArrayList<Line> lineBuffer) {
        removeDuplicates(pointBuffer);
        Arrays.sort()
    }
*/    

    private static void removeDuplicates(ArrayList<Position> buffer) {
        //List<Position> temp = new ArrayList<Position>();
        Set<Position> temp = new HashSet<>();
        temp.addAll(buffer);
        buffer.clear();
        buffer.addAll(temp);
    }
}
