/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.map;

import java.util.ArrayList;

/**
 * This class represent a location in a grid. It contains the row and the column
 * of the location
 * 
 * @author Eirik Thon
 */
public class MapLocation {
    private int row;
    private int column;

    /**
     * Constructor
     * @param row
     * @param column 
     */
    public MapLocation(int row, int column) {
        this.row = row;
        this.column = column;
    }

    /**
     * Returns the row index
     * @return 
     */
    public int getRow() {
        return row;
    }

    /**
     * Returns the column index
     * @return 
     */
    public int getColumn() {
        return column;
    }
    
    /**
     * Prints the row and column of the MapLocation
     */
    public void print() {
        System.out.println("Row "+row+", column "+ column);
    }
    
    /**
     * Returns a deep copy of the specified MapLocation
     * @param location
     * @return 
     */
    public static MapLocation copy(MapLocation location){
        return new MapLocation(location.row, location.column);
    }
    
    /**
     * Returns true if two location have equal values.
     * @param loc1
     * @param loc2
     * @return 
     */
    public static boolean equals(MapLocation loc1, MapLocation loc2){
        return loc1.row==loc2.row && loc1.column==loc2.column;
    }
    
    /**
     * Returns a new MapLocation that is the sum of the specified locations
     * @param loc1
     * @param loc2
     * @return 
     */
    public static MapLocation sum(MapLocation loc1, MapLocation loc2) {
        return new MapLocation(loc1.row+loc2.row, loc1.column+loc2.column);
    }
    
    /**
     * Returns the distance between two location. OBS: Not the real distance
     * as cell size is no accounted for
     * @param loc1
     * @param loc2
     * @return 
     */
    public static double distance(MapLocation loc1, MapLocation loc2){
        return Math.sqrt(Math.pow(loc1.row - loc2.row, 2) + Math.pow(loc1.column - loc2.column, 2));
    }
    
    /**
     * Returns the angle of a line that goes through both locations
     * @param loc1
     * @param loc2
     * @return 
     */
    public static double angleBetween(MapLocation loc1, MapLocation loc2){
        int dx = loc2.getColumn()-loc1.getColumn();
        int dy = loc2.getRow()-loc1.getRow();
        double angle = Math.toDegrees(Math.atan2(dy, dx));
        if(angle < 0){
            return angle += 360;
        }else{
            return angle;
        }
    }
    
    /**
     * Transforms the MapLocation into the first octant. Found on
     * https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
     * @param octant
     * @param location
     * @return 
     */
    public static MapLocation switchToOctantZeroFrom(int octant, MapLocation location) {
        MapLocation newLocation = new MapLocation(0, 0);
        switch(octant) { 
            case 0:
                newLocation.column = location.column;
                newLocation.row = location.row;
                return newLocation;
            case 1:
                newLocation.column = location.row;
                newLocation.row = location.column;
                return newLocation;
            case 2:
                newLocation.column = location.row;
                newLocation.row = -location.column;
                return newLocation;
            case 3:
                newLocation.column = -location.column;
                newLocation.row = location.row;
                return newLocation;
            case 4:
                newLocation.column = -location.column;
                newLocation.row = -location.row;
                return newLocation;
            case 5:
                newLocation.column = -location.row;
                newLocation.row = -location.column;
                return newLocation;
            case 6:
                newLocation.column = -location.row;
                newLocation.row = location.column;
                return newLocation;
            case 7:
                newLocation.column = location.column;
                newLocation.row = -location.row;
                return newLocation;
            default:
                System.out.println("ERROR INVALID OCTANT: " + octant);
                return newLocation;
        }
    }
    
    
    /**
     * Transforms the MapLocation back to its original octant. Found on
     * https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
     * @param octant
     * @param location
     * @return 
     */
    public static MapLocation switchFromOctantZeroTo(int octant, MapLocation location) {
        MapLocation newLocation = new MapLocation(0, 0);
        switch(octant) {
            case 0:
                newLocation.column = location.column;
                newLocation.row = location.row;
                return newLocation;
            case 1:
                newLocation.column = location.row;
                newLocation.row = location.column;
                return newLocation;
            case 2:
                newLocation.column = -location.row;
                newLocation.row = location.column;
                return newLocation;
            case 3:
                newLocation.column = -location.column;
                newLocation.row = location.row;
                return newLocation;
            case 4:
                newLocation.column = -location.column;
                newLocation.row = -location.row;
                return newLocation;
            case 5:
                newLocation.column = -location.row;
                newLocation.row = -location.column;
                return newLocation;
            case 6:
                newLocation.column = location.row;
                newLocation.row = -location.column;
                return newLocation;
            case 7:
                newLocation.column = location.column;
                newLocation.row = -location.row;
                return newLocation;
            default:
                return newLocation;
        }
    }
    
    /**
     * Returns the octant of an angle. Found on
     * https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
     * @param angle
     * @return 
     */
    public static int getOctant(double angle) {
        if (angle > 0 && angle <= 45) {
            return 0;
        }
        if (angle > 45 && angle <= 90) {
            return 1;
        }
        if (angle > 90 && angle <= 135) {
            return 2;
        }
        if (angle > 135 && angle <= 180) {
            return 3;
        }
        if (angle > 180 && angle <= 225) {
            return 4;
        }
        if (angle > 225 && angle <= 270) {
            return 5;
        }
        if (angle > 270 && angle <= 315) {
            return 6;
        }
        if ((angle > 315 && angle < 360) || angle == 0) {
            return 7;
        } else {
            System.out.println("ERROR INVALID ANGLE: " + angle);
            return -1;
        }
    }
    
    /**
     * Special hash function that creates an equal hash code for locations
     * with the same row and column. Found on
     * http://stackoverflow.com/questions/9135759/java-hashcode-for-a-point-class
     * @return 
     */
    @Override
    public int hashCode() {
        int result = row;
        result = 31 * result + column;
        return result;
    }

    /**
     * Returns true if two MapLocation are equal
     * @param obj
     * @return 
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final MapLocation other = (MapLocation) obj;
        if (this.row != other.row) {
            return false;
        }
        if (this.column != other.column) {
            return false;
        }
        return true;
    }
}
