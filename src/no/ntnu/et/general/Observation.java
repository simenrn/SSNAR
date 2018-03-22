/**
 * This code is written as a part of a Master Thesis
 * the fall of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.et.general;

/**
 * Holds the coordinates of measurements made by each of the four IR-rays
 * 
 * @author geirhei
 */
public class Observation {
    private final Position position1;
    private final Position position2;
    private final Position position3;
    private final Position position4;
    
    public Observation(Position pos1, Position pos2, Position pos3, Position pos4) {
        position1 = pos1;
        position2 = pos2;
        position3 = pos3;
        position4 = pos4;
    }
}
