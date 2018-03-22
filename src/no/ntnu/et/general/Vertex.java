/**
 * This code is written as a part of a Master Thesis
 * the fall of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.et.general;

/**
 * Interior = 0
 * Exterior = 1
 * Ill-defined = 2
 * 
 * @author geirhei
 */
public class Vertex {
    public static final int INTERIOR = 0;
    public static final int EXTERIOR = 1;
    public static final int ILL_DEFINED = 2;
    
    private int type;
    private Position position;
    
    /**
     * Constructor
     * 
     * @param type 
     */
    public Vertex(int type, Position position) {
        this.type = type;
        this.position = position;
    }
    
    /**
     * Setter for type.
     * @return 
     */
    public int getType() {
        return type;
    }
    
    /**
     * Getter for type.
     * @param type 
     */
    public void setType(int type) {
        this.type = type;
    }
    
    public Position getPosition() {
        return position;
    }
    
    public void setPosition(Position pos) {
        this.position = pos;
    }
}
