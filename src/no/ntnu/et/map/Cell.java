/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.map;

import java.util.ArrayList;

/**
 * This class is used to represent a cell in a grid map.
 * 
 * @author Eirik Thon
 */
public class Cell {
    private boolean previouslyObserved;
    private boolean restricted;
    private boolean weaklyRestricted;
    private boolean frontier;
    private int measurementCounter;
    private boolean occupied;
    private boolean stateChanged;
    private boolean isTarget;
    private boolean isPath;
    private boolean particle;
    
    ArrayList<Cell> restrictingCells; // Other cells that restricts this cell
    ArrayList<Cell> weaklyRestrictingCells; // Other cells that weakly restricts this cell
    
    
    /**
     * Creates a new cell with default initial values
     */
    public Cell(){
        occupied = false;
        previouslyObserved = false;
        restricted = false;
        frontier = false;
        measurementCounter = 0;
        stateChanged = false;
        weaklyRestricted = false;
        isPath = false;
        isTarget = false;
        restrictingCells = new ArrayList<Cell>();
        weaklyRestrictingCells = new ArrayList<Cell>();
    }
    
    /**
     * Adds a cell to the restricting cell. If the cell has more than
     * one restricting cell its restricted status is set to true
     * @param otherCell 
     */
    public void addRestrictingCell(Cell otherCell){
        restrictingCells.add(otherCell);
        if(!restrictingCells.isEmpty()){
            restricted = true;
        }
    }
    
    /**
     * Adds a cell to the weakly restricting cells. If the cell has more than
     * one weakly restricting cell its weakly restricted status is set to true
     * @param otherCell 
     */
    public void addWeaklyRestrictingCell(Cell otherCell){
        weaklyRestrictingCells.add(otherCell);
        if(!weaklyRestrictingCells.isEmpty()){
            weaklyRestricted = true;
        }
    }
    
    /**
     * Removes a cell from the restricting cells. If the cell has 0 restricting
     * cells its restricted status is set to false
     * @param otherCell cell to remove
     */
    public void removeRestrictingCell(Cell otherCell){
        restrictingCells.remove(otherCell);
        if(restrictingCells.isEmpty()){
            restricted = false;
        }
    }
    
    
    
    /**
     * Removes a cell from the weakly restricting cells. If the cell has 0
     * weakly restricting cell its weakly restricted status is set to false
     * @param otherCell 
     */
    public void removeWeaklyRestrictingCell(Cell otherCell){
        weaklyRestrictingCells.remove(otherCell);
        if(weaklyRestrictingCells.isEmpty()){
            weaklyRestricted = false;
        }
    }
    
    /**
     * Returns the weakly restricted status of the Cell object
     * @return 
     */
    public boolean isWeaklyRestricted(){
        return weaklyRestricted;
    }
    public boolean isPath(){
        return isPath;
    }
    
    public void setPath(){
        isPath = true;
    }
    
    public void setNotPath(){
        isPath = false;
    }
    public boolean isTarget(){
        return isTarget;
    }
    
    public void setTarget(){
        isTarget = true;
    }
    
    public void setNotTarget(){
        isTarget = false;
    }
    
    void update(boolean measurement) {          
        if (measurement){
            if(occupied == false){
                stateChanged = true;
            }
            else{
                stateChanged = false;
            }
            occupied = true;
        }
        else{
            if(occupied == true){
                stateChanged = true;
            }
            else{
                stateChanged = false;
            }
            occupied = false;
        }
        if(!previouslyObserved){
            previouslyObserved = true;
            stateChanged = true;
        }  
    }

    /**
     * Returns true if and only if the "occupied" variable changed during the 
     * latest measurement of the cell
     * @return 
     */
    boolean stateChanged(){
        return stateChanged;
    }
    
    public void setFrontier(){
        frontier = true;
    }
    
    public void setNotFrontier(){
        frontier = false;
    }
    
    boolean isFrontierPoint(){
        return frontier;
    }
    
    public void setUnrestricted(){
        restricted = false;
    }
    
    public void setRestricted(){
        restricted = true;
    }
    
    public boolean isRestricted(){
        return restricted;
    }
    
    public boolean isPreviouslyObserved() {
        return previouslyObserved;
    }
    
    /**
     * Returns true if the cell has been previously observed and is no occupied
     * @return 
     */
    public boolean isFree(){
        if (!previouslyObserved) {
            return false;
        }
        return !occupied;
    }
    
    /**
     * Returns true if the cell has been previously observed and is occupied
     * @return 
     */
    public boolean isOccupied(){
        if (!previouslyObserved) {
            return false;
        }
        return occupied;
    }
    
    /**
     * Returns true if the cell is free and not restricted
     * @return 
     */
    public boolean isWeaklyTargetable(){
        return (!occupied && previouslyObserved && !restricted);
    }
    
    /**
    * Returns true if the cell is free and not weakly restricted
    * @return 
    */
    public boolean isFreelyTargetable(){
        return (!occupied && previouslyObserved && !restricted && !weaklyRestricted);
    }
    
    public void setParticle(){
        particle = true;
    }
    
    public void removeParticle(){
        particle = false;
    }
    
    public boolean isParticle(){
        return particle;
    }
}
