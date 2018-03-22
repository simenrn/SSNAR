/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.map;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import no.ntnu.et.general.Line;
import no.ntnu.et.general.Observation;
import no.ntnu.et.general.Position;
import no.ntnu.et.general.Vertex;
import no.ntnu.et.mapping.MappingController;

/**
 * This class represents a grid map. A hash table of MapLocations and Cells are
 * used to represent the actual map. The map expand in all directions. Expansion
 * downwards and to the left will lead to negative rows and columns. The map
 * contains variables to specify the indexes of the columns and rows at its
 * boundaries.
 * 
 * ABOUT THE GRID: The cells of the grid are square areas determined by the
 * cell size. The grid is implemented so that it is connected to a coordinate
 * system. It is easiest to explain how this works with an example:
 * 
 * If the cell size is set to 5 cm, all positions in the coordinate system
 * with x-values from [0, 5) are mapped into the first column. Positions with
 * x-values [5,10) are mapped to the second column and so on. The same principle
 * applies to y-values and rows. For example the position (0,0) is has the
 * indexes (row=0,col=0) in the map The position (4,3) also has the indexes
 * (row=0,col=0). The position (5,1) maps to (row=0,col=1).
 * 
 * This principle also continues over to the negative side of the coordinate
 * system. With 5 cm cell size: (-1,3) maps to (row=0, col=-1), and (-5,3) maps
 * to (row=0, col=-2).
 * 
 * This may be confusing, but it (hopefully) makes sense after a while 
 * 
 * @author Eirik Thon
 */

public class GridMap{
    private ConcurrentHashMap<MapLocation, Cell> map;
    private int cellSize;
    private int topRow;
    private int bottomRow;
    private int rightColumn;
    private int leftColumn;
    
    private ArrayList<Vertex> vertices = new ArrayList();
    
    //SLAMrobot simulation
    private ArrayList<MapLocation> obstructed = new ArrayList();
    private ArrayList<MapLocation> frontiers = new ArrayList();
    
    private ArrayList<Position> StateSpace = new ArrayList();
    private ArrayList<Observation> observationHistory = new ArrayList();
    private ArrayList<int[]> actionHistory = new ArrayList();
    private ArrayList<Position> PointBufferF = new ArrayList();
    private ArrayList<Position> PointBufferL = new ArrayList();
    private ArrayList<Position> PointBufferB = new ArrayList();
    private ArrayList<Position> PointBufferR = new ArrayList();
    private ArrayList<ArrayList<Position>> pointBuffers = new ArrayList<ArrayList<Position>>();
    private ArrayList<ArrayList<Line>> lineBuffers = new ArrayList<ArrayList<Line>>();
    //private ArrayList<Line> lineRepository = new ArrayList<Line>();
    private List<Line> lineRepository;
    
    /**
     * Constructor for the GridMap class
     * @param cellSize Specifies the size of cells in cm. Cells are quadratic
     * so no extra dimension is needed
     * @param width Specifies the initial width of the map in cm.
     * @param height Specifies the initial height of the map in cm
     */
    public GridMap(int cellSize, int width, int height) {
        if (cellSize <= 0 || width <= 0 || height <= 0 || width % cellSize != 0 || height % cellSize != 0){
            System.out.println("Error. All parameters must be positive and width and height must be a positive multiple of cellSize");
        }else{
            this.cellSize = cellSize;
            map = new ConcurrentHashMap<MapLocation, Cell>();
            topRow = height/cellSize-1;
            bottomRow = 0;
            rightColumn = width/cellSize-1;
            leftColumn = 0;
            for (int i = bottomRow; i <= topRow; i++) {
                for (int j = leftColumn; j <= rightColumn; j++) {
                    map.put(new MapLocation(i, j), new Cell());
                }
            }
            
            vertices.add( new Vertex(Vertex.INTERIOR, new Position(25, 25)) );
            for (int k = 0; k < 4; k++) {
                pointBuffers.add(new ArrayList<Position>());
                lineBuffers.add(new ArrayList<Line>());
            }
            lineRepository = Collections.synchronizedList(new ArrayList<Line>());
        }
    }
    
    public ArrayList<ArrayList<Position>> getPointBuffers() {
        return pointBuffers;
    }
    
    public ArrayList<ArrayList<Line>> getLineBuffers() {
        return lineBuffers;
    }
    
    public List<Line> getLineRepository() {
        return lineRepository;
    }
    
    public ArrayList<MapLocation> getObstructed() {
        return obstructed;
    }
    
    public ArrayList<MapLocation> getFrontiers() {
        return frontiers;
    }
    
    public void setFrontier(MapLocation loc) {
        obstructed.remove(loc);
        if (!frontiers.contains(loc)) {
            frontiers.add(loc);
        }
    }
    
    public void setObstructed(MapLocation loc) {
        frontiers.remove(loc);
        if (!obstructed.contains(loc)) {
            obstructed.add(loc);
        }
    }
    
    public void clearLocation(MapLocation loc) {
        frontiers.remove(loc);
        obstructed.remove(loc);
    }
    
    public void addVertex(Vertex vertex) {
        vertices.add(vertex);
    }
    
    public ArrayList<Vertex> getVertices() {
        return vertices;
    }
    
    /**
     * Returns the total number of cells in the map (NOT USED)
     * @return 
     */
    public int getNumberOfCells(){
        return map.size();
    }

    /**
    * Returns the index top row of the map
    * @return 
    */
    public int getTopRow() {
        return topRow;
    }

    /**
    * Returns the index bottom row of the map
    * @return 
    */
    public int getBottomRow() {
        return bottomRow;
    }

    /**
    * Returns the index of the rightmost column of the map
    * @return 
    */
    public int getRightColumn() {
        return rightColumn;
    }

    /**
    * Returns the index of the leftmost column of the map
    * @return 
    */
    public int getLeftColumn() {
        return leftColumn;
    }
    
    /**
     * Returns the size of the cells
     * @return 
     */
    public int getCellSize(){
        return cellSize;
    }
    
    /**
     * Returns the total number of rows
     * @return 
     */
    public int getNumberOfRows(){
        return topRow-bottomRow+1;
    }
    
    /**
     * Returns the total number of columns
     * @return 
     */
    public int getNumberOfColumns(){
        return rightColumn-leftColumn+1;
    }
    
    /**
     * Returns the width of the map in cm (NOT USED)
     * @return 
     */
    public int getMapWidth(){
        return getNumberOfColumns()*cellSize;
    }
    
    /**
     * Returns the height of the map in cm (NOT USED)
     * @return 
     */
    public int getMapHeight(){
        return getNumberOfRows()*cellSize;
    }
    
    /**
     * Returns the cell at the specified location
     * @param location
     * @return 
     */
    public Cell findCell(MapLocation location){
        //
        return map.get(location);
    }

    /**
     * Returns the map
     * @return 
     */
    public ConcurrentHashMap<MapLocation, Cell> getMap(){
        return map;
    }
    
    /**
     * Updates the map. This method also updates the restricted and weakly
     * restricted area of the map if the occupied status of a cell changes.
     * @param location
     * @param measurement 
     */
    public void addMeasurement(MapLocation location, boolean measurement) {
        Cell measuredCell = map.get(location);
        measuredCell.update(measurement);

        // If the cell changes from occupied to free or vice versa, the restricted
        // status of nearby cells are updated here:
        if(measuredCell.stateChanged()){
            ArrayList<MapLocation> restricted = createCircle(location, 15);
            ArrayList<MapLocation> weaklyRestricted = createCircle(location, 25);
            for(MapLocation location2: restricted){
                if(measuredCell.isOccupied()){
                    map.get(location2).addRestrictingCell(measuredCell);
                }
                else {
                    map.get(location2).removeRestrictingCell(measuredCell);
                }
            }
            for(MapLocation location2: weaklyRestricted){
                if(measuredCell.isOccupied()){
                    map.get(location2).addWeaklyRestrictingCell(measuredCell);
                }
                else {
                    map.get(location2).removeWeaklyRestrictingCell(measuredCell);
                }
            }
        }
    }
    
    /**
     * Finds and returns the MapLocation of all cells that are free, not
     * restricted and has an unobserved neighbor
     * @return 
     */
    public ArrayList<MapLocation> getFrontierLocations() {
        ArrayList<MapLocation> frontierLocations = new ArrayList<MapLocation>();
        for(int i = bottomRow; i <= topRow; i++) {
            for(int j = leftColumn; j <= rightColumn; j++) {
                MapLocation location = new MapLocation(i, j);
                if(map.get(location).isWeaklyTargetable()) {
                    ArrayList<MapLocation> neighbors = findDirectNeighborCells(location);
                    for(MapLocation neighbor: neighbors){
                        if(!findCell(neighbor).isPreviouslyObserved()){
                            //cell.setFrontier();
                            frontierLocations.add(location);
                            break;
                        }
                    }
                }
            }
        }
        return frontierLocations;
    }
    
    /**
     * Checks if there are any occupied locations in a given radius around
     * the given location. Used by the SLAMrobot. Radius may have to be
     * adjusted.
     * 
     * @param location
     * @return whether the location is restricted
     */
    public boolean isRestricted(MapLocation location){
        int radius = 15; // cm. 
        radius = radius/cellSize;
        int row = location.getRow();
        int column = location.getColumn();
        int top = row + radius;
        int bottom = row - radius;
        int right = column + radius;
        int left = column - radius;
        for (int i = bottom; i <= top; i++) {
            for (int j = left; j <= right; j++) {
                MapLocation otherLocation = new MapLocation(i, j);
                if (obstructed.contains(otherLocation)) {
                    return true;
                }
            }
        }
        return false;
    }
    
    /**
     * Returns the number of frontier locations in the map.
     * 
     * @return 
     */
    public int getFrontierCount() {
        ArrayList<MapLocation> frontierLocations = getFrontierLocations();
        return frontierLocations.size();
    }
    
    /**
     * Returns the number of occupied cells in the map.
     * 
     * @return 
     */
    public int getOccupiedCount() {
        int count = 0;
        for (int i = bottomRow; i <= topRow; i++) {
            for (int j = 0; j <= rightColumn; j++) {
                MapLocation location = new MapLocation(i, j);
                if (map.get(location).isOccupied()) {
                    count++;
                }
            }
        }
        return count;
    }
    
    /**
     * Returns the number of unexplored cells around the specified location
     * within the specified radius
     * @param location
     * @param radius
     * @return 
     */
    public int countUnknownCellsAroundLocation(MapLocation location, int radius){
        ArrayList<MapLocation> circle = createCircle(location, radius);
        int counter = 0;
        for(MapLocation location2: circle){
            if(!map.get(location2).isPreviouslyObserved()){
                counter++;
            }
        }
        return counter;
    }
    
    /**
     * See findLocationInMap(Position position).
     * @param position
     * @return 
     */
    public MapLocation findLocationInMap(int[] position) {
        return findLocationInMap(new Position(position[0], position[1]));
    }
    
    /**
     * Returns the MapLocation that corresponds to the specified position.
     * @param position
     * @return 
     */
    public MapLocation findLocationInMap(Position position) {
        int row = 0;
        if(position.getYValue() >= 0){
            row = (int)(position.getYValue()/cellSize);
        }else{
            if(position.getYValue()%cellSize == 0){
                row = (int)(position.getYValue()/cellSize);
            }else{
                row = (int)(position.getYValue()/cellSize)-1;
            }
        }
        int column = 0;
        if(position.getXValue() >= 0){
            column = (int)(position.getXValue()/cellSize);
        }else{
            if(position.getXValue()%cellSize == 0){
                column = (int)(position.getXValue()/cellSize);
            }else{
                column = (int)(position.getXValue()/cellSize)-1;
            }
        }
        return new MapLocation(row, column);
    }
    
    /**
     * Returns an array of MapLocations that creates a circle around the 
     * specified location within the specified radius. If any part of the circle
     * exceeds the boundaries of the map, those MapLocations are not included in
     * the array
     * @param location
     * @param radius
     * @return 
     */
    ArrayList<MapLocation> createCircle(MapLocation location, int radius){
        ArrayList<MapLocation> circle = new ArrayList<MapLocation>();
        radius = radius/cellSize;
        int row = location.getRow();
        int column = location.getColumn();
        int top = row + radius;
        if (top > topRow) {
            top = topRow;
        }
        int bottom = row - radius;
        if (bottom < bottomRow) {
            bottom = bottomRow;
        }
        int right = column + radius;
        if (right > rightColumn) {
            right = rightColumn;
        }
        int left = column - radius;
        if (left < leftColumn) {
            left = leftColumn;
        }
        for(int i = bottom; i <= top; i++) {
            for(int j = left; j <= right; j++) {
                MapLocation otherLocation = new MapLocation(i, j);
                if((i-row)*(i-row)+(j-column)*(j-column) <= radius*radius){
                    circle.add(otherLocation);
                }
            }
        }
        return circle;
    }
    
    /**
     * Adds the necessary rows and/or columns to the map so that the specified
     * position is included in the map.
     * @param position 
     */
    public void resize(Position position) {
        MapLocation location = findLocationInMap(position);
        int row = location.getRow();
        int column = location.getColumn();
        if (row > topRow) {
            addRowsTop(row-topRow);
        }
        else if (row < bottomRow) {
            addRowsBottom(Math.abs(row)-Math.abs(bottomRow));
        }
        if (column > rightColumn) {
            addColumnsRight(column-rightColumn);
        }
        else if (column < leftColumn) {
            addColumnsLeft(Math.abs(column)-Math.abs(leftColumn));
        }
    }

    /**
     * Creates the specified amount of rows at the bottom of the map
     * @param numberOfRows 
     */
    private void addRowsBottom(int numberOfRows) {
        for (int i = bottomRow-numberOfRows; i < bottomRow; i++) {
            for (int j = leftColumn; j <= rightColumn; j++) {
                MapLocation location = new MapLocation(i,j);
                map.put(location, new Cell());
                ArrayList<MapLocation> restrictedCircle = createCircle(location, 15);
                for(MapLocation otherLoc: restrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
                ArrayList<MapLocation> weaklyRestrictedCircle = createCircle(location, 25);
                for(MapLocation otherLoc: weaklyRestrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addWeaklyRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
            }
        }
        bottomRow -= numberOfRows;
    }
    
     /**
     * Creates the specified amount of rows at the top of the map
     * @param numberOfRows 
     */
    private void addRowsTop(int numberOfRows) {
        for (int i = topRow+1; i <= topRow+numberOfRows; i++) {
            for (int j = leftColumn; j <= rightColumn; j++) {
                MapLocation location = new MapLocation(i,j);
                map.put(location, new Cell());
                ArrayList<MapLocation> restrictedCircle = createCircle(location, 15);
                for(MapLocation otherLoc: restrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
                ArrayList<MapLocation> weaklyRestrictedCircle = createCircle(location, 25);
                for(MapLocation otherLoc: weaklyRestrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addWeaklyRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
            }
        }
        topRow += numberOfRows;
    }
    
     /**
     * Creates the specified amount of columns at the left side of the map
     * @param numberOfRows 
     */
    private void addColumnsLeft(int numberOfColumns) {
        for (int i = bottomRow; i <= topRow; i++) {
            for (int j = leftColumn-numberOfColumns; j < leftColumn; j++) {
                MapLocation location = new MapLocation(i,j);
                map.put(location, new Cell());
                ArrayList<MapLocation> restrictedCircle = createCircle(location, 15);
                for(MapLocation otherLoc: restrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
                ArrayList<MapLocation> weaklyRestrictedCircle = createCircle(location, 25);
                for(MapLocation otherLoc: weaklyRestrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addWeaklyRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
            }
        }
        leftColumn -= numberOfColumns;
    }
    
    /**
     * Creates the specified amount of columns at the right side of the map
     * @param numberOfRows 
     */
    private void addColumnsRight(int numberOfColumns) {
        for (int i = bottomRow; i <= topRow; i++) {
            for (int j = rightColumn+1; j <= rightColumn+numberOfColumns; j++) {
                MapLocation location = new MapLocation(i,j);
                map.put(location, new Cell());
                ArrayList<MapLocation> restrictedCircle = createCircle(location, 15);
                for(MapLocation otherLoc: restrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
                ArrayList<MapLocation> weaklyRestrictedCircle = createCircle(location, 25);
                for(MapLocation otherLoc: weaklyRestrictedCircle){
                    if(map.get(otherLoc).isOccupied()){
                        map.get(location).addWeaklyRestrictingCell(map.get(otherLoc));
                        break;
                    }
                }
            }
        }
        rightColumn += numberOfColumns;
    }
    
    /**
     * Creates and returns new MapLocations that are directly next to the
     * specified location. Does not return MapLocations that are outside the
     * map's boundaries
     * @param location
     * @return 
     */
    public ArrayList<MapLocation> findDirectNeighborCells(MapLocation location){
        ArrayList<MapLocation> neighbors = new ArrayList<MapLocation>();
        int row = location.getRow();
        int column = location.getColumn();
        if(row + 1 <= topRow){ // Above
            neighbors.add(new MapLocation(row+1, column));
        }
        if(column + 1 <= rightColumn){ // Right
            neighbors.add(new MapLocation(row, column+1));
        }
        if(row - 1 >= bottomRow){ // Below
            neighbors.add(new MapLocation(row-1, column));
        }
        if(column - 1 >= leftColumn){ // Left
            neighbors.add(new MapLocation(row, column-1));
        }
        return neighbors;
    }
    
    
    /**
     * Creates and returns new MapLocations that are diagonally connected to the
     * specified location. Does not return MapLocations that are outside the
     * map's boundaries
     * @param location
     * @return 
     */
    public ArrayList<MapLocation> findDiagonalNeighborCells(MapLocation location){
        ArrayList<MapLocation> neighbors = new ArrayList<MapLocation>();
        int row = location.getRow();
        int column = location.getColumn();
        if(row + 1 <= topRow && column + 1 <= rightColumn){ // Above right
            neighbors.add(new MapLocation(row+1, column+1));
        }
        if(row + 1 <= topRow && column - 1 >= leftColumn){ // Above left 
            neighbors.add(new MapLocation(row+1, column-1));
        }
        if(row - 1 >= bottomRow && column + 1 <= rightColumn){ // Below right
            neighbors.add(new MapLocation(row-1, column+1));
        }
        if(row - 1 >= bottomRow && column - 1 >= leftColumn){ // Below left
            neighbors.add(new MapLocation(row-1, column-1));
        }
        return neighbors;
    }
    
    /**
     * Returns the position position of the center of the specified MapLocation
     * @param location
     * @return 
     */
    public Position mapLocation2Position(MapLocation location){
        return new Position((double)(location.getColumn()*cellSize)+(double)cellSize/2, (double)(location.getRow()*cellSize)+(double)cellSize/2);
    }
    
    /**
     * Fills in small unexplored areas of the map with free space
     */
    public void cleanUp(){
        for(int i = bottomRow; i <= topRow; i++) {
            for(int j = leftColumn; j <= rightColumn; j++) {
                MapLocation location = new MapLocation(i, j);
                if(!map.get(location).isPreviouslyObserved()){
                    ArrayList<MapLocation> neighbors = findDirectNeighborCells(location);
                    for(MapLocation neighbor: neighbors){
                        if(map.get(neighbor).isPreviouslyObserved()){
                            if(countUnknownCellsAroundLocation(location, 5)*cellSize*cellSize < 23){
                                map.get(location).update(false);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
        /**
     * Function to make lines from point clouds.
     * 
     * @author Henrik
     */
    public void cleanMap(){
        //Split map into x*y number of pxp sized submaps
        int p = 5;
        //Expand map to avoid unprocessed edge residues (if height % p != 0 for instance)
        addColumnsRight(p-1);
        addRowsTop(p-1);
        int rows = ((topRow-bottomRow+1)/p);
        int cols = ((rightColumn-leftColumn+1)/p);
        ArrayList<MapLocation> centerPointCells = new  ArrayList<MapLocation>();
        //Itterate over submaps and find center of submaps
        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                GridMap subMap = new GridMap(cellSize,p*cellSize,p*cellSize);
                int points = 0;
                //Itterate over locations and transfer to submap
                int n = 0;
                for (int i = bottomRow+p*x; i < bottomRow+p+p*x; i++) {
                    int m = 0;
                    for (int j = leftColumn+p*y; j < leftColumn+p+p*y; j++) {
                        Cell cell = map.get(new MapLocation(i,j));
                        if(cell.isOccupied()){
                            points++;
                            subMap.addMeasurement(new MapLocation(n,m), true);
                        }
                        m++;
                    }
                    n++;
                }
                for (int i = bottomRow+p*x; i < bottomRow+p+p*x; i++) {
                    for (int j = leftColumn+p*y; j < leftColumn+p+p*y; j++) {
                        Cell measuredCell = map.get(new MapLocation(i,j));
                        if(measuredCell.isPreviouslyObserved()){
                            measuredCell.update(false);
                        }
                        if(i == bottomRow+p*x+(p/2) && j == leftColumn+p*y+(p/2)
                            && points > 1){
                            centerPointCells.add(new MapLocation(i,j));
                        }
                    }
                }
            }
        }
        //use centerpoints to make lines
        for(MapLocation startLocation : centerPointCells){
            MapLocation endLocation = startLocation;
            for(MapLocation point : centerPointCells){
                if(Math.abs(point.getRow() - startLocation.getRow()) < (p+1) 
                    && Math.abs(point.getColumn() - startLocation.getColumn()) < (p+1) ){
                    endLocation = point;
                    ArrayList<MapLocation> line = MappingController.getLineBetweenPoints(startLocation, endLocation);
                    for(MapLocation lineSegment : line){
                        Cell linePoint = map.get(lineSegment);
                        linePoint.update(true);
                    }                
                }
            }
        }
    }
    
    /**
     * Adds locations of the particles in the filter to the map.
     * 
     * @param location 
     */
    public void addParticle(MapLocation location) {
        if(map.get(location) != null && !(map.get(location).isOccupied())) {
            Cell measuredCell = map.get(location);
            measuredCell.setParticle();
        }
    }
    
    /**
     * Removes the outdated particles from the map.
     * 
     * @param location 
     */
    public void removeParticle(MapLocation location) {
        if(map.get(location) != null) {
            Cell measuredCell = map.get(location);
            measuredCell.removeParticle();
        }
    }
    
    /**
     * Add discovered vertice to the map.
     * 
     * @param location
     * @param type 0 = exterior, 1 = interior
     */
    public void addVertice(MapLocation location, int type) {
        int radius = 5;
        ArrayList<MapLocation> circle = createCircle(location, 5);
    }
}
