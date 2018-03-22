/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import java.util.concurrent.ConcurrentHashMap;
import javax.swing.JPanel;
import no.ntnu.et.general.Line;
import no.ntnu.et.general.Vertex;
import no.ntnu.et.map.Cell;
import no.ntnu.et.map.GridMap;
import no.ntnu.et.map.MapLocation;
import no.ntnu.et.simulator.SimRobot;
import no.ntnu.tem.application.RobotController;
import no.ntnu.tem.robot.Robot;

/**
 * This class provides functionality that draws the map shown in the main GUI.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class MapGraphic extends JPanel {

    protected GridMap gridmap;
    private RobotController rc;
    protected Dimension d;
    boolean debug = false;
    int numberOfRows;
    int numberOfColumns;
    int cellSize;
    protected Double scrollSize = 1.0;
    int origo;
    protected AffineTransform temp;
    protected AffineTransform initial;

    /**
     * Constructor for the class MapGraphic
     *
     * @param gridmap the gridmap to show in the GUI
     * @param rc the systems robotcontroller
     */
    public MapGraphic(GridMap gridmap, RobotController rc) {
        this.gridmap = gridmap;
        this.rc = rc;
        this.cellSize = gridmap.getCellSize();
        d = new Dimension(gridmap.getNumberOfColumns() * cellSize, gridmap.getNumberOfRows() * cellSize);
        setSize(d);

        addMouseWheelListener(new java.awt.event.MouseWheelListener() {
            public void mouseWheelMoved(java.awt.event.MouseWheelEvent evt) {
                scroll(evt.getUnitsToScroll());
            }
        });

    }

    /**
     * Changes the graphical size of the map.
     *
     * @param scroll units to scroll.
     */
    private void scroll(double scroll) {
        if (scrollSize > 1 && scroll > 0 || scrollSize < 5 && scroll < 0) {
            scroll = scroll / Math.abs(scroll);
            scrollSize = scrollSize - scroll;
        }
    }

    /**
     * Paints the entire map and the robots.
     *
     * @param g Graphics component.
     */
    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        if (debug) {
            System.out.println("columns: " + gridmap.getNumberOfColumns() + ", rows: " + gridmap.getNumberOfRows());
        }
        d = new Dimension(gridmap.getNumberOfColumns() * cellSize * scrollSize.intValue(), gridmap.getNumberOfRows() * cellSize * scrollSize.intValue());
        setSize(d);
        Graphics2D g2D = (Graphics2D) g;
        // Save the initial transform
        initial = g2D.getTransform();
        // Turn the graphic object so that the origin is in the bottom left corner instead of the top left corner
        g2D.translate(0, getHeight() - 1);
        g2D.scale(1, -1);
        temp = g2D.getTransform();
        numberOfRows = gridmap.getNumberOfRows();
        numberOfColumns = gridmap.getNumberOfColumns();
        cellSize = gridmap.getCellSize();
        paintMap(g2D);
        paintLines(g2D);
        paintRobots(g2D);
        g2D.setTransform(initial);

    }

    /**
     * Paints the map
     *
     * @param g2D The Graphics2D object
     */
    private void paintMap(Graphics2D g2D) {
        ConcurrentHashMap<MapLocation, Cell> map = gridmap.getMap();
        int lowest = gridmap.getBottomRow();
        int leftmost = gridmap.getLeftColumn();
        for (ConcurrentHashMap.Entry<MapLocation, Cell> entry : map.entrySet()) {
            Cell cell = entry.getValue();
            if (!cell.isPreviouslyObserved()) {
                g2D.setPaint(Color.gray);
            } else if (cell.isOccupied()) {
                g2D.setPaint(Color.black);
            } else if (cell.isRestricted()) {
                g2D.setPaint(Color.lightGray);
            } /*else if (cell.isWeaklyRestricted()) {
                g2D.setPaint(Color.yellow);
            } */ else if (cell.isTarget()) {
                g2D.setPaint(Color.blue);
            } else if (cell.isPath()) {
                g2D.setPaint(Color.green);
            } else if (cell.isFree()) {
                g2D.setPaint(Color.white);
            } else {
                g2D.setPaint(Color.yellow);
            }
            if (cell.isParticle()) {
                g2D.setPaint(Color.cyan); 
            }
            g2D.fillRect((entry.getKey().getColumn() - leftmost) * cellSize * scrollSize.intValue(), (entry.getKey().getRow() - lowest) * cellSize * scrollSize.intValue(), cellSize * scrollSize.intValue(), cellSize * scrollSize.intValue());
        }
    }
    
    /**
     * Paint all lines in the line repository
     * 
     * @param g2D 
     */
    private void paintLines(Graphics2D g2D) {
        g2D.setPaint(Color.black);
        List<Line> lines = gridmap.getLineRepository();
        //ArrayList<ArrayList<Line>> lineBuffers = gridmap.getLineBuffers();
        synchronized (lines) {
            ListIterator<Line> iter = lines.listIterator();
            while (iter.hasNext()) {
                Line line = iter.next();
                g2D.drawLine((int) line.getA().getXValue(), (int) line.getA().getYValue(), (int) line.getB().getXValue(), (int) line.getB().getYValue());
            }
        }
    }
    
    /**
     * Paints the robots
     *
     * @param g2D The Graphics2D object
     */
    protected void paintRobots(Graphics2D g2D) {
        for (Robot robot : rc.getRobotList()) {

            Color robotColor = selectRobotColor(g2D, robot.getId());
            g2D.setPaint(robotColor);
            g2D.fillOval((robot.getPosition()[0] - gridmap.getLeftColumn() * cellSize - 7) * scrollSize.intValue(), (robot.getPosition()[1] - gridmap.getBottomRow() * cellSize - 7) * scrollSize.intValue(), 15 * scrollSize.intValue(), 15 * scrollSize.intValue());
            g2D.setStroke(new BasicStroke(3));

            int posx = (robot.getPosition()[0] - gridmap.getLeftColumn() * cellSize) * scrollSize.intValue();
            int posy = (robot.getPosition()[1] - gridmap.getBottomRow() * cellSize) * scrollSize.intValue();
            int destx = (robot.getDestination()[0] - gridmap.getLeftColumn() * cellSize-7) * scrollSize.intValue();
            int desty = (robot.getDestination()[1] - gridmap.getBottomRow() * cellSize-7) * scrollSize.intValue();
            int angle = robot.getRobotOrientation();
            double rad = angle * Math.PI / (180);
            Shape r = new Line2D.Double(posx, posy, posx + 12 * Math.cos(rad) * scrollSize.intValue(), posy + 12 * Math.sin(rad) * scrollSize.intValue());
            g2D.setStroke(new BasicStroke(scrollSize.intValue() * 3));
            g2D.draw(r);
            g2D.setPaint(Color.black);
            Font f = new Font("Sans Serif", Font.PLAIN, -20 * scrollSize.intValue());
            g2D.setFont(f);
            g2D.translate(getWidth() - 1, 0);
            g2D.scale(-1, 1);
            g2D.drawString(robot.getName(), getWidth() - posx, posy - 20 * scrollSize.intValue());
            g2D.setPaint(robotColor);
            g2D.drawString("X", getWidth() - destx, desty);
            g2D.setTransform(temp);
        }
    }

    /**
     * Sets an unique color to each robot
     *
     * @param g Graphical element
     * @param i The ID of the robot
     * @return an unique color
     */
    static Color selectRobotColor(Graphics2D g, int i) {
        switch (i % 10) {
            case 0:
                return Color.blue;
            case 1:
                return Color.red;
            case 2:
                return Color.green;
            case 3:
                return Color.pink;
            case 4:
                return Color.orange;
            case 5:
                return Color.cyan;
            case 6:
                return Color.pink;
            case 7:
                return Color.magenta;
            case 8:
                return Color.darkGray;
            case 9:
                return Color.lightGray;
            default:
                return Color.black;
        }
    }

}
