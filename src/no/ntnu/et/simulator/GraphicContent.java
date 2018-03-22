/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.simulator;

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
import java.util.LinkedList;
import java.util.concurrent.ConcurrentHashMap;
import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Position;
import no.ntnu.et.general.Utilities;
import no.ntnu.et.map.Cell;
import no.ntnu.et.map.MapLocation;
import no.ntnu.tem.robot.Robot;

/**
 * This class is used for displaying the graphical content of the simulator 
 * by calling repaint on this class . It is inspired by the map graphic class
 * created by Thor Eivind Andersen and Mats Rødseth.
 * 
 * @author Eirik Thon
 */
public class GraphicContent extends javax.swing.JPanel {
    private boolean showTargets;
    private boolean showEstimatedPoses;
    private boolean showSensorBeams;
    private double scale;
    private SimWorld world;
    
    /**
     * Creates new form Graphics
     */
    public GraphicContent(SimWorld world) {
        initComponents();
        setBackground(Color.white);  
        scale = 1;
        showEstimatedPoses = false;
        showTargets = false;
        showSensorBeams = false;
        Dimension dimension = new Dimension((int)Math.round(world.getMapWidth() * scale + 1), (int)Math.round(world.getMapHeight() * scale + 1));
        setSize(dimension);
        this.world = world;
    }
    
    /**
     * Switches an internal state in the SimWorld so that the robots target
     * position are being painted or stops being painted
     */
    void flipShowTarget() {
        showTargets = !showTargets;
    }
    
    /**
     * Switches an internal state in the SimWorld so that the robots estimated
     * poses are being painted or stops being painted
     */
    void flipShowEstimatedPose() {
        showEstimatedPoses = !showEstimatedPoses;
    }
    
    void flipShowSensorBeams() {
        showSensorBeams = !showSensorBeams;
    }
    
    double getScale(){
        return scale;
    }
    
    void changeScale(int scaleChange) {
        if(scaleChange < 0 && scale > 0.1){
            scale += (double)scaleChange/100;
            Dimension dimension = new Dimension((int)Math.round(world.getMapWidth() * scale + 1), (int)Math.round(world.getMapHeight() * scale + 1));
            setSize(dimension);
        }
        else if(scaleChange > 0 && scale < 10){
            scale += (double)scaleChange/50;
            Dimension dimension = new Dimension((int)Math.round(world.getMapWidth() * scale + 1), (int)Math.round(world.getMapHeight() * scale + 1));
            setSize(dimension);
        }
    }
    
    void updateDimension(){
        Dimension dimension = new Dimension((int)Math.round(world.getMapWidth() * scale + 1), (int)Math.round(world.getMapHeight() * scale + 1));
        setSize(dimension);
    }
    
    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2D = (Graphics2D) g;

        // Save the initial transform
        AffineTransform initial = g2D.getTransform();
        
        // Turn the graphic object so that the origin is in the bottom left corner instead of the top left corner
        g2D.translate(0, getHeight() - 1);
        g2D.scale(1, -1);
        
        //Paint content
        paintMap(g2D);
        paintRobots(g2D);
        if(showEstimatedPoses) {
            paintEstimatedPositions(g2D);
        }
        if(showTargets){
            paintTargetPositions(g2D);
        }
        if(showSensorBeams) {
            paintSensorBeams(g2D);
        }
        
        // Reset transform
        g2D.setTransform(initial);
    }
    
    private void paintMap(Graphics2D g2D) {
        //Paint features
        ArrayList<Feature> features = world.getFeatures();
        g2D.setPaint(Color.black);
        for (int i = 0; i < features.size (); i++){
            features.get(i).paint(g2D, scale);
        }
    }
    
    private void paintRobots(Graphics2D g2D) {
        ArrayList<SimRobot> robots = world.getRobots();
        for(int i = 0; i < robots.size(); i++){
            Color color = Utilities.selectColor(i);
            g2D.setPaint(color);
            robots.get(i).getPose().paint(g2D, scale);
        }
    }
    
    private void paintEstimatedPositions(Graphics2D g2D){
        ArrayList<SimRobot> robots = world.getRobots();
        for(int i = 0; i < robots.size(); i++){
            Color color = Utilities.selectColor(i);
            g2D.setPaint(color);
            Pose relativeEstimatedPose = Pose.copy(robots.get(i).getEstimatedPose());
            Pose initialPose = robots.get(i).getInitialPose();
            relativeEstimatedPose.getPosition().transform(initialPose.getHeading());
            Pose estimatedPose = Pose.sum(relativeEstimatedPose, initialPose);
            estimatedPose.paint(g2D, scale);
        }
    }

    private void paintTargetPositions(Graphics2D g2D){
        ArrayList<SimRobot> robots = world.getRobots();
        for(int i = 0; i < robots.size(); i++){
            Color color = Utilities.selectColor(i);
            g2D.setPaint(color);
            Position target;
            target = robots.get(i).getTargetPosition();
            target.drawCross(g2D, 3, scale);
        }
    }
    
    private void paintSensorBeams(Graphics2D g2D){
        ArrayList<SimRobot> robots = world.getRobots();
        for(int i = 0; i < robots.size(); i++){
            Color color = Utilities.selectColor(i);
            g2D.setPaint(color);
            Angle lineOfSightAngle = Angle.sum(robots.get(i).getTowerAngle(), robots.get(i).getPose().getHeading());
            for (int j = 0; j < 4; j++) {
                // Create a feature along the line of sight for each sensor
                lineOfSightAngle.add(j * 90);
                Position offset = Utilities.polar2cart(lineOfSightAngle, robots.get(i).getMaxSensorDistance());
                Position lineOfSightEnd = Position.sum(robots.get(i).getPose().getPosition(), offset);
                g2D.drawLine((int)(robots.get(i).getPose().getPosition().getXValue()*scale), (int)(robots.get(i).getPose().getPosition().getYValue()*scale), (int)(lineOfSightEnd.getXValue()*scale), (int)(lineOfSightEnd.getYValue()*scale));
            }
        }
    }
    
    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 377, Short.MAX_VALUE)
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 317, Short.MAX_VALUE)
        );
    }// </editor-fold>//GEN-END:initComponents


    // Variables declaration - do not modify//GEN-BEGIN:variables
    // End of variables declaration//GEN-END:variables
}
