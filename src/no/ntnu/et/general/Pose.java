/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.general;
import java.awt.Graphics2D;

/**
 * This class is used to represent the pose of a robot. The pose consists of
 * the position and orientation of the robot.
 * 
 * @author Eirik Thon
 */
public class Pose {
    
    private Position position;
    
    private Angle heading;
    
    /**
     * Create a new pose object with all values set to 0
     */
    public Pose(){
        position = new Position();
        heading = new Angle();
    }
    
    /**
     * Creates a new pose object with values specified by the input parameters
     * @param initXPos
     * @param initYPos
     * @param initOrientation 
     */
    public Pose(int initXPos, int initYPos, int initOrientation){
        position = new Position(initXPos, initYPos);
        heading = new Angle(initOrientation);
    }
    
    /**
     * Creates a new pose object with values specified by the input parameters
     * @param initialPosition
     * @param initialHeading 
     */
    public Pose(Position initialPosition, Angle initialHeading){
        position = initialPosition;
        heading = initialHeading;
    }
    
    /**
     * Copy constructor
     * @param pose 
     */
    public Pose(Pose pose){
        this.position = new Position(pose.getPosition().getXValue(),pose.getPosition().getYValue());
        this.heading = new Angle(pose.getHeading().getValue());
    }
    /**
     * Returns the position of the Pose object
     * @return 
     */
    public Position getPosition() {
        return position;
    }

    /**
     * Returns the orientation of the Pose object.
     * @return 
     */
    public Angle getHeading() {
        return heading;
    }
    
    /**
     * Changes heading
     * @param heading 
     */
    public void setHeading(Angle heading) {
        this.heading = heading;
    } 
    
    /**
     * Creates a new Pose object that is a deep copy of the object given as an
     * input parameter
     * @param pose
     * @return Pose
     */
    public static Pose copy(Pose pose){
        return new Pose(Position.copy(pose.position), Angle.copy(pose.heading));
    }
    
    /**
     * Paints the pose onto a Graphics2D object. The pose is painted as a circle
     * at the position and a line indicating the heading.
     * @param g2D Graphics2D object to paint upon.
     */
    public void paint(Graphics2D g2D, double scale){
        position.drawCircle(g2D, 10, scale);
        heading.drawAngle(g2D, position.getXValue(), position.getYValue(), 7, scale);
    }
    
    /**
     * Adds the input Angle to the Angle of the Pose object
     * @param rotation 
     */
    public void rotate(Angle rotation) {
        heading.add(rotation.getValue());
    }
    
    /**
     * Moves the position of the pose object the given distance in the direction
     * of the current Angle
     * @param distance 
     */
    public void move(double distance) {
        Position offset = Utilities.polar2cart(heading, distance);
        position.add(offset);
    }
    
    /**
     * Creates a new Pose object from the sum of the positions and the
     * angles of the Pose objects in the parameters
     * @param pose1
     * @param pose2
     * @return 
     */
    public static Pose sum(Pose pose1, Pose pose2) {
        Position newPosition = Position.sum(pose1.position, pose2.position);
        Angle newAngle = Angle.sum(pose1.heading, pose2.heading);
        return new Pose(newPosition, newAngle);
    }
    
    /**
     * The Pose object is transformed using a rotational matrix and the heading
     * of the input Pose. Then the input Pose is added to the Pose.
     * @param otherPose 
     */
    public void transform(Pose otherPose){
        position.transform(otherPose.heading);
        position.add(otherPose.position);
        heading.add(otherPose.heading);
    }
}
