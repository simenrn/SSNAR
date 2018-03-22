package no.ntnu.et.simulator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;
import no.ntnu.et.general.Angle;
import no.ntnu.et.general.Pose;
import no.ntnu.et.general.Line;


public class Drone extends SimRobot{
    private int height = 100; // Unused as of now
    private int frameHeight = 250;
    private int frameWidth = 250;
    private SimWorld world;
    
    Drone(SimWorld world, Pose initialPose, String name, int id, int address) {
        super(world,initialPose,name,id,address);
        this.world = world; 
    }
        
    /**
     * Way to manipulate drone, it now teleports
     */
    @Override
    boolean moveRobot(double noise){
        if (!rotationFinished) {
            System.out.println("Moveing");
            if (Math.abs(measuredRotation) >= Math.abs(targetRotation)){
                measuredRotation = 0;
                rotationFinished = true;
                return false;
            }
            pose.rotate(new Angle(targetRotation));
            estimatedPose.rotate(new Angle(targetRotation));
            measuredRotation += targetRotation;
        } else if (!translationFinished) {
            if(Math.abs(measuredDistance) >= Math.abs(targetDistance)) {
                measuredDistance = 0;
                translationFinished = true;
                return false;
            }
            estimatedPose.move(targetDistance);
            measuredDistance += targetDistance;
            pose.move(targetDistance);
        }
        return false;
    }
    
    @Override
    int[] createMeasurement() {
        ArrayList<Feature> features =  world.getFeatures();
        //Returns start and end of lines
        double lineStartX = 0;
        double lineStartY = 0;
        double lineStopX = 0;
        double lineStopY = 0;

        //Frame edges of photo
        //LS = LeftSide, B = Bottom etc.
        double frameLS = getPose().getPosition().getXValue()-(frameWidth/2);
        double frameRS = getPose().getPosition().getXValue()+(frameWidth/2);
        double frameT = getPose().getPosition().getYValue()+(frameHeight/2);
        double frameB = getPose().getPosition().getYValue()-(frameHeight/2);
        
        double startX = features.get(0).getStartPosition().getXValue();
        double startY = features.get(0).getStartPosition().getYValue();
        double endX = features.get(0).getEndPosition().getXValue();
        double endY = features.get(0).getEndPosition().getYValue();
        
        Line line = Line.convertFeatureToLine(features.get(0));
        double[] dir = line.getDirection();
        
        mainLoop:
        //Do until points meet
        while(!((startX == endX) && (startY == endY))) {
            //If Start point is inside frame
            if((startX >= frameLS && startY >= frameB)
            && (startX <= frameRS && startY <= frameT)) {
                lineStartX = startX;
                lineStartY = startY;
                //Until points meet
                while(!((startX == endX) && (startY == endY))) {
                    lineStopX = endX;
                    lineStopY = endY;
                    //If end point is innside frame
                    if((endX >= frameLS && endY >= frameB)
                        && (endX <= frameRS && endY <= frameT)){
                        //Both points are innside, break out of main loop
                        break mainLoop;
                    }
                    //Else move endpoint towards start point
                    else{
                        endX -= dir[0];
                        endY -= dir[1];
                    }
                }
                break;
            }
            //Else move start point towards end point
            else{
                startX += dir[0];
                startY += dir[1];
            }
        }
        
        //Randomize list so we don't always chooses the same line, probably sub-optimal
        Random rnd = new Random();
        Collections.shuffle(features,rnd);
        //assume same structure as other updates, IR = start and end of line (x,y,x,y)
        int[] measurement = new int[8];
        measurement[0] = (int)getEstimatedPose().getPosition().getXValue(); 
        measurement[1] = (int)getEstimatedPose().getPosition().getYValue(); 
        measurement[2] = (int)getEstimatedPose().getHeading().getValue(); 
        measurement[3] = 0; 
        measurement[4] = (int)lineStartX;
        measurement[5] = (int)lineStartY;
        measurement[6] = (int)lineStopX;
        measurement[7] = (int)lineStopY;
        return measurement;
    }
    
}
