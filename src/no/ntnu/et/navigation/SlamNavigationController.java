/**
 * This code is written as a part of a Master Thesis
 * the fall of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.et.navigation;

import static no.ntnu.et.general.Navigation.getShortestDistanceAngle;
import no.ntnu.et.general.Position;
import static no.ntnu.et.general.Utilities.getMeasurementHeadings;
import no.ntnu.et.map.GridMap;
import no.ntnu.tem.application.Application;
import no.ntnu.tem.application.RobotController;
import no.ntnu.tem.robot.Measurement;
import no.ntnu.tem.robot.Robot;

/**
 *
 * @author geirhei
 */
public class SlamNavigationController extends NavigationController {
    //private ConcurrentLinkedQueue<Measurement> slamMeasurements;
    private int[] distances;
    private final String robotName = "SLAM";
    //private final Robot applicationRobot;
    private NavigationRobot navigationRobot;
    private int id;
    private boolean busy = false;
    
    private boolean paused; // hides superclass field
    private final boolean debug = false; // hides superclass field
    
    public SlamNavigationController(RobotController robotController, Application application, GridMap map) {
        super(robotController, application, map);
        //applicationRobot = super.getRobotController().getRobot(robotName);
        //id = applicationRobot.getId();
        navigationRobot = new NavigationRobot();
        //slamMeasurements = new ConcurrentLinkedQueue<>();
        distances = new int[360];
        for (int i = 0; i < distances.length; i++) {
            distances[i] = Integer.MAX_VALUE;
        }
    }
    
    public void addRobot(int id) {
        Position currentPosition = new Position(super.getRobotController().getRobot(robotName).getPosition());
        navigationRobot = new NavigationRobot(currentPosition);
        this.id = id;
        //collisionManager.addRobot(robotName, newNavRobot);
    }
    
    private void stopRobot() {
        getApplication().unPauseRobot(id);
    }
    
    
    @Override
    public void start() {
        if (!isAlive()) {
            super.start();
        }
        resumeRobot();
        paused = false;
    }
    
    private void resumeRobot() {
        super.getApplication().unPauseRobot(id);
    }
    
    
    @Override
    public void run() {
        setName("Slam Navigation Controller");
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        boolean finished = false;
        while (!finished) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                stopRobot();
                break;
            }
            if (paused) {
                continue;
            }
            
            Robot applicationRobot = super.getRobotController().getRobot(robotName);
            if (applicationRobot.isInManualMode()) {
                continue;
            }
            
            /* Measurement handling ************/
            Measurement m = applicationRobot.getSlamMeasurement();
            while (m != null) {
                int[] irHeadings = m.getIRHeading();
                //int robotHeading = m.getTheta();
                int[] measurementHeadings = getMeasurementHeadings(irHeadings);
                int[] irData = m.getIRdata();
                //System.out.println(irData[0] + ", " + irData[1]+ ", " + irData[2]+ ", " + irData[3]);
                for (int j = 0; j < measurementHeadings.length; j++) {
                    if (irData[j] == 0 || irData[j] > 80) {
                        distances[measurementHeadings[j]] = Integer.MAX_VALUE;
                    } else {
                        if (irData[j] == 0) {
                            //System.out.println(irData[j]);
                        }
                        distances[measurementHeadings[j]] = irData[j];
                    }
                }
                m = applicationRobot.getSlamMeasurement();
            }
            /**********************************/

            if (!busy) {
                int[] newCommand = new int[]{50, -50};
                super.writeCommandToRobot(id, robotName, newCommand[0], newCommand[1]);
            }
            
            
            while (true) {
                for (int i = 0; i < 6; i++) {
                    if (distances[267 + i] < 30) {
                        break;
                    }
                }
                break;
            }
            
            int[] newCommand = new int[]{0, 0};
            super.writeCommandToRobot(id, robotName, newCommand[0], newCommand[1]);
            busy = true;
            /*
            int[] nextCommand;
            if (distances[90] < 60) {
                nextCommand = applicationRobot.getPosition();
            } else {
                nextCommand = new int[]{50, 0};
            }
            application.writeCommandToRobot(id, name, nextCommand[0], nextCommand[1]);
            */

            System.out.println("Distances: " + distances[90] + ", " + distances[91] + ", " + distances[92] + ", " + distances[93] + ", "+ distances[94] + ", " + distances[95] + ", " + distances[96] + ", " + distances[97]);
            int shortestAngle = getShortestDistanceAngle(distances);
            //System.out.println("Angle: " + shortestAngle);
            //System.out.println("distances[120]: " + distances[120]);
            //break;
            
            
            /**************** Original algorithm ******************
            if (navigationRobot.hasNewPriorityCommand()) {
                int[] nextCommand = navigationRobot.getPriorityCommand();
                if (debug) {
                    System.out.println("Priority command: " + nextCommand[0] + "," + nextCommand[1]);
                }
                writeCommandToRobot(id, robotName, nextCommand[0], nextCommand[1]);
            } else if (!applicationRobot.isBusy() && !navigationRobot.isInCollisionManagement()) {
                if (navigationRobot.hasMoreWaypoints()) {
                    // Get next target
                    Position nextWaypoint = navigationRobot.getNextWaypoint();
                    int[] newCommand = {(int) nextWaypoint.getXValue(), (int) nextWaypoint.getYValue()};
                    if (debug) {
                        System.out.println("Sending waypoint: " + newCommand[0] + "," + newCommand[1]);
                    }
                    writeCommandToRobot(id, robotName, newCommand[0], newCommand[1]);
                } else if (!getRobotTaskManager().isWorkingOnTask(robotName)) {
                    if (debug) {
                        System.out.println(robotName + ": Idle. Searching for best target");
                    }
                    getRobotTaskManager().createNewTask(applicationRobot, navigationRobot, robotName);
                }
            }
            */
        }
    }
}
