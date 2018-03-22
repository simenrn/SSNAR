/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.navigation;

/**
 * This class checks for commands in all of the Navigation Robots and sends them
 * to the robots via the Application. If a robot has no new commands this class
 * creates a new worker thread in RobotTaskManager to find a new task for the
 * robot.
 *
 * @author Eirik Thon
 */
import java.util.ArrayList;
import java.util.HashMap;
import static no.ntnu.et.general.Navigation.getShortestDistanceAngle;
import no.ntnu.et.general.Position;
import static no.ntnu.et.general.Utilities.getMeasurementHeadings;
import no.ntnu.et.map.GridMap;
import no.ntnu.tem.application.Application;
import no.ntnu.tem.application.RobotController;
import no.ntnu.tem.robot.Measurement;
import no.ntnu.tem.robot.Robot;
import org.ejml.simple.SimpleMatrix;

/**
 *
 * @author Eirik Thon
 */
public class NavigationController extends Thread {

    private CollisionManager collisionManager;

    private RobotTaskManager robotTaskManager;

    private RobotController robotController;

    private Application application;

    private HashMap<String, NavigationRobot> navigationRobots;

    private ArrayList<String> robotNames;

    private boolean paused;

    private boolean debug = false;
    
    

    public NavigationController(RobotController robotController, Application application, GridMap map) {
        this.robotController = robotController;
        this.application = application;
        robotTaskManager = new RobotTaskManager(map);
        collisionManager = new CollisionManager(map, robotController);
        collisionManager.setName("Collision management");
        robotNames = new ArrayList<String>();
        navigationRobots = new HashMap<String, NavigationRobot>();
        
        
    }

    public void addRobot(String robotName, int id) {
        Position currentPosition = new Position(robotController.getRobot(robotName).getPosition());
        NavigationRobot newNavRobot = new NavigationRobot(currentPosition);
        navigationRobots.put(robotName, newNavRobot);
        robotNames.add(robotName);
        collisionManager.addRobot(robotName, newNavRobot);
    }

    public void removeRobot(String robotName) {
        robotNames.remove(robotName);
        collisionManager.removeRobot(robotName);
    }

    @Override
    public void start() {
        if (!isAlive()) {
            super.start();
            collisionManager.start();
        } else {
            collisionManager.unpause();
        }
        resumeAllRobots();
        paused = false;
    }

    public void pause() {
        collisionManager.pause();
        stopAllRobots();
        paused = true;
    }
    
    boolean isPaused() {
        return paused;
    }

    public void quit() {
        paused = true;
    }
    
    boolean debugEnabled() {
        return debug;
    }
    
    NavigationRobot getNavigationRobot(String name) {
        return navigationRobots.get(name);
    }
    
    RobotTaskManager getRobotTaskManager() {
        return robotTaskManager;
    }
    
    void writeCommandToRobot(int id, String name, int x, int y) {
        application.writeCommandToRobot(id, name, x, y);
    }

    private void stopAllRobots() {
        for (int i = 0; i < robotNames.size(); i++) {
            String name = robotNames.get(i);
            Robot robot = robotController.getRobot(name);
            int id = robot.getId();
            application.pauseRobot(id);
        }
    }

    private void resumeAllRobots() {
        for (int i = 0; i < robotNames.size(); i++) {
            String name = robotNames.get(i);
            Robot robot = robotController.getRobot(name);
            int id = robot.getId();
            application.unPauseRobot(id);
        }
    }
    
    RobotController getRobotController() {
        return robotController;
    }
    
    Application getApplication() {
        return application;
    }

    @Override
    public void run() {
        setName("Navigation Controller");
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
                stopAllRobots();
                break;
            }
            if (paused) {
                continue;
            }

            for (int i = 0; i < robotNames.size(); i++) {
                String name = robotNames.get(i);
                Robot applicationRobot = robotController.getRobot(name);
                if (name.equals("SLAM")){continue;}
                if (applicationRobot.isInManualMode()) {continue;}
                int id = applicationRobot.getId();
                /*
                if (applicationRobot.isHome() && !applicationRobot.isGoingHome()) {
                    
                    // For optimal S_REF send robot 35 cm forward.. deactiavted for mutual testing
                    //application.writeCommandToRobot(id, name, 0, 35);
                    
                    if (debug) {
                        System.out.println("Send to correct point!");
                    }
                    System.out.println(name);
                    while (applicationRobot.isDockReady()) {
                        // When robot arrives at point 0,35 it starts the S_ref scan
                        if (!applicationRobot.isBusy() && !applicationRobot.isRangeScanBase()) {
                            System.out.println("SCAN S_REF!");
                            applicationRobot.setRangeScanBase(true);
                            if (debug) {
                                System.out.println("SCAN S_REF!");
                            }
                        }
                    }
                   
                } else if (applicationRobot.isAtBase() && applicationRobot.isGoingHome() && applicationRobot.isDockReady()) {
                    //application.writeCommandToRobot(id, name, -(applicationRobot.getRobotOrientation() - 90), 0);
                    applicationRobot.setRangeScanBase(true);
                    if (debug) {
                        System.out.println("RETURNED TO BASE. DOCKING -> TRUE!!!");
                    }
                    applicationRobot.setHome(true);

                    while (applicationRobot.isDockReady()) {
                        if (!applicationRobot.isBusy() && !applicationRobot.isRangeScanBase() && applicationRobot.isHome()) {
                            if (debug) {
                                System.out.println("SCAN S_NEW!!!!! range scan -> TRUE");
                            }
                            applicationRobot.setRangeScanBase(true);
                        }
                    }
                }
                */
                
                
                
                
                if (navigationRobots.get(name).hasNewPriorityCommand()) {
                    int[] nextCommand = navigationRobots.get(name).getPriorityCommand();
                    if (debug) {
                        System.out.println("Priority command: " + nextCommand[0] + "," + nextCommand[1]);
                    }
                    application.writeCommandToRobot(id, name, nextCommand[0], nextCommand[1]);
                } else if (!applicationRobot.isBusy() && !navigationRobots.get(name).isInCollisionManagement()) {
                    if (navigationRobots.get(name).hasMoreWaypoints()) {
                        // Get next target
                        Position nextWaypoint = navigationRobots.get(name).getNextWaypoint();
                        // Get current obot location and orientation
                        /*
                        Position nextWaypoint2 = correct(nextWaypoint, applicationRobot);
                         */
                        // Get current obot location and orientation
                        //Position currentPosition = new Position(applicationRobot.getPosition());
                        //int currentOrientation = applicationRobot.getRobotOrientation();
                        // Command the robot to move to the next waypoint along its path
                        //int[] newCommand = findCommandToTargetPoint(nextWaypoint, currentPosition, currentOrientation);
                        int[] newCommand = {(int) nextWaypoint.getXValue(), (int) nextWaypoint.getYValue()};
                        if (debug) {
                            //System.out.println(name + ": Executing next command, ROTATION " + newCommand[0] + ", DISTANCE " + newCommand[1]);
                            System.out.println("Sending waypoint: " + newCommand[0] + "," + newCommand[1]);
                        }
                        application.writeCommandToRobot(id, name, newCommand[0], newCommand[1]);
                    } else if (!robotTaskManager.isWorkingOnTask(name)) {
                        if (debug) {
                            System.out.println(name + ": Idle. Searching for best target");
                        }
                        robotTaskManager.createNewTask(robotController.getRobot(name), navigationRobots.get(name), name);
                    }
                }
                // DOCKING -> disabled
                /*
                while (applicationRobot.getAdjustRobot() > -1) {
                    switch (applicationRobot.getAdjustRobot()) {
                        case 0:
                            // Idle case
                            break;
                        case 1:
                            // After the TransformationAlg is preformed send the robot to correct position
                            Position nextWaypoint = new Position((int) applicationRobot.getRealPose(0, 0), (int) applicationRobot.getRealPose(1, 0));
                            Position currentPosition = new Position(applicationRobot.getPosition());
                            if (debug) {
                                System.out.println("TARGET: " + nextWaypoint.getXValue() + ", " + nextWaypoint.getYValue());
                                System.out.println("Distance to target: " + Position.distanceBetween(currentPosition, nextWaypoint));
                            }
                            System.out.println("TARGET: " + nextWaypoint.getXValue() + ", " + nextWaypoint.getYValue());
                            System.out.println("Distance to target: " + Position.distanceBetween(currentPosition, nextWaypoint));
                            int[] newCommand = findCommandToTargetPoint(nextWaypoint, currentPosition, applicationRobot.getRobotOrientation());
                            if (Position.distanceBetween(currentPosition, nextWaypoint) > 1) {
                                application.writeCommandToRobot(id, name, newCommand[0], newCommand[1] * 2);
                                applicationRobot.setAdjustRobot(2);
                            } else {
                                application.writeCommandToRobot(id, name, 90, 0);
                                applicationRobot.setAdjustRobot(2);
                            }
                            break;
                        case 2:
                            // Turn robot to what it thinks is 90 degrees
                            if (!applicationRobot.isBusy()) {
                                application.writeCommandToRobot(id, name, -(applicationRobot.getRobotOrientation() + applicationRobot.getAdjustDirection() - 90), 0);
                                applicationRobot.setAdjustRobot(3);
                            }
                            break;
                        case 3:
                            // Adjust for error in orientation
                            if (!applicationRobot.isBusy()) {
                                //application.writeCommandToRobot(id, name, applicationRobot.getAdjustDirection(), 0);
                                applicationRobot.setAdjustRobot(4);
                                applicationRobot.setAdjustDirection(0);
                                applicationRobot.setPosition(applicationRobot.getBasePosition());
                                robotController.getRobot(name).setPosition(applicationRobot.getBasePosition());
                            }
                            break;
                        case 4:
                            // Find wall inside charging station
                            if (!applicationRobot.isBusy()) {
                                if (!applicationRobot.isRobotAligned()) {
                                    if (debug) {
                                        System.out.println("Waiting to scan back wall..");
                                    }
                                    applicationRobot.setAdjustRobot(-3);
                                } else {
                                    application.writeCommandToRobot(id, name, 0, -applicationRobot.getBackUpDist());
                                    applicationRobot.setAdjustRobot(5);
                                }
                            }
                            break;
                        case 5:

                            // Pause robot in charger
                            if (!applicationRobot.isBusy()) {
                                if (debug) {
                                    System.out.println("pausing robot");
                                }
                                applicationRobot.setAdjustRobot(-2);
                                applicationRobot.setRobotAligned(false);
                                application.pauseRobot(id);
                                break;
                            }
                            break;
                        case 6:
                            application.unPauseRobot(id);
                            applicationRobot.setAdjustRobot(-1);
                            break;
                        case 7:
                            application.writeCommandToRobot(id, name, 0, -15);
                            applicationRobot.setAdjustRobot(-3);
                            break;
                        default:
                            break;
                    }
                }
                // Resume mapping
                if (!applicationRobot.isGoingHome()) {
                    applicationRobot.setAtBase(false);
                    applicationRobot.setConfirmPose(false);
                }
                */
            }
        }
    }

    public Position correct(Position p, Robot r) {
        double[][] vect = new double[3][1];
        vect[0][0] = p.getXValue();
        System.out.println("nextWaypointO.getXValue() " + p.getXValue());
        System.out.println("nextWaypointO.getYValue " + p.getYValue());
        vect[1][0] = p.getYValue();
        vect[2][0] = 1;
        SimpleMatrix nextWaypointVect = new SimpleMatrix(vect);
        SimpleMatrix t = r.getTransMatrix().mult(nextWaypointVect);
        System.out.println("T");
        t.print();
        Position n = new Position(t.get(0, 0), t.get(1, 0));
        return n;
    }

    static int[] findCommandToTargetPoint(Position target, Position currentPosition, int robotHeading) {
        int distance = (int) Position.distanceBetween(currentPosition, target);
        int rotation = ((int) Position.angleBetween(currentPosition, target).getValue() - robotHeading + 360) % 360;
        if (rotation > 180) {
            rotation -= 360;
        }
        int[] command = {rotation, distance};
        return command;
    }
}
