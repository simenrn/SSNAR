/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.application;

import no.ntnu.tem.gui.MapGraphic;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.UnsupportedCommOperationException;
import java.awt.Desktop;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.LinkedList;
import java.util.logging.Level;
import java.util.logging.Logger;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import no.ntnu.et.map.GridMap;
import no.ntnu.et.navigation.NavigationController;
import no.ntnu.tem.communication.Communication;
import no.ntnu.tem.gui.MainGUI;
import no.ntnu.et.simulator.Simulator;
import no.ntnu.et.mapping.MappingController;
import no.ntnu.et.navigation.SlamNavigationController;
import no.ntnu.tem.robot.Robot;
import no.ntnu.hkm.particlefilter.Particlefilter;
import no.ntnu.hkm.particlefilter.MapMerger;

/**
 * This class is the main class of the program. It connects the packages in the
 * project, and instantiates the different modules.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public final class Application {

    private final String MAPLOCATION;
    private final Communication com;
    private final RobotController rc;
    private LinkedList<String> portList;
    private final MainGUI gui;
    private Simulator sim;
    private final NavigationController navigation;
    //private final SlamNavigationController slamNavigation;
    private final MappingController slam;
    private final MapGraphic worldMapGraphic;
    private final GridMap worldMap;
    private boolean simulatorActive = false;
    private boolean pause = false;
    private boolean activateParticlefilter = false;
    private double[] particleFilterOptions = {0,0,0,0,0};
    
    private boolean debug = false;
    
    /**
     * Constructor of the class Application
     */
    public Application() {
        Installer.generateSystemDependantLibraries();
        MainGUI.setLookAndFeel();
        this.MAPLOCATION = new File("maps\\big_map.txt").getAbsolutePath();
        this.rc = new RobotController();
        this.com = new Communication(this, rc);
        this.worldMap = new GridMap(2, 50, 50);
        //this.worldMap = new GridMap(1, 50, 50);
        this.worldMapGraphic = new MapGraphic(worldMap, rc);
        this.slam = new MappingController(rc, worldMap);
        this.navigation = new NavigationController(rc, this, worldMap);
        //this.slamNavigation = new SlamNavigationController(rc, this, worldMap);
        this.gui = new MainGUI(this);
        if (System.getProperty("os.name").startsWith("Windows")) {
            getPDFList();
        }
    }
    public void connectToRobot(Robot r) {
        if (!simulatorActive) {
            System.out.println("ID:" + r.getId());
            System.out.println("Address:" + r.getAddress());
            com.confirmHandshake(r.getAddress());
        } else {
            sim.unpauseRobot(r.getName());
        }
        r.setConnected(true);
        navigation.addRobot(r.getName(), r.getId());
        slam.addRobot(r.getName());
    }
    /**
     * Opens a PDF using the default PDF viewer, the PDF should be in a manuals
     * folder thats next to the program path.
     *
     * @param name The name of the PDF to open, without ".pdf"
     */
    
    public void openPDF(String name) {
        if (Desktop.isDesktopSupported()) {
            try {
                String filename = "manuals\\" + name + ".pdf";
                File myFile = new File(filename);
                Desktop.getDesktop().open(myFile);
            } catch (IOException ex) {
                // no application registered for PDFs
            }
        }
    }

    /**
     * Method that returns a list of Strings containing the name all PDF's
     * inside the manuals folder that is in the same folder as the program
     *
     * @return LinkedList of Strings containing the name of all manuals
     */
    public LinkedList<String> getPDFList() {
        LinkedList<String> pdfList = new LinkedList<>();
        try {

            Files.walk(Paths.get("manuals\\")).forEach(filePath -> {
                if (Files.isRegularFile(filePath)) {
                    pdfList.addFirst(("" + filePath.getFileName()));
                }
            });
        } catch (IOException ex) {
            Logger.getLogger(Application.class.getName()).log(Level.SEVERE, null, ex);
        }
        return pdfList;
    }

    /**
     * Method that returns the world map
     *
     * @return the world map
     */
    public MapGraphic getWorldMap() {
        return worldMapGraphic;
    }
    
    /**
     * Method that returns robot controller
     * @return robot controller
     */
    public RobotController getRobotController() {
        return rc;
    }
    //////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZERS ////////////////////////////
    //////////////////////////////////////////////////////////////////////
    /**
     * Method that starts the system for self-navigating autonomous robots
     */
    public void startSystem() {
        slam.start();
        navigation.start();
        //slamNavigation.start();
        if(activateParticlefilter){
            for(Robot robot : rc.getRobotList()){
                if(robot.getName().equals("Drone")){continue;} //Drone does not need PF
                if(robot.getName().equals("SLAM")){continue;}
                Particlefilter p = new Particlefilter(robot, worldMap,particleFilterOptions);
                robot.setParticleFilter(p);
            } 
            activateParticlefilter = false;
        }
        for(Robot robot : rc.getRobotList()){
            if(robot.getParticleFilter() != null){
                robot.getParticleFilter().start();
            }
        }
    }

    /**
     * Method that initiates the communication module, whether the simulator is
     * active or not
     *
     * @return boolean that tells if success or not
     */
    public boolean startCommunication() {
        boolean success = false;
        if (!simulatorActive) {
            try {
                success = com.startCommunication(com.getComPort());
                if(success) com.startInboxReader();
            } catch (UnsupportedCommOperationException | PortInUseException | IOException | NoSuchPortException e) {
                success = false;
            }
        }
        return success;
    }

    //////////////////////////////////////////////////////////////////////
    /////////////////////////// ROBOT HANDLING ///////////////////////////
    //////////////////////////////////////////////////////////////////////


    /**
     * Method that returns the connected robot list
     *
     * @return the list
     */
    public ObservableList<Robot> getConnectedRobotList() {
        return rc.getRobotList();
    }

    //////////////////////////////////////////////////////////////////////
    /////////////////////// SIMULATOR FUNCTIONALITY //////////////////////
    //////////////////////////////////////////////////////////////////////
    /**
     * Method that returns the simulator object
     *
     * @return the simulator
     */
    public Simulator getSim() {
        return sim;
    }

    /**
     * Method that sets the simulator object
     *
     * @param sim the simulator
     */
    public void setSim(Simulator sim) {
        this.sim = sim;
    }

    /**
     * Method that sets the simulator active or not
     *
     * @param active simulator mode
     */
    public void setSimulatorActive(boolean active) {
        if (active) {
            sim = new Simulator(com.getInbox());
            com.startInboxReader();
            sim.openGui();

        }
        simulatorActive = active;
    }

    /**
     * Returns the state of the simulator
     *
     * @return true if simulator mode
     */
    public boolean isSimulatorActive() {
        return simulatorActive;
    }

    //////////////////////////////////////////////////////////////////////
    /////////////////////// COMUNICATION COMMANDS ////////////////////////
    //////////////////////////////////////////////////////////////////////


    /**
     * Method that cancels the connection to the specified robot
     *
     * @param name the robots name
     */
    public void disconnectRobot(String name) {
        if (!simulatorActive) {
            com.disconnectRobot(rc.getRobot(name).getAddress());
        } else {
            sim.disconnectRobot(name);
        }
        rc.removeRobot(name);
        slam.removeRobot(name);
        navigation.removeRobot(name);
        //slamNavigation.removeRobot("SLAM");
    }

    /**
     * Method that sends a command to a physical or simulated robot
     * (changed to (x,y)
     *
     * @param robotID the robots id
     * @param robotName the robots name
     * @param x the x-coordinate
     * @param y the y-coordinate
     */
    public void writeCommandToRobot(int robotID, String robotName, int x, int y) {
        //  newTime[robotID] = System.currentTimeMillis();
        if (!simulatorActive) {
            if (debug) {
                System.out.println("debug: writeCommandToRobot() entered: " + x + "," + y);
            }
            com.sendOrderToRobot( rc.getRobot(robotID).getAddress(), x, y);
            System.out.println("writeCommandToRobot() entered: " + x + "," + y);
        } else {
            sim.setRobotCommand(robotName, x, y);
        }
        rc.getRobot(robotName).setBusy(true);
    }
    
    /**
     * Method that sends a command to a physical or simulated robot
     * (changed to (x,y)
     *
     * @param robotID the robots id
     * @param robotName the robots name
     * @param orientation the new orientation
     * @param distance the distance to go
     */
    /*
    public void writePriorityCommandToRobot(int robotID, String robotName, int orientation, int distance) {
        //  newTime[robotID] = System.currentTimeMillis();
        if (!simulatorActive) {
            if (debug) {
                System.out.println("writePriorityCommandToRobot() entered: " + orientation + "," + distance);
            }
            com.sendPriorityOrderToRobot( rc.getRobot(robotID).getAddress(), orientation, distance);
        } else {
            sim.setRobotCommand(robotName, orientation, distance);
        }
        rc.getRobot(robotName).setBusy(true);
    }
    */

    /**
     * Method that confirms that the robot is finished
     *
     * @param robotID the id of the robot.
     */
    public void confirmRobotFinished(int robotID) {
        if (!simulatorActive) {
            com.confirmRobotFinished(robotID);
        }
    }

    /**
     * Method that pauses the robot
     *
     * @param robotID the robots ID
     */
    public void pauseRobot(int robotID) {
        if (!simulatorActive) {
            com.pauseRobot(robotID);
        }
    }

    /**
     * Method that unpauses the robot
     *
     * @param robotID the robots ID
     */
    public void unPauseRobot(int robotID) {
        if (!simulatorActive) {
            com.unPauseRobot(robotID);
        }
    }

    //////////////////////////////////////////////////////
    /////////////// COMPORT FUNCTIONALITY ////////////////
    //////////////////////////////////////////////////////
    /**
     * Method that returns a list of available com ports
     *
     * @return the list
     */
    public LinkedList<String> getPortList() {
        if (!simulatorActive) {
            this.portList = com.listPorts();
            return portList;
        } else {
            return null;
        }
    }

    /**
     * Method that sets the current com port
     *
     * @param port the ports name
     */
    public void setComPort(String port) {
        com.setComPort(port);
    }

    //////////////////////////////////////////////////////
    /////////////////       MAIN       ///////////////////
    //////////////////////////////////////////////////////
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        new Application();
    }

    /**
     * Pauses the navigation and slam
     */
    public void stopSystem() {
        navigation.pause();
        //slamNavigation.pause();
        slam.pause();
        for(Robot robot : rc.getRobotList()){
            if(robot.getParticleFilter() != null){
                robot.getParticleFilter().pause();
            }
        }
    }


    /**
     * Method should be used before turning off the java program, this ensures
     * that the dongle is reset and disconnected before next use
     */
    public void turnOffProgram() {
        if (!simulatorActive) {
            try {
                System.out.println("Closing port");
                com.getSerialCommunication().closeComPort();
                System.out.println("Port closed");
            } catch (Exception e) {

            }
        }
    }
    
    public void useParticlefilter(boolean state, double particles, double convergenceRate, double windowSize,double movmentUncertainty,double rotationUncertainty){
        activateParticlefilter = state;
        particleFilterOptions[0] =  particles;
        particleFilterOptions[1] =  convergenceRate;
        particleFilterOptions[2] =  windowSize;
        particleFilterOptions[3] =  movmentUncertainty;
        particleFilterOptions[4] =  rotationUncertainty;
    }

    /**
     * Makes lines from point clouds in the map
     */
    public void cleanMap(){
        worldMap.cleanMap();
    }    
}
