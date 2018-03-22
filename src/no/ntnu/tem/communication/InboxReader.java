/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.communication;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.ConcurrentLinkedQueue;
import no.ntnu.tem.application.RobotController;
import no.ntnu.tem.robot.Robot;

/**
 * This class retrieves the messages from the inbox. Furthermore it pushes the information to the
 * RobotController and thus to the rest of the application.
 *
 * @author Thor Eivind and Mats (Master 2016 at NTNU)
 * @author Kristian Lien (Master 2017 at NTNU)
 */
public class InboxReader extends Thread {

    private final RobotController rc;
    private final ConcurrentLinkedQueue<Message> inbox;
    private final boolean debug = false;

    /**
     * Constructor of the class InboxReader
     *
     * @param inbox the systems Inbox
     * @param rc the systems RobotController
     */
    public InboxReader(ConcurrentLinkedQueue<Message> inbox, RobotController rc) {
        this.setName("InboxReader");
        this.rc = rc;
        this.inbox = inbox;
    }

    /**
     * Method that retrieves a message from the inbox and initiates the
     * interpreting
     */
    @Override
    public void run() {
        super.run();
        int counter = 0;
        while (true) {
            if (!inbox.isEmpty()) {
                Message message = inbox.poll();
                int address = message.getSender();
                try {
                    switch (message.getType()) {
                        case Message.HANDSHAKE:
                            HandshakeMessage handshake = new HandshakeMessage(message.getData());
                            String name = handshake.getName();
                            int width = handshake.getWidth();
                            int length = handshake.getLength();
                            int axleOffset = handshake.getAxelOffset();
                            int messageDeadline = handshake.getDeadline();
                            int[] towerOffset = handshake.getTowerOffsets();
                            int[] sensorOffset = handshake.getSensorOffsets();
                            int[] irHeading = handshake.getSensorHeadings();
                            doHandshake(address, name, width, length, axleOffset, messageDeadline, towerOffset, sensorOffset, irHeading);
                            System.out.println("Message type: \tHANDSHAKE");
                           // System.out.println("IR Heading: \t" + irHeading);
                            break;
                        case Message.UPDATE:
                            UpdateMessage update = new UpdateMessage(message.getData());
                            int orientation = update.getHeading();
                            int[] position = update.getPosition();
                            int towerAngle = update.getTowerAngle();
                            int[] irData = update.getSensorValues();
                            doUpdate(address, orientation, position, towerAngle, irData);
                            //System.out.println("Message type: \t\tUPDATE");
                            //System.out.println("Orientation: \t\t" + orientation);
                            //System.out.println("Position: \t\t" + Arrays.toString(position));
                            //System.out.println("Tower Angle: \t\t" + towerAngle);
                            //System.out.println("IR sensor values: \t" + Arrays.toString(irData));
                            break;
                        case Message.DRONE_UPDATE:
                            DroneUpdateMessage droneUpdate = new DroneUpdateMessage(message.getData());
                            int droneOrientation = droneUpdate.getHeading();
                            int[] dronePosition = droneUpdate.getPosition();
                            int[] line = droneUpdate.getLine();
                            doDroneUpdate(address, droneOrientation, dronePosition, line);
                            System.out.println("Message type: \tDRONE UPDATE");
                            break;
                        case Message.IDLE:
                            doIdleUpdate(address);
                            System.out.println("Message type: \tIDLE");
                            break;

                        case Message.BATTERY_UPDATE:
                            BatteryMessage batteryUp = new BatteryMessage(message.getData());
                            int level = (int)batteryUp.getBatLevel();
                            doBatteryUpdate(address, level);
                            System.out.println("Message type: \tBATTERY UPDATE");
                            break;

                        case Message.DEBUG:
                            String msg = new String(message.getData());
                            System.out.println("Message type: \tDEBUG");
                            try (FileWriter fw = new FileWriter("LOG.txt", true)) {
                                fw.write(msg);
                                fw.write("\n");
                                fw.flush();
                            } catch (IOException e) {
                                System.err.println("IOexception: " + e.getMessage());
                            }
                            System.out.println(msg);
                            break;
                    }
                } catch (Message.MessageCorruptException ex) {
                    Robot r = rc.getRobot(address);
                    if (r != null) {
                        r.addMessageCorruptCount();
                    }
                    //  Logger.getLogger(InboxReader.class.getName()).log(Level.SEVERE, null, ex);
                } catch (Message.ValueCorruptException ex) {
                    Robot r = rc.getRobot(address);
                    if (r != null) {
                        r.addValueCorruptCount();
                    }
                    //   Logger.getLogger(InboxReader.class.getName()).log(Level.SEVERE, null, ex);
                }
            } else {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException ex) {
                    //Logger.getLogger(InboxReader.class.getName()).log(Level.SEVERE, null, ex);
                }
            }
        }

    }

    /**
     * Method for adding a new robot to the system after receiving a handshake
     *
     * @param name The robots name
     * @param width The robots physical width (cm)
     * @param length The robots physical length (cm)
     * @param axleOffset Axle offset lengthwise
     * @param messageDeadline The robots message deadline (min update rate[ms])
     * @param towerOffset Tower offset [a,b] a-lengthwise, b-crosswise
     * @param sensorOffset Sensor offset [a,b,c,d,..] from centre (radius)
     * @param irHeading IR heading relative to 0 deg (straight forward)
     */
    private void doHandshake(int address, String name, int width, int length,
            int axleOffset, int messageDeadline, int[] towerOffset, int[] sensorOffset,
            int[] irHeading) {
        rc.addRobot(address, name, width, length, messageDeadline, axleOffset,
                towerOffset, sensorOffset, irHeading);
    }

    /**
     * Method for updating one of the robots in the system
     *
     * @param name The robots name
     * @param orientation The robots orientation
     * @param position The robots position (x,y)
     * @param irHeading The IR-sensors heading
     * @param irData Data gathered from IR-sensors
     */
    private void doUpdate(int address, int orientation, int[] position,
            int irHeading, int[] irData) {
        rc.addMeasurment(address, orientation, position, irHeading, irData);
    }

    /**
     * Method for updating one of the robots in the system
     *
     * @param name The robots name
     * @param orientation The robots orientation
     * @param position The robots position (x,y)
     * @param irHeading The IR-sensors heading
     * @param irData Data gathered from IR-sensors
     */
    private void doDroneUpdate(int address, int orientation, int[] position, int[] line) {
        rc.addDroneMeasurment(address, orientation, position, line);
    }
    
        private void doBatteryUpdate(int address, int level) {
        rc.updateBattery(address, level);
    }

    /**
     * Method for updating the status to one of the robots in the system
     *
     * @param address
     * @param status
     */
    private void doIdleUpdate(int address) {
        rc.setIdle(address);
    }

    /*
    * Method for updating the battery of the robot
    *
    * @param name The robots name
    * @param the battery level
     */
    private void doBatteryUpdate(String name, int level) {
        rc.updateBattery(name, level);
    }

}
