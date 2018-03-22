/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.communication;

import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.UnsupportedCommOperationException;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import no.ntnu.tem.application.RobotController;
import java.util.concurrent.ConcurrentLinkedQueue;
import no.ntnu.tem.application.Application;
import no.ntnu.tem.robot.Robot;
/**
 * This class represents the initializer in the communication package. It holds
 * the different instantiated objects, and it provides methods for sending
 * communication commands.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class Communication implements ARQProtocol.DisconnectedListener {

    public static final HashMap<Integer, Protocol> protocolSettings = new HashMap<>();
    public static final int LOCAL_ADDRESS = 0;
    private final ConcurrentLinkedQueue<Message> messageInbox;
    private final ConcurrentLinkedQueue<Frame> networkInbox;
    private final InboxReader inR;
    private final SerialCommunication serialcom;
    private final ListPorts lp;
    private final Network network;
    private String comPort = null;

    private final ARQProtocol arqProtocol;
    private final SimpleProtocol simpleProtocol;
    private final Application app;
    /**
     * Constructor of the class Communication
     *
     * @param rc the RobotController object
     */
    public Communication(Application app, RobotController rc) {
        this.app = app;
        this.networkInbox = new ConcurrentLinkedQueue<>();
        this.messageInbox = new ConcurrentLinkedQueue<>();
        
        
        this.inR = new InboxReader(messageInbox, rc);
        this.serialcom = new SerialCommunication(networkInbox);
        this.network = new Network(LOCAL_ADDRESS, serialcom, networkInbox);
        this.lp = new ListPorts();
        this.arqProtocol = new ARQProtocol(network, messageInbox);
        this.simpleProtocol = new SimpleProtocol(network, messageInbox);
        
        // The following can be used to select what protocol is used to send the different message types
        protocolSettings.put(Message.CONFIRM, arqProtocol);
        protocolSettings.put(Message.FINISH, arqProtocol);
        protocolSettings.put(Message.ORDER, arqProtocol);
        //protocolSettings.put(Message.PRIORITY_ORDER, arqProtocol);
        protocolSettings.put(Message.PAUSE, arqProtocol);
        protocolSettings.put(Message.UNPAUSE, arqProtocol);
        
        ARQProtocol.addDisconnectListener(this);
        
        network.setReceiver(Network.PROTOCOL_ARQ, arqProtocol);
        network.setReceiver(Network.PROTOCOL_SIMPLE, simpleProtocol);
        
        network.start();
    }

    /**
     * Method for starting the communication module
     *
     * @param port The port to connect to, e.g. COM1, COM2, COM3 on Windows
     * @throws gnu.io.UnsupportedCommOperationException .
     * @throws gnu.io.PortInUseException if the port is busy
     * @throws java.io.IOException .
     * @throws gnu.io.NoSuchPortException if the port doesn't exist
     */
    public boolean startCommunication(String port)
            throws UnsupportedCommOperationException, PortInUseException,
            IOException, NoSuchPortException {
        boolean success = serialcom.connect(port);
        if (serialcom.isAlive()) {
        } else {
            serialcom.start();
        }

        return success;
    }

    /**
     * Method that returns a list containing available com ports
     *
     * @return the list
     */
    public LinkedList<String> listPorts() {
        return lp.listPorts();
    }

    /**
     * Method that initiates the inbox reader
     */
    public void startInboxReader() {
        if (!inR.isAlive()) {
            inR.start();
        }
    }

    /**
     * Method that returns the name of the current com in use
     *
     * @return the name of the current com-port
     */
    public String getComPort() {
        return this.comPort;
    }

    /**
     * Method that sets the name of the current com port to use
     *
     * @param comPort name of the comport
     */
    public void setComPort(String comPort) {
        this.comPort = comPort;
    }

    /**
     * Method that returns the inbox object
     *
     * @return the inbox
     */
    public ConcurrentLinkedQueue<Message> getInbox() {
        return this.messageInbox;
    }

    /**
     * Method that returns the serial communication object
     *
     * @return the serial communication
     */
    public SerialCommunication getSerialCommunication() {
        return serialcom;
    }
    public void disconnectRobot(int address) {
        confirmRobotFinished(address);
        arqProtocol.closeConnection(address);
    }
    /**
     * Method that wraps a message and sends it to a robot
     *
     * @param address The robots address
     * @param x the x-coordinate
     * @param y the y-coordinate
     */
    public void sendOrderToRobot(int address, int x, int y) {
        ByteBuffer buffer = ByteBuffer.allocate(5);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        buffer.put( (byte) Message.ORDER);
        buffer.putShort((short)x);
        buffer.putShort((short)y);
        byte data[] = new byte[5];
        buffer.rewind();
        buffer.get(data);
        String s = StandardCharsets.UTF_8.decode(buffer).toString();
        //System.out.println("x short = " + ((short)x) + ", y short = " + ((short)y));
        //System.out.println("sendOrderToRobot() entered, data[]: " + data[0] + "," + data[1]);
        //System.out.println("sendOrderToRobot buffer = " + s);
        //System.out.println("sendOrderToRobot data = " + Arrays.toString(data));
        protocolSettings.get(Message.ORDER).send(address, data);
        
    }
    
    /**
     * Method that wraps a message and sends it to a robot
     * (changed to (x,y))
     *
     * @param address The robots address
     * @param orientation the robots wanted orientation
     * @param distance the distance to move
     */
    /*
    public void sendPriorityOrderToRobot(int address, int orientation, int distance) {
        ByteBuffer buffer = ByteBuffer.allocate(5);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        buffer.put( (byte) Message.ORDER);
        buffer.putShort((short)orientation);
        buffer.putShort((short)distance);
        byte data[] = new byte[5];
        buffer.rewind();
        buffer.get(data);
        
        //System.out.println("sendOrderToRobot() entered, data[]: " + data[0] + "," + data[1]);
        
        protocolSettings.get(Message.PRIORITY_ORDER).send(address, data);
    }
    */

    /**
     * Method that confirms that the server has received the handshake
     *
     * @param address The robots address
     */
    public void confirmHandshake(int address) {
        protocolSettings.get(Message.ORDER).send(address, new byte[]{Message.CONFIRM});
    }

    /**
     * Method that pauses the robot
     *
     * @param address The robots address
     */
    public void pauseRobot(int address) {
        protocolSettings.get(Message.ORDER).send(address, new byte[]{Message.PAUSE});
        System.out.println("send pause to robot");
    }

    /**
     * Method that unpauses the robot
     *
     * @param address The robots address
     */
    public void unPauseRobot(int address) {
        protocolSettings.get(Message.ORDER).send(address, new byte[]{Message.UNPAUSE});
    }

    /**
     * Method that confirms that the robot is finished
     *
     * @param address The robots address
     */
    public void confirmRobotFinished(int address) {
       protocolSettings.get(Message.ORDER).send(address, new byte[]{Message.FINISH});
    }

    @Override
    public void connectionLost(int address) {
        for( Robot r : app.getConnectedRobotList() ) {
            if(r.getAddress() == address) {
                app.disconnectRobot(r.getName());
                break;
            }
        }
    }
}
