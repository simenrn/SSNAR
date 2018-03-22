/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.communication;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.NoSuchPortException;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * This class represents the serial communication between the server and the
 * nRF51-dongle connected to it. It holds the communication parameters and
 * provides functionality for writing messages over serial to the dongle.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class SerialCommunication extends Thread {

    private CommPortIdentifier portIdentifier;
    private CommPort commPort;
    private CommPort shdwCommPort;
    private SerialPort serialPort;
    private InputStream inStream;
    private OutputStream outStream;
    private StringBuilder sb = new StringBuilder();
    private String shdwPort;
    private boolean debug = false;
    private final ConcurrentLinkedQueue<Frame> inbox;

    /**
     * Constructor of the class Communication
     *
     * @param inbox The systems message inbox
     */
    public SerialCommunication(ConcurrentLinkedQueue<Frame> inbox) {
        this.inbox = inbox;
    }

    /**
     * Method for connecting to a given port
     *
     * @param portName the port to connect to
     * @throws gnu.io.UnsupportedCommOperationException .
     * @throws gnu.io.PortInUseException .
     * @throws java.io.IOException .
     * @throws gnu.io.NoSuchPortException .
     */
    public boolean connect(String portName) throws UnsupportedCommOperationException,
            PortInUseException, IOException, NoSuchPortException {
        disconnectCommPort(shdwCommPort);
        this.portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
        if (portIdentifier.isCurrentlyOwned()) {
            System.out.println("Error: Port is currently in use");
        } else {
            this.commPort = portIdentifier.open(this.getClass().getName(), 2000);
            shdwCommPort = this.commPort;

            if (commPort instanceof SerialPort) {
                this.serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(38400, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);

                serialPort.setDTR(true);
                serialPort.setRTS(true);

                this.inStream = serialPort.getInputStream();
                this.outStream = serialPort.getOutputStream();
                inStream.skip(inStream.available());
                return true;
            } else {
                System.out.println("Error: Only serial ports are handled by this example.");
            }
        }
        return false;
    }

    /**
     * Method that sends a message to the connected com-port
     *
     * @param data The bytes to be sent
     */
    public synchronized void send(byte[] data) {
        try {
            outStream.write(data);
            outStream.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        super.run();
        ByteArrayOutputStream frame = new ByteArrayOutputStream();
        while (true) {
            try {
                int c;
                while ((c = inStream.read()) != -1) {
                    frame.write(c);
                    if (c == 0) { //End of COBS-encoded packet
                        inbox.add(new Frame(CobsUtils.decode(frame.toByteArray())));
                        frame.reset();
                    }
                }
            } catch (IOException ex) {
                Logger.getLogger(SerialCommunication.class.getName()).log(Level.SEVERE, null, ex);
                frame.reset();
            } catch (Frame.FrameCorruptException ex) {
                System.out.println("Frame corrupt");
                frame.reset();
            }
        }
    }

    /**
     * Disconnects the previous used com port
     *
     * @param shdwCommPort the port to disconnect
     */
    private void disconnectCommPort(CommPort shdwCommPort) {

        if (shdwCommPort != null) {
            new Thread() {
                @Override
                public void run() {
                    shdwCommPort.close();
                    if (debug) {
                        System.out.println("ComPort " + shdwCommPort.getName() + " closed.");
                    }
                }

            }.start();
        }
    }

    /**
     * Method that closes the current com-port
     */
    public void closeComPort() {
        disconnectCommPort(shdwCommPort);
    }
}
