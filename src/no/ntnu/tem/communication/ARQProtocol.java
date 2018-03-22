/*
 * This code is written as a part of a Master Thesis
 * the spring of 2017.
 *
 * Kristian Lien (Master 2017 @ NTNU)
 */
package no.ntnu.tem.communication;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 *
 * @author Kristian Lien
 */
public class ARQProtocol implements Protocol {
    private static final int WINDOW_SIZE = 4;
    private static final int RETRANSMISSION_TIMEOUT = 200;
    private static final int LOST_CONNECTION_TIMEOUT = 1000;
    private static final int MAX_DATA = Network.MAX_FRAME_SIZE - 2; // The available space in a frame minus the ARQ header bytes (sequence number and type)
    private final ConcurrentLinkedQueue<Integer> newConnections = new ConcurrentLinkedQueue<>();
    private final HashMap<Integer, ARQConnection> connections = new HashMap<>();
    private final ConcurrentLinkedQueue<Message> inbox;
    public boolean listening = false;
    private final Network network;
    private static final List<DisconnectedListener> disconnectedListeners = new ArrayList<>();

    public ARQProtocol(Network network, ConcurrentLinkedQueue<Message> inbox) {
        this.network = network;
        this.inbox = inbox;
    }

    /**
     * Method for connecting to a remote address. The remote address must be
     * listening for incoming connections for a connection to be made.
     *
     * @param remoteAddress The address of the device
     * @return A handle to the connection that was made
     */

    public ARQConnection newConnection(int remoteAddress) {
        if (connections.containsKey(remoteAddress)) {
            return null;
        }
        ARQConnection con = new ARQConnection(remoteAddress);
        connections.put(remoteAddress, con);
        if (con.connect()) {
            return con;
        } else {
            connections.remove(remoteAddress);
            return null;
        }
    }

    /**
     * Method for closing a connection.
     *
     * @param remoteAddress The address of the currently connected device to be
     * disconnected
     */
    public void closeConnection(int remoteAddress) {
        ARQConnection con = connections.get(remoteAddress);
        if (con != null) {
            con.close();
        }
    }

    public ARQConnection getConnection(int address) {
        return connections.get(address);
    }
    
    @Override
    public void send(int address, byte[] data) {
        ARQConnection con = connections.get(address);
        if(con != null) con.send(data);
    }
    /**
     * Method for handling incoming bytes sent with the ARQ protocol. If a
     * connection exists with the sender of the data, the bytes are handed to
     * the corresponding connection manager. If no connection exists for the
     * sender, and the received segment was a synchronize message, a new
     * connection with the remote device is set up.
     *
     * @param sender The address of the device that sent the data
     * @param data The received bytes
     */
    @Override
    public synchronized void receive(int sender, byte[] data) {
        ARQSegment segment = new ARQSegment();
        segment.wrap(data);
        if (connections.containsKey(sender)) {
            ARQConnection con = connections.get(sender);
            con.receive(segment);
        } else if (segment.isType(ARQSegment.TYPE_SYN)) {
            ARQConnection con = new ARQConnection(sender);
            connections.put(sender, con);
            if(listening) newConnections.add(sender);
            con.status = ARQConnection.STATUS_CONNECTED;
            con.start();
            segment = new ARQSegment();
            segment.makeSynAck();
            network.send(sender, Network.PROTOCOL_ARQ, segment.getBytes());
        }
    }

    /**
     * Method for listening for a new connection from any remote device. A call
     * to this function will block until an incoming connection is detected.
     *
     * @return The address of the device with whom a connection is made
     */

    public int listen() {
        listening = true;
        int address;
        while (newConnections.peek() == null) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
                //Logger.getLogger(InboxReader.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
        address = newConnections.poll();
        listening = false;
        return address;
    }

    public static void addDisconnectListener(DisconnectedListener l) {
        disconnectedListeners.add(l);
    }

    /**
     * This class represents connection to a device
     *
     */
    public class ARQConnection extends Thread {
        public static final int STATUS_CLOSED = 0;
        public static final int STATUS_CONNECTED = 1;
        public static final int STATUS_CONNECTING = 2;
        public static final int STATUS_CLOSING = 3;

        ConcurrentLinkedQueue<ARQSegment> transmitWindow;
        ConcurrentLinkedQueue<ByteBuffer> sendBuffer;

        private int sequenceNumber; //Number of next packet to be sent
        private int requestNumber; // Next packet expected received
        private int sequenceBase; // Next packet receiver is expecting
        private int timer;
        private int timeout;
        private int status;
        private boolean timerStarted;
        private final int remoteAddress;
        private long lastCommunication;
        private ByteBuffer incompleteMessage;

        public ARQConnection(int remoteAddress) {
            this.transmitWindow = new ConcurrentLinkedQueue<>();
            this.sendBuffer = new ConcurrentLinkedQueue<>();
            this.remoteAddress = remoteAddress;
            sequenceNumber = requestNumber = sequenceBase = timer = timeout = status = 0;
            timerStarted = false;
            lastCommunication = System.currentTimeMillis();
        }

        /**
         * Method for closing an active connection. When this function is called
         * the status is changed. The connection's thread continuously checks
         * the status variable, and when it detects a change to 'closed' the
         * thread will destroy itself.
         */
        private void close() {
            if(status != STATUS_CLOSED) status = STATUS_CLOSING;
        }

        /**
         * Method for connecting to the address given to the constructor. A
         * synchronize segment is sent to the address, and the method blocks for
         * a maximum of 1 second while waiting for a response.
         *
         * @return True if the connection was successful
         */
        public boolean connect() {
            status = STATUS_CONNECTING;
            ARQSegment segment = new ARQSegment();
            segment.makeSyn();
            network.send(remoteAddress, Network.PROTOCOL_ARQ, segment.getBytes());
            start();
            while (timeout < 1000) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException ex) {
                    //Logger.getLogger(InboxReader.class.getName()).log(Level.SEVERE, null, ex);
                }
                if (status == STATUS_CONNECTED) {
                    return true;
                }
            }
            return false;
        }

        /**
         * Method for handling bytes received on this connection. If the
         * received segment is data, then an acknowledgment for the data is sent
         * if the sequence number was the one expected. If the sequence number
         * was not correct, an acknowledgment is sent with the sequence number
         * of the data we expect.
         *
         * If the segment was an acknowledgment of sent data, then that data is
         * removed from the transmit window.
         */
        private void receive(ARQSegment segment) {
            synchronized (this) {
                lastCommunication = System.currentTimeMillis();
                if (segment.isType(ARQSegment.TYPE_DATA)) {
                    if (segment.getSequenceNumber() == requestNumber) {
                        requestNumber++;
                        if (requestNumber == 128) {
                            requestNumber = 0;
                        }
                        reassemble(segment.getDataBytes());
                    }
                    sendAck(requestNumber);
                } else if (segment.isType(ARQSegment.TYPE_ACK)) {
                    int count = segment.getSequenceNumber() - sequenceBase;
                    int i;
                    if (count < 0) {
                        count += 128;
                    }
                    if (count != 0) {
                        for (i = 0; i < count; i++) {
                            transmitWindow.poll();
                        }
                        timer = timeout = 0;
                        sequenceBase = segment.getSequenceNumber();
                        if (sequenceBase == sequenceNumber) {
                            timerStarted = false;
                        }
                    }
                } else if (segment.isType(ARQSegment.TYPE_SYNACK)) {
                    sendAck(0);
                    status = STATUS_CONNECTED;
                } else if (segment.isType(ARQSegment.TYPE_SYN)) {
                    segment = new ARQSegment();
                    segment.makeSynAck();
                    network.send(remoteAddress, Network.PROTOCOL_ARQ, segment.getBytes());
                }
            }
        }

        /**
         * Method for sending bytes on this connection. The data is divided into
         * segments (or just one segment if the length does not exceed the max
         * limit) which are added to the send buffer. The segments are
         * transferred from the send buffer to the transmit window and sent when
         * possible.
         *
         * @param data The bytes to send
         */

        public void send(byte[] data) {
            if(status != STATUS_CONNECTED) return;
            int remaining = data.length;
            ByteBuffer segment = ByteBuffer.allocate(remaining + 2 > MAX_DATA ? MAX_DATA : remaining + 2);
            segment.order(ByteOrder.LITTLE_ENDIAN);
            segment.putShort((short) data.length); // 16 bytes
            int i;

            for (i = 0; i < data.length; i++) {
                --remaining;
                segment.put(data[i]);
                if (segment.position() == segment.capacity()) {
                    segment.rewind(); // Reset the position to the start, so the receiver starts reading for the correct location
                    sendBuffer.add(segment);
                    segment = ByteBuffer.allocate(remaining > MAX_DATA ? MAX_DATA : remaining); // Create a new buffer for the next segment
                    segment.order(ByteOrder.LITTLE_ENDIAN);
                }
            }
        }

        /**
         * Method for sending an acknowledgment on this connection.
         *
         * @param sequeneNumber The sequence number of the data segment to be
         * acknowledged.
         */
        private void sendAck(int sequenceNumber) {
            ARQSegment ackMsg = new ARQSegment();
            ackMsg.makeAck(sequenceNumber);
            network.send(remoteAddress, Network.PROTOCOL_ARQ, ackMsg.getBytes());
        }

        /**
         * This method reassembles bytes sent over the network to a complete
         * message. The first two bytes of a message contains the total length,
         * and when this amount is reached the message is added to the inbox.
         *
         * @param partMessage The received bytes.
         */
        private void reassemble(byte[] partMessage) {
            ByteBuffer part = ByteBuffer.wrap(partMessage);
            part.order(ByteOrder.LITTLE_ENDIAN);
            if (incompleteMessage == null) { // No incomplete message has been received on the connection 
                incompleteMessage = ByteBuffer.allocate(part.getShort());
            }

            incompleteMessage.put(part);

            if (!incompleteMessage.hasRemaining()) { // The buffer has been filled => the complete message has been received.
                byte[] fullMessage = new byte[incompleteMessage.capacity()];

                incompleteMessage.rewind();
                incompleteMessage.get(fullMessage);

                inbox.add(new Message(remoteAddress, fullMessage));
                incompleteMessage = null;
            }
        }

        /**
         * The method run by this connection's thread. When the connection is
         * started (using .start() ) the thread starts running.' The thread
         * checks if there is available space in the transmit window, and if so
         * sends a segment from the send buffer (if it is not empty). Each
         * iteration a timer is increased if the connection has sent data but
         * not yet received an acknowledgment. If the timer expires the segments
         * in the transmit window are resent.
         */
        @Override
        public void run() {
            super.run();
            while (true) {
                switch (status) {
                    case STATUS_CONNECTED:
                    case STATUS_CLOSING:
                        if (timerStarted) {
                            timer+=10;
                            timeout+=10;
                            
                            if(timeout > LOST_CONNECTION_TIMEOUT) {
                                //Close
                                System.out.println("Timeout");
                                status = STATUS_CLOSED;
                                disconnectedListeners.stream()
                                    .forEach(l -> l.connectionLost(remoteAddress));
                                break;
                            }
                            else if (timer > RETRANSMISSION_TIMEOUT) { 
                                ARQSegment s;
                                System.out.println("Resending");
                                Iterator<ARQSegment> itr = transmitWindow.iterator();
                                while (itr.hasNext()) {
                                    network.send(remoteAddress, Network.PROTOCOL_ARQ, itr.next().getBytes());
                                }
                                timer = 0;
                            }
                        }
                        if ((System.currentTimeMillis() - lastCommunication > 3000) && transmitWindow.isEmpty() && sendBuffer.isEmpty()) {
                            ARQSegment s = new ARQSegment();
                            s.makeAliveTest(sequenceNumber++);
                            //System.out.println("Alive test");
                            lastCommunication = System.currentTimeMillis();
                            if (sequenceNumber == 128) {
                                sequenceNumber = 0;
                            }
                            timerStarted = true;
                            transmitWindow.add(s);
                            network.send(remoteAddress, Network.PROTOCOL_ARQ, s.getBytes());
                            
                        }
                        if (transmitWindow.size() < WINDOW_SIZE && !sendBuffer.isEmpty()) {
                            ARQSegment s = new ARQSegment();
                            ByteBuffer buffer = sendBuffer.poll();
                            byte[] data = new byte[buffer.capacity()];

                            buffer.get(data);
                            s.makeData(sequenceNumber++, data);

                            if (sequenceNumber == 128) {
                                sequenceNumber = 0;
                            }
                            timerStarted = true;
                            transmitWindow.add(s);
                            network.send(remoteAddress, Network.PROTOCOL_ARQ, s.getBytes());
                        }
                        if(transmitWindow.isEmpty() && sendBuffer.isEmpty() && status == STATUS_CLOSING) status = STATUS_CLOSED;
                        break;
                    case STATUS_CONNECTING:
                        timeout++;
                        break;
                    case STATUS_CLOSED:
                        connections.remove(remoteAddress);
                        return;
                }
                        

                try {
                    Thread.sleep(10);
                } catch (InterruptedException ex) {
                    //Logger.getLogger(InboxReader.class.getName()).log(Level.SEVERE, null, ex);
                }
            }
        }
    }

    private static class ARQSegment {

        public static final byte TYPE_DATA = 0;
        public static final byte TYPE_ACK = 1;
        public static final byte TYPE_SYN = 2;
        public static final byte TYPE_SYNACK = 3;
        public static final byte TYPE_ALIVE_TEST = 4;

        private byte type;
        private int sequenceNumber;
        private byte[] bytes;

        public void wrap(byte[] contents) {
            this.bytes = contents;
            this.type = contents[0]; // The first byte signals the type
            if (type == TYPE_DATA || type == TYPE_ACK) {
                this.sequenceNumber = contents[1]; // The second contains the sequence number
            } else {
                sequenceNumber = 0xFF;
            }
        }

        public void makeAck(int sequenceNumber) {
            bytes = new byte[]{TYPE_ACK, (byte) sequenceNumber};
            this.type = TYPE_ACK;
            this.sequenceNumber = sequenceNumber;
        }

        public void makeData(int sequenceNumber, byte[] data) {
            bytes = new byte[data.length + 2];
            bytes[0] = TYPE_DATA;
            bytes[1] = (byte) sequenceNumber;
            System.arraycopy(data, 0, bytes, 2, data.length);
            this.type = TYPE_DATA;

            this.sequenceNumber = sequenceNumber;
        }

        public void makeSyn() {
            this.type = TYPE_SYN;
            bytes = new byte[1];
            bytes[0] = TYPE_SYN;
        }

        public void makeAliveTest(int sequenceNumber) {
            this.type = TYPE_ALIVE_TEST;
            bytes = new byte[2];
            bytes[0] = TYPE_ALIVE_TEST;
            bytes[1] = (byte) sequenceNumber;
        }

        public void makeSynAck() {
            this.type = TYPE_SYNACK;
            bytes = new byte[1];
            bytes[0] = TYPE_SYNACK;
        }

        public int getSequenceNumber() {
            return sequenceNumber;
        }

        public byte[] getBytes() {
            return bytes;
        }

        public byte[] getDataBytes() {
            if (type == TYPE_DATA) {
                return Arrays.copyOfRange(bytes, 2, bytes.length);
            } else {
                return null;
            }
        }

        public boolean isType(byte type) {
            return this.type == type;
        }
    }

    public interface DisconnectedListener {

        void connectionLost(int address);
    }
}
