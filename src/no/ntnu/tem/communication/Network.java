/*
 * This code is written as a part of a Master Thesis
 * the spring of 2017.
 *
 * Kristian Lien (Master 2017 @ NTNU)
 */
package no.ntnu.tem.communication;

import java.util.Arrays;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 *
 * @author Kristian
 */
public class Network extends Thread {
    public static final int PROTOCOL_SIMPLE = 0;
    public static final int PROTOCOL_ARQ = 1;
    public static final int MAX_FRAME_SIZE = 50;
    public static final int MAX_PAYLOAD_LENGTH = MAX_FRAME_SIZE - 6;
    
    // Frame: [ RECEIVER | SENDER | PROTOCOL |        DATA           | CRC | 0x00 ]
    
    private final ConcurrentLinkedQueue<Frame> frameInbox;
    private final SerialCommunication serialcom;
    private final int localAddress;
    private final Protocol[] receivers;
    
    public Network(int localAddress, SerialCommunication serialcom, ConcurrentLinkedQueue<Frame> frameInbox) {
        this.localAddress = localAddress;
        this.frameInbox = frameInbox;
        this.serialcom = serialcom;
        receivers = new Protocol[2];
    }
    public void setReceiver(int protocol, Protocol receiver) {
        receivers[protocol] = receiver;
    }
    public void send(int receiver, int protocol, byte[] data) {
        //System.out.println("send function in Network.java = " + Arrays.toString(data));
        if(data.length > MAX_PAYLOAD_LENGTH) return;
        Frame frame = new Frame(receiver, localAddress, protocol, data);
        serialcom.send( CobsUtils.encode( frame.getBytes() ) );
    }
    
    @Override
    public void run() {
        super.run();
        
        while (true) {
            if(!frameInbox.isEmpty()) {
                Frame frame = frameInbox.poll();
                if(frame.getReceiver() == localAddress && receivers[ frame.getProtocol() ] != null) receivers[ frame.getProtocol() ].receive(frame.getSender(), frame.getData());
            } 
        }
    }
}
