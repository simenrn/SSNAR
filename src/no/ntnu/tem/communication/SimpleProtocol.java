/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Kristian Lien (Master 2017 @ NTNU)
 */
package no.ntnu.tem.communication;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.HashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 *
 * @author Kristian
 */
public class SimpleProtocol implements Protocol {
    private final ConcurrentLinkedQueue<Message> inbox;
    private final Network network;
    private final HashMap<Integer, SimpleMessage> messages = new HashMap<>();
    public SimpleProtocol(Network network, ConcurrentLinkedQueue<Message> inbox) {
        this.inbox = inbox;
        this.network = network;
    }
    
    @Override
    public void receive(int address, byte[] data) {
        reassemble(address, data);
    }

    public void send(int address, byte[] data) {
        int remaining = data.length;
        ByteBuffer segment ;
        byte current_number = 0;
        byte total_number = (byte) (data.length/(Network.MAX_PAYLOAD_LENGTH-2) + data.length % (Network.MAX_PAYLOAD_LENGTH-2) == 0 ? 0 : 1);
        int i;
        byte[] msg; 
        int num;
        int offset = 0;
        for (i = 0; i < total_number; i++) {
            num = remaining < (Network.MAX_PAYLOAD_LENGTH-2) ? remaining : (Network.MAX_PAYLOAD_LENGTH-2);
            segment = ByteBuffer.allocate(num+2);
            segment.put(current_number++);
            segment.put( (byte) (total_number - 1) );
            segment.put(Arrays.copyOfRange(data, offset, offset+num));
            offset+=num;
            remaining-= num;
            segment.rewind(); 
            msg = new byte[segment.capacity()];
            segment.get(msg);
            System.out.println("HEISANN");
            System.out.println(Arrays.toString(msg));
            network.send(address, Network.PROTOCOL_SIMPLE, msg);
        }
    }
    /**
     * This method reassembles bytes sent over the network to a complete message. The first 
     * two bytes of a message contains the total length, and when this amount is reached 
     * the message is added to the inbox.
     * 
     * @param partMessage The received bytes.
     * @param address The address of the device who sent the bytes
     */
    public void reassemble(int address, byte[] partMessage) {
        SimpleMessage incompleteMessage = messages.remove(address);
        if(incompleteMessage == null) incompleteMessage = new SimpleMessage();
        if (partMessage[0] == 0) { // Sequence number 0 => start of a message
            incompleteMessage.data = Arrays.copyOfRange(partMessage, 2, partMessage.length);
            incompleteMessage.nextPart = 1;
        } else if (partMessage[0] == incompleteMessage.nextPart) {
            byte[] tmp = incompleteMessage.data;
            incompleteMessage.data = new byte[incompleteMessage.data.length + partMessage.length - 2];
            System.arraycopy(tmp, 0, incompleteMessage.data, 0, tmp.length);
            System.arraycopy(partMessage, 2, incompleteMessage.data, tmp.length, partMessage.length - 2);
            incompleteMessage.nextPart++;
        } else {
            return;
        }
        if(partMessage[0] == partMessage[1]) {// Sequence number of this message is the total message length => this is the last part
            inbox.add(new Message(address, incompleteMessage.data));
        } else {
            messages.put(address, incompleteMessage);
        }
    }
    class SimpleMessage {
       int nextPart = -1;
       byte[] data;
    }
}
