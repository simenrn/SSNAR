/*
 * This code is written as a part of a Master Thesis
 * the spring of 2017.
 *
 * Kristian Lien (Master 2017 @ NTNU)
 */
package no.ntnu.tem.communication;

import java.nio.ByteBuffer;

/**
 *
 * @author Kristian Lien
 * A frame has the following format (number of bytes of each field in parentheses):
 * 
 * [ SENDER (1) | RECEIVER (1) | PROTOCOL (1) |        DATA (x)         | CRC (1) ]
 * 
 */
public class Frame {
   
    private final byte[] frame;
    private final int sender;
    private final int receiver;
    private final int crc;
    private final int protocol;
    private final byte[] data;
    
    public Frame(int receiver, int sender, int protocol, byte[] data) {
        this.sender = sender;
        this.receiver = receiver;
        this.protocol = protocol;
        this.data = data;
        frame = new byte[data.length+4];
        frame[0] = (byte)receiver;
        frame[1] = (byte)sender;
        frame[2] = (byte)protocol;
        System.arraycopy(data, 0, frame, 3, data.length);
        this.crc = CRC8.compute(frame, 0, frame.length-1, 0);
        frame[ frame.length-1 ] = (byte) crc;
    }
    public Frame(byte[] bytes) throws FrameCorruptException {
        if(bytes == null) throw new FrameCorruptException();
        frame = bytes;
        ByteBuffer buf = ByteBuffer.wrap(bytes);
        receiver = buf.get();
        sender = buf.get();
        protocol = buf.get();
        data = new byte[bytes.length - 4];
        buf.get(data);
        crc = buf.get();
        
        if(data.length <= 0) throw new FrameCorruptException();
        if( crc != (byte) CRC8.compute(bytes, 0, bytes.length-1, 0)) throw new FrameCorruptException();
    }
    
    public int getSender() {
        return sender;
    }
    public int getReceiver() {
        return receiver;
    }
    public int getProtocol() {
        return protocol;
    }
    public byte[] getData() {
        return data;
    }
    public byte[] getBytes() {
        return frame;
    }
    public static class FrameCorruptException extends Exception {
        public FrameCorruptException() {
        }
    }
}
