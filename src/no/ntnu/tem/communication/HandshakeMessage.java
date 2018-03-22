/*
 * This code is written as a part of a Master Thesis
 * the spring of 2017.
 *
 * Kristian Lien (Master 2017 @ NTNU)
 */
package no.ntnu.tem.communication;

import java.io.DataOutputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * A received handshake message has the follow fields, with byte length in parentheses:
 * Name (1-10) | Name length (1) | Width (2) | Length (2) | Tower offset X (1) | Tower offset Y (1)
 * Axel offset (1) | Sensor offset (1) | Sensor offset (1) | Sensor offset (1) | Sensor offset (1)
 * Sensor1 heading (2) | Sensor2 heading (2) | Sensor3 heading (2) | Sensor4 heading (2) | Deadline (2)

 * @author Kristian
 */
public class HandshakeMessage {
    public static final int BASE_LENGTH = 22;
    public static final int MIN_NAME_LENGTH = 1;
    public static final int MAX_NAME_LENGTH = 10;
    private String name;
    private int width;
    private int length;
    private int towerOffsetX;
    private int towerOffsetY;
    private int axelOffset;
    private int[] sensorOffsets;
    private int[] sensorHeadings;
    private int deadline;
    private byte[] data;
    
    public HandshakeMessage(byte[] data) throws Message.MessageCorruptException, Message.ValueCorruptException {
        ByteBuffer buffer = ByteBuffer.wrap(data);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        if(buffer.remaining() < (BASE_LENGTH+MIN_NAME_LENGTH) || buffer.remaining() > (BASE_LENGTH+MAX_NAME_LENGTH)) throw new Message.MessageCorruptException();
        int nameLength = buffer.get();
        if(nameLength < MIN_NAME_LENGTH || nameLength > MAX_NAME_LENGTH) throw new Message.MessageCorruptException();
        
        byte[] n = new byte[nameLength];
        buffer.get(n);
        
        this.name = new String(n);

        width = buffer.getShort();
        length = buffer.getShort();
        towerOffsetX = buffer.get();
        towerOffsetY = buffer.get();
        axelOffset = buffer.get();
        int i;
        sensorOffsets = new int[4];
        sensorHeadings = new int[4];
        for(i=0;i<4;i++) sensorOffsets[i] = buffer.get();
        for(i=0;i<4;i++) sensorHeadings[i] = buffer.getShort();

        deadline = buffer.getShort();
        this.data = data;
    }
    public byte[] getBytes() {
        return data;
    }
    public String getName() { return name; }
    public int getWidth() { return width; }
    public int getLength() { return length; }
    public int[] getTowerOffsets() { return new int[]{towerOffsetX, towerOffsetY}; }
    public int getAxelOffset() { return axelOffset; }
    public int[] getSensorOffsets() { return sensorOffsets; }
    public int[] getSensorHeadings() { return sensorHeadings; }
    public int getDeadline() { return deadline; }
}
