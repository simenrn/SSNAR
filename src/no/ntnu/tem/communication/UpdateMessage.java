/*
 * This code is written as a part of a Master Thesis
 * the spring of 2017.
 *
 * Kristian Lien(Master 2017 @ NTNU)
 */
package no.ntnu.tem.communication;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * A received update message has the follow fields, with byte length in parentheses:
 * X (2) | Y (2) | Heading (2) | Tower angle (2) | Sensor1 (1) | Sensor2 (1) | Sensor3 (1) | Sensor4 (1) 
 * 
 * Units are cm for lengths, and degrees for angles
 * 
 * @author Kristian
 */
public class UpdateMessage {
    
    public final static int IR_MAX_VALUE = 90;
    public final static int IR_MIN_VALUE = 0;
    public final static int TOWER_HEADING_MAX = 100;
    public final static int TOWER_HEADING_MIN = 0;
    
    private int x;
    private int y;
    private int heading;
    private int towerAngle;
    private int[] sensorValues;
    private byte[] data;
    public UpdateMessage(byte[] data) throws Message.MessageCorruptException, Message.ValueCorruptException {
        ByteBuffer buffer = ByteBuffer.wrap(data);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        this.data = data;
        if(buffer.remaining() != 12)  throw new Message.MessageCorruptException();
        x = buffer.getShort();
        y = buffer.getShort();
        heading = buffer.getShort();
        
        towerAngle = buffer.getShort();
        if(towerAngle < TOWER_HEADING_MIN || towerAngle > TOWER_HEADING_MAX) throw new Message.ValueCorruptException();
        
        int i;
        sensorValues = new int[4];
        for(i=0;i<4;i++) {
            sensorValues[i] = buffer.get();
            if(sensorValues[i] < IR_MIN_VALUE || sensorValues[i] > IR_MAX_VALUE) throw new Message.ValueCorruptException();
            
        }
    }
    public byte[] getBytes() {
        return data;
    }
    public int[] getPosition() { return new int[]{x,y}; }
    public int getHeading() { return heading; }
    public int getTowerAngle() { return towerAngle; }
    public int[] getSensorValues() { return sensorValues; }
}
