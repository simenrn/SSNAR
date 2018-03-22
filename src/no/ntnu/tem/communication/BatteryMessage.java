/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Lars Marius Strande (Master 2017 @ NTNU)
 */
package no.ntnu.tem.communication;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 *
 * @author larsmast
 */
public class BatteryMessage {

    private byte[] data;
    private double batLevel;

    public BatteryMessage(byte[] data) throws Message.MessageCorruptException, Message.ValueCorruptException {
        ByteBuffer buffer = ByteBuffer.wrap(data);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        this.data = data;
        batLevel = buffer.getShort();
    }

    public byte[] getBytes() {
        return data;
    }
    
    public double getBatLevel() {return batLevel;}
}
