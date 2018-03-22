/*
 * This code is written as a part of a Master Thesis
 * the spring of 2017.
 *
 * Kristian Lien (Master 2017 @ NTNU)
 */
package no.ntnu.tem.communication;

/**
 *
 * @author Kristian
 */
public interface Protocol {
    public void send(int address, byte[] data);
    public void receive(int address, byte[] data);
}
