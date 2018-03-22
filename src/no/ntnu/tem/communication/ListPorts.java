/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.communication;

import gnu.io.*;
import java.util.LinkedList;

/**
 * This class provides functionality for listing all available com-ports.
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class ListPorts {

    /**
     * Method for listing available ports at the PC
     *
     * @return linkedList with Strings containing available com Ports
     */
    public LinkedList<String> listPorts() {
        LinkedList portList = new LinkedList<String>();

        java.util.Enumeration<CommPortIdentifier> portEnum =
                CommPortIdentifier.getPortIdentifiers();
        while (portEnum.hasMoreElements()) {
            CommPortIdentifier portIdentifier = portEnum.nextElement();
            portList.add(portIdentifier.getName());
        }
        return portList;
    }

    /**
     * Method for identifying which port type the given port is
     *
     * @param portType Selected port
     * @return Port type
     */
    static String getPortTypeName(int portType) {
        switch (portType) {
            case CommPortIdentifier.PORT_I2C:
                return "I2C";
            case CommPortIdentifier.PORT_PARALLEL:
                return "Parallel";
            case CommPortIdentifier.PORT_RAW:
                return "Raw";
            case CommPortIdentifier.PORT_RS485:
                return "RS485";
            case CommPortIdentifier.PORT_SERIAL:
                return "Serial";
            default:
                return "unknown type";
        }
    }
}
