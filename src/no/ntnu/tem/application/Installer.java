/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Thor Eivind Andersen and Mats Rødseth (Master 2016 @ NTNU)
 */
package no.ntnu.tem.application;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Thor Eivind and Mats (Master 2016 @ NTNU)
 */
public class Installer {

    private final static String RXTXP32 = "rxtxParallel32.dll";
    private final static String RXTXS32 = "rxtxSerial32.dll";

    private final static String RXTXP64 = "rxtxParallel64.dll";
    private final static String RXTXS64 = "rxtxSerial64.dll";

    private final static String RXTXS = "rxtxSerial.dll";
    private final static String RXTXP = "rxtxParallel.dll";

    private final static String RXTXMAC = "librxtxSerial.jnilib";

    private final static String BIN = "bin";

    /**
     * Method that generates (copies and replaces) the system dependent
     * libraries
     */
    public static void generateSystemDependantLibraries() {
        if (System.getProperty("os.name").startsWith("Windows")) {
            if (System.getProperty("os.arch").equals("x86")) {
                copyFile(BIN + "/" + RXTXS32, RXTXS);
                copyFile(BIN + "/" + RXTXP32, RXTXP);
            } else {
                copyFile(BIN + "/" + RXTXS64, RXTXS);
                copyFile(BIN + "/" + RXTXP64, RXTXP);
            }
        } else if (System.getProperty("os.name").startsWith("Mac")) {
            copyFile(BIN + "/" + RXTXMAC, RXTXMAC);
        }
    }

    /**
     * Method that copies a file from a given folder to a new folder
     *
     * @param from existing folder
     * @param to new folder
     */
    private static void copyFile(String from, String to) {
        InputStream in = null;
        OutputStream out = null;

        try {
            File s = new File(from);
            File sc = new File(to);

            in = new FileInputStream(s);
            out = new FileOutputStream(sc);

            byte[] buffer = new byte[1024];

            int length;

            while ((length = in.read(buffer)) > 0) {
                out.write(buffer, 0, length);
            }

            in.close();
            out.close();

        } catch (FileNotFoundException ex) {
            Logger.getLogger(Installer.class.getName()).log(Level.SEVERE, null, ex);
        } catch (IOException ex) {
            Logger.getLogger(Installer.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

}
