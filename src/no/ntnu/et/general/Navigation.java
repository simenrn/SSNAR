/**
 * This code is written as a part of a Master Thesis
 * the fall of 2017.
 *
 * Geir Eikeland (Master 2017 @ NTNU)
 */
package no.ntnu.et.general;

/**
 *
 * @author geirhei
 */
public class Navigation {
    
    /**
     * Method for retrieving the index of the array that has the lowest value
     * 
     * @param distances array [0,360)
     * @return int angle [0,360)
     */
    public static int getShortestDistanceHeading(int[] distances) {
        int angle = -1;
        int shortest = Integer.MAX_VALUE;
        for (int i = 0; i < distances.length; i++) {
            if (distances[i] < shortest) {
                angle = i;
            }
        }
        return angle;
    }
}
