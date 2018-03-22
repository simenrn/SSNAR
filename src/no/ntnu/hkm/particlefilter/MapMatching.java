package no.ntnu.hkm.particlefilter;

import java.util.HashMap;
import java.util.Map;

/**
 * Calculates correlation score between maps.
 * Simply how good overlap it is between the local and global map.
 * @author Henrik
 */
public class MapMatching {
    private final HashMap<String, Integer> globalMap;
    private final HashMap<String, Integer> localMap;
    private final double convergence; //Controls rate of convergence (weighting factor) 
    
    public MapMatching(HashMap<String, Integer> globalMap, HashMap<String, Integer> localMap, double convergence){
        this.globalMap = globalMap;
        this.localMap = localMap;
        this.convergence = convergence;
    }
    
    /**
     * The main method that calculates correlation score.
     * @return correlation score
     */
    public double correlationScore(){
        int w = 0;
        for(Map.Entry<String, Integer> entry : localMap.entrySet()){
            //Location is observed in both maps, and wall in localMap
            if(globalMap.containsKey(entry.getKey()) && entry.getValue() == 1){
                if(globalMap.get(entry.getKey()).equals(entry.getValue())){
                    w++;
                }
                else{
                    w--;
               }
            }
        }
        double weight = Math.exp(convergence*w);

        return weight;
    }
    
}
