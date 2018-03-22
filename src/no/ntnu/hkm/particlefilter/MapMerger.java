package no.ntnu.hkm.particlefilter;

import no.ntnu.et.map.*;
import no.ntnu.et.general.Position;
import Jama.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.Arrays;
import java.util.Random;

/**
 * Merges two maps into one.
 * @author hkm
 */
public class MapMerger {
    private GridMap map1;
    private GridMap map2;
    private GridMap mergedMap;
    private Matrix map1Mat;
    private Matrix map2Mat;  
    private Matrix adjustedMap2;  
    private Matrix T;
    private Matrix Tproposal;
    private double theta;
    private double thetaProposal;
    private Random rand;
    int iter_max;//Max iterration counter
    double CLock; //scaling factor, necessary overlap between the maps (aviod overfitting)
    double Temp;
    
    /**
     * Tries to fit two partially overlapping maps together.
     * @param map1
     * @param map2 
     */
    public MapMerger(GridMap map1, GridMap map2, int iter, int temp, double CLock){
        this.map1 = map1;
        this.map2 = map2;
        this.iter_max = iter;
        this.Temp = temp;
        this.CLock = CLock;
        resizeMaps();
        map1Mat = makeMatrix(this.map1); 
        map2Mat = makeMatrix(this.map2);
        rand = new Random();
        initTMatrix();
        Merge();
    }

    public GridMap getMergedMap(){
        return mergedMap;
    }
    
    /**
     * Makes a NxM matrix from the map.
     * @param map
     * @return 
     */
    private Matrix makeMatrix(GridMap map){        
        ConcurrentHashMap<MapLocation, Cell> grid = map.getMap();
        double[][] array = new double[this.map1.getMapWidth()/this.map1.getCellSize()][this.map1.getMapHeight()/this.map1.getCellSize()];
        for (double[] row: array) Arrays.fill(row, 0);
        for(int x = 0; x < this.map1.getMapWidth()/this.map1.getCellSize(); x++){
            for(int y = 0; y < this.map1.getMapHeight()/this.map1.getCellSize(); y++){
                Cell cell = grid.get(new MapLocation(map.getBottomRow()+y,map.getLeftColumn()+x));
                if(cell.isOccupied()){
                    array[x][y] = 1;
                }
                else if(cell.isFree()){
                    array[x][y] = -1;
                }
                else{
                    array[x][y] = 0;
                }
            }
        }
         return new Matrix(array);
     }
    
    /**
     * Resizes the maps, so they are of the same size.
     * Adds some padding (p) at the boarders.
     */
    private void resizeMaps(){
        int p = 50;
        map1.resize(new Position(Math.floor(map2.getTopRow()*map2.getCellSize()+p), Math.floor(map2.getRightColumn()*map2.getCellSize()+p)));
        map2.resize(new Position(Math.floor(map1.getTopRow()*map1.getCellSize()+p), Math.floor(map1.getRightColumn()*map1.getCellSize()+p)));
        map1.resize(new Position(Math.floor(map1.getTopRow()*map1.getCellSize()+p), Math.floor(map1.getRightColumn()*map1.getCellSize()+p)));
        map2.resize(new Position(Math.floor(map2.getTopRow()*map2.getCellSize()+p), Math.floor(map2.getRightColumn()*map2.getCellSize()+p)));        
        //Also in negative direction
        map1.resize(new Position(Math.floor(map2.getBottomRow()*map2.getCellSize()-p), Math.floor(map2.getLeftColumn()*map2.getCellSize()-p)));
        map2.resize(new Position(Math.floor(map1.getBottomRow()*map1.getCellSize()-p), Math.floor(map1.getLeftColumn()*map1.getCellSize()-p)));
        map1.resize(new Position(Math.floor(map1.getBottomRow()*map1.getCellSize()-p), Math.floor(map1.getLeftColumn()*map1.getCellSize()-p)));
        map2.resize(new Position(Math.floor(map2.getBottomRow()*map2.getCellSize()-p), Math.floor(map2.getLeftColumn()*map2.getCellSize()-p)));
    }
    
    /**
     * Initializes the transformation matrix p.
     */
    private void initTMatrix(){
        double[][] array = new double[3][3];
        array[0][0] = 1;
        array[0][1] = 0;
        array[0][2] = 0;
        array[1][0] = 0;
        array[1][1] = 1;
        array[1][2] = 0;
        array[2][0] = 0;
        array[2][1] = 0;
        array[2][2] = 1;
        Tproposal = new Matrix(array);
        T = new Matrix(array);
        theta = 0;
        thetaProposal = 0;
    }
    
    private Matrix adaptiveRandomWalk(double Temp){
        //Creates a neighbouring state
        Tproposal = T.copy();
        double mode = rand.nextDouble();
        if(mode < 0.33){
            thetaProposal = theta + Math.floor(rand.nextGaussian()*Math.exp(Temp*0.0003));  
            double cos = Math.cos(Math.toRadians(thetaProposal));
            double sin = Math.sin(Math.toRadians(thetaProposal));

            //Shift center of rotation to image center
            int xcent = map2Mat.getRowDimension()/2;
            int ycent = map2Mat.getColumnDimension()/2;
            double[][] tempMat = new double[map2Mat.getRowDimension()][map2Mat.getColumnDimension()];
            for (double[] row: tempMat) Arrays.fill(row, 0);
            Matrix transformedMap2 = new Matrix(tempMat);
            //Transform T * [x,y,1]^T
            for(int y = 0; y < map2Mat.getColumnDimension(); y++){
                for(int x = 0; x < map2Mat.getRowDimension(); x++){
                    //Shift to origin
                    double px = x - xcent;
                    double py = y - ycent;
                    //Rotate
                    double tempX = px*cos - py*sin;
                    double tempY = px*sin + py*cos;
                    //Translate back
                    px = tempX + ycent;
                    py = tempY + xcent;
                    if(px < transformedMap2.getRowDimension()    && px > -1 
                    && py < transformedMap2.getColumnDimension() && py > -1 ) {
                        transformedMap2.set((int)px, (int)py, map2Mat.get(x, y));
                    }
                }
            }
            return transformedMap2;
        }
        else if(mode < 0.66){
            double dx = T.get(0, 2) + Math.floor(rand.nextGaussian()*Math.exp(Temp*0.0003));
            Tproposal.set(0, 2, dx);
        }
        else{
            double dy = T.get(1, 2) + Math.floor(rand.nextGaussian()*Math.exp(Temp*0.0003));
            Tproposal.set(1, 2 , dy);
        }
        double[][] tempMat = new double[map2Mat.getRowDimension()][map2Mat.getColumnDimension()];
        for (double[] row: tempMat) Arrays.fill(row, 0);
        Matrix transformedMap2 = new Matrix(tempMat);
        //Transform T * [x,y,1]^T
        for(int y = 0; y < map2Mat.getColumnDimension(); y++){
            for(int x = 0; x < map2Mat.getRowDimension(); x++){
                double[][] cords = new double[3][1];
                cords[0][0] = x;
                cords[1][0] = y;
                cords[2][0] = 1;
                Matrix cordm = new Matrix(cords);
                Matrix shifted = Tproposal.times(cordm);
                if((int)shifted.get(0, 0) < transformedMap2.getRowDimension()
                && (int)shifted.get(0, 0) > -1 
                && (int)shifted.get(1, 0) < transformedMap2.getColumnDimension()
                && (int)shifted.get(1, 0) > -1 ) {
                    transformedMap2.set((int)shifted.get(0, 0), (int)shifted.get(1, 0), map2Mat.get(x, y));
                }
            }
        }
        return transformedMap2;
    }
    
    private Matrix dmapc(Matrix map, int mode){
        double[][] array = new double[map.getRowDimension()][map.getColumnDimension()];
        Matrix dmapc = new Matrix(array);
        for(int y = 0; y < map.getColumnDimension(); y++){
            for(int x = 0; x < map.getRowDimension(); x++){
                if(map.get(x, y) == mode){
                    dmapc.set(x, y, 0);
                }
                else{
                    dmapc.set(x, y, Double.MAX_VALUE);
                }
            }
        }
        double h = 0;
        for(int y = 1; y < map.getColumnDimension()-1; y++){
            for(int x = 1; x < map.getRowDimension()-1; x++){
                h = Math.min(dmapc.get(x-1, y)+1, dmapc.get(x, y-1)+1);
                dmapc.set(x, y, Math.min(dmapc.get(x, y), h));
            }
        }

        for(int y = map.getColumnDimension()-2; y > -1 ; y--){
            for(int x = map.getRowDimension()-2; x > -1 ; x--){
                h = Math.min(dmapc.get(x+1, y)+1, dmapc.get(x, y+1)+1);
                dmapc.set(x, y, Math.min(dmapc.get(x,y),h));
            }
        }
        return dmapc;
     }
    
    private double computeD(Matrix firstMap, Matrix dmapc, int mode){
        double d = 0;
        for(int y = 0; y < firstMap.getColumnDimension(); y++){
            for(int x = 0; x < firstMap.getRowDimension(); x++){
                if(firstMap.get(x, y) == mode){
                    d = d + dmapc.get(x, y);
                }
            }
        }
        return d;
    }
    
    private GridMap matrixToMap(Matrix map){
        GridMap newMap = new GridMap(map1.getCellSize(), map.getRowDimension()*map1.getCellSize() , map.getColumnDimension()*map1.getCellSize() );
        int i = 0;
        for(int y = newMap.getBottomRow(); y < newMap.getTopRow(); y++){
            int j = 0;
            for(int x = newMap.getLeftColumn(); x < newMap.getRightColumn(); x++){
                if(map.get(j, i) >= 1){ //Får rare out of bounds problemer, neg num?
                    newMap.addMeasurement(new MapLocation(y,x), true);
                }
                else if(map.get(j, i) <= -1){
                    newMap.addMeasurement(new MapLocation(y,x), false);
                }
                j++;
            }
            i++;
        }
        return newMap;        
    }
    
    private double agr(Matrix map1, Matrix map2){
        int agr = 0;
        for(int y = 0; y < map1.getColumnDimension(); y++){
            for(int x = 0; x < map1.getRowDimension(); x++){
                if(map1.get(x, y) == map2.get(x, y) && map1.get(x, y) != 0){
                    agr++;
                }
            }
        }
        return agr;
    }
    
    private double dis(Matrix map1, Matrix map2){
        int dis = 0;
        for(int y = 0; y < map1.getColumnDimension(); y++){
            for(int x = 0; x < map1.getRowDimension(); x++){
                if(map1.get(x, y) != map2.get(x, y) && map1.get(x, y) != 0 && map2.get(x, y) != 0){
                    dis++;
                }
            }
        }
        return dis;
    }
    
    private double calculateFitness(Matrix sample){
        double d12c1 = computeD(map1Mat, dmapc(sample, 1),1);
        double d12c2 = computeD(map1Mat, dmapc(sample, -1),-1);

        double d21c1 = computeD(sample, dmapc(map1Mat, 1),1);
        double d21c2 = computeD(sample, dmapc(map1Mat, -1),-1);

        double psi = d12c1 + d12c2 + d21c1 + d21c2; //Perfect overlap --> psi = zero
        return psi;
    }
    
    private boolean randomSelector(double Temp, double delta, double cs){
        if(delta < cs)return true;
        if(Temp <= 0)return false;
        double prob = Math.exp(-(delta - cs)/Temp);
        return rand.nextDouble() < prob;
    }
    
    private void Merge(){
        double cs = Double.MAX_VALUE; 
        Double ai; //percent match
        int iter = 0; 
        double dT = Temp/iter_max;
        Matrix walkedMap2 = map2Mat.copy();
        int resetCount = 0;
        Matrix sample;
        Matrix tempBest = null;
        //System.out.println("Starting merger:");
        while(true){ 
            //New sample, but test original spot first
            if(iter == 0){
                sample = walkedMap2;
            }
            else{
                sample = adaptiveRandomWalk(Temp);
            }
            double psi = calculateFitness(sample);
            //Overfitting adaption
            double dis = dis(map1Mat,sample);
            double agr = agr(map1Mat,sample);
            double delta = psi + CLock*(dis-agr);
            ai = 1.0 - (dis/(agr+dis));
            if(randomSelector(Temp, delta, cs)){
                cs = delta;
                walkedMap2 = sample;
                T = Tproposal;
                theta = thetaProposal;
                System.out.println("iter: "+iter+" ai(): "+ai+" delta: "+delta+" psi: "+psi+" Temp: "+Temp+" T(theta, dx, dy): "+theta+","+T.get(0, 2)+","+T.get(1, 2));
            }
            if((cs < 1) || iter > iter_max ){
                boolean reset = false;
                if((ai < 0.95 || ai == Double.NaN) && resetCount < 10 && reset) {
                    //Resets if bad match
                    tempBest = walkedMap2;
                    iter = 1;
                    resetCount++;
                    cs =  Double.MAX_VALUE;
                    initTMatrix();
                    Temp = 10000;
                    //Tries to adjust CLock
                    CLock++;
                    continue;
                }
                else if((ai < 0.97 || ai == Double.NaN) && resetCount >= 10){
                    walkedMap2 = tempBest;
                }
                adjustedMap2 = walkedMap2;
                break;
            }
            Temp -= dT;
            iter++;            
        }
        
        mergedMap = matrixToMap(map1Mat.plus(adjustedMap2));
        mergedMap.cleanUp();
    }
    
}
