package no.ntnu.hkm.particlefilter;

import java.io.FileWriter;
import java.io.IOException;
import no.ntnu.et.general.*;
import no.ntnu.et.map.*;
import no.ntnu.tem.robot.*;
import no.ntnu.tem.robot.Measurement;
import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;

/**
 * This class implements a particle filter to improve the position estimate of
 * the robots. 
 * 
 * @author Henrik
 */
public class Particlefilter extends Thread {
    private int numberOfParticels;
    Vector<Particle> particles = new Vector(numberOfParticels);
    private final int windowSize;
    private final double convergence;
    private double moved;
    private final double Mx;
    private final double R;
    private GridMap map;
    private GridMap winnerMap;
    private final Robot robot;
    private Pose currentPose;
    private Pose previousPose;
    private final Random rand;
    private boolean updatedSensors = false;
    private boolean paused;
    private final ArrayBlockingQueue<Measurement> measurementQueue = new ArrayBlockingQueue<>(10000);
    
    public Particlefilter(Robot robot, GridMap map, double[] options){
        this.robot = robot;
        this.map = map;
        winnerMap = map;
        numberOfParticels = (int)options[0];
        convergence = options[1];
        windowSize = (int)options[2];
         //Uncertanty model
        Mx = options[3];
        R = options[4];
        rand = new Random();
        moved = 0;
        currentPose = new Pose(robot.getPosition()[0] , robot.getPosition()[1], robot.getRobotOrientation()); 
        for(int i = 0; i < numberOfParticels; i++){
            particles.add(new Particle(currentPose, 0.01, robot, map.getCellSize(),convergence));
        }
    }
    
    public GridMap getMap(){
        //This function also prepares the map!
        printParticleDataToFile();
        return winnerMap;
    }
    
    /**
     * Removes all the rendered particles from map image.
     */
    public void cleanParticles(){
        for(Particle particle : particles){
            map.removeParticle(map.findLocationInMap(particle.getPosition())); 
        }
    }
    
    /**
     * Calculate distance between two positions.
     * 
     * @param previousPosition
     * @param currentPosition
     * @return distance
     */
    public double distanceBetweenPoints(Pose previousPosition, Pose currentPosition){
        double distance = Math.sqrt(Math.pow(previousPosition.getPosition().getXValue() - currentPosition.getPosition().getXValue(), 2) 
                                  + Math.pow(previousPosition.getPosition().getYValue() - currentPosition.getPosition().getYValue(), 2));
        if(distance == 0) return 0;
        //Test if robot is reversing
        Pose testmove = new Pose(previousPosition);
        testmove.move(-distance);
        if(Math.sqrt(Math.pow(testmove.getPosition().getXValue() - currentPosition.getPosition().getXValue(), 2) 
            + Math.pow(testmove.getPosition().getYValue() - currentPosition.getPosition().getYValue(), 2)) > distance ){
            return -distance;
        }
        return distance;
    }

    /**
     * Updates the particles position by drawing from the motion model,
     * accounting for noise by using a Gaussian distribution.
     */
    public void predictionStep(){
        //remove rendered particles
        cleanParticles();
        //Update movement of robot
        previousPose = currentPose;
        int orientation = robot.getRobotOrientation();
        currentPose = new Pose(robot.getPosition()[0] , robot.getPosition()[1], orientation);
        double distance = distanceBetweenPoints(currentPose, previousPose);
        moved += distance;
        //Propogate particles
        for(Particle particle : particles){
            particle.getPose().setHeading(new Angle(orientation+Math.abs(orientation - previousPose.getHeading().getValue())*rand.nextGaussian()*R));
            particle.getPose().move(distance + distance*rand.nextGaussian()*Mx);
            map.addParticle(map.findLocationInMap(particle.getPosition()));
            particle.addHistory();
        }
    }
    
    /**
     * Weight the particles according to the current map of the particle with
     * respect to an other map using a map-matching algorithm.
     * Update weights based on observations: w_i = p(z|m,x_i) = target/proposal.
     */
    public void correctionStep(){
        for(Particle particle : particles){
            Vector<Pose> history = particle.getHistory();
            Pose last = history.remove(history.size()-1);
            for(Pose pose : history){
                //Check history for return to a known place
                if(Math.abs(pose.getPosition().getXValue() - particle.getPose().getPosition().getXValue()) < 10
                && Math.abs(pose.getPosition().getXValue() - particle.getPose().getPosition().getXValue()) < 10){
                    //Estimate of w_i = P(z|m,x_i) based on Map matching
                    particle.MapMatch();
                }
            }
            history.add(last);
            particle.updateGlobalMap();
        }
        //Debugg check particle weights
        double weight = 0;
        for(Particle particle : particles){
            particle.updateGlobalMap(); //Add the last gathered data
            if(particle.getWeight() > weight){
                weight = particle.getWeight();
            }
        }
    }
    
    /**
     * Uses low variance resampling to improve particle distribution.
     */
    public void resample(){
        //Normalize weigths
        double totWeight = 0;
        double[] w = new double[numberOfParticels];
        for(int i = 0; i < numberOfParticels; i++){
            w[i] = particles.get(i).getWeight();
            totWeight += particles.get(i).getWeight();
        }
        for(int i = 0; i < numberOfParticels; i++){
            w[i] /= totWeight;
        }      
        //Resampleing criteria: only resample if number of effictive particles is low
        boolean enable = true;
        if(enable == true){
            double weightsqrd = 0;
            for(int i = 0; i < numberOfParticels; i++){
                weightsqrd += Math.pow(w[i],2);
            }
            if((1/weightsqrd) > (numberOfParticels/2)) {
                for(int i = 0; i < numberOfParticels; i++){
                    particles.get(i).setWeight(w[i]);
                }
                return;
            }
        }
        Vector<Particle> resampled = new Vector();
        //Resample
        double r = rand.nextDouble()*(1.0/(double)numberOfParticels);
        double U;
        double c = w[0];
        int i = 0;
        for(int j = 0; j < numberOfParticels; j++){
            U = r + j/((double)numberOfParticels);
            while(U > c){
                i++;
                c += w[i];
            }
            resampled.add(new Particle(particles.get(i),robot,convergence));
        }
        cleanParticles();
        particles = resampled;
        //Redraw
        for(Particle particle : particles){
            map.addParticle(map.findLocationInMap(particle.getPosition()));         
        }
    }

    /**
     * External method called by MappingController to update the maps of the particles.
     * For thread safety the measurement is put in a queue.
     * 
     * @param measurement 
     */
    public void updateMeasurement(Measurement measurement){
        Measurement copyMeasurement = new Measurement(measurement);
        measurementQueue.add(copyMeasurement);
        updatedSensors = true;
    }
    
    /**
     * The measurement queue is pushed the the particle map.
     */
    public void updateParticleMapData(){
        while(!measurementQueue.isEmpty()){
            Measurement measurement = measurementQueue.remove();
            for(Particle particle : particles){
                particle.updateLocalMap(measurement);
            }
        }
        updatedSensors = false;
    }
    
    @Override
    public void start(){
        if(!isAlive()){
            super.start();
        }
        paused = false;
    }

    /**
     * Pauses the mapping
     */
    public void pause(){
        paused = true;
    }
    
    /**
     * Finds the current best particle and prints the data to a csv file
     */
    public void printParticleDataToFile(){
        Particle winner = null;
        double weight = 0;
        for(Particle particle : particles){
            particle.updateGlobalMap(); //Add the last gathered data
            if(particle.getWeight() > weight){
                winner = particle;
                weight = particle.getWeight();
            }
        }
        //Add latest changes
        winner.updateGlobalMap();
        String string = "";
        for(Map.Entry<String, Integer> entry : winner.getGlobalMap().entrySet()){
            if(entry.getValue() == 1){
                string += (entry.getKey().toString()+System.lineSeparator());
            }
        }
        try (FileWriter fw = new FileWriter(robot.getName()+"data.csv", false)) {
            fw.write(string);
            fw.flush();
        } catch (IOException e) {
            System.err.println("IOexception: " + e.getMessage());
        }
        makeParticleMap(winner);
    }

    /**
     * Makes a new map from winning particle data.
     * @param winner 
     */
    private void makeParticleMap(Particle winner){
        winnerMap = new GridMap(map.getCellSize(),40,40);
        for(Map.Entry<String, Integer> entry : winner.getGlobalMap().entrySet()){
            String cordsString = entry.getKey();
            int index = cordsString.indexOf(",");
            int x = Integer.parseInt(cordsString.substring(0, index));
            int y = Integer.parseInt(cordsString.substring(index+1,cordsString.length()));
            MapLocation loc = new MapLocation(y,x); //Swithced somewhere...
            Position pos = new Position((int)loc.getColumn()*map.getCellSize()+1, (int)loc.getRow()*map.getCellSize()+1);
            winnerMap.resize(pos);
            winnerMap.addMeasurement(loc, entry.getValue()==1);
        }
        winnerMap.cleanUp();
    }
    
    /**
     * Returns if the mapping is running or paused
     * @return 
     */
    public boolean isRunning(){
        return !paused;
    }
    
    @Override
    public void run() {
        while (true) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
                break;
            }
            if (paused){
                continue;
            }
            predictionStep();
            if(moved > windowSize){
                correctionStep();
                resample();
                moved = 0;
            }
            if(updatedSensors){
                updateParticleMapData();
            }
        }
    }

}
