/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.mycompany.vpp.core;

/**
 *
 * @author anton
 */
public class EquilibriumCalculator {
    private ResistanceCalculator rc=null;
    private AeroCalculator ac=null;
    private double bestSpeed;
    private double bestDrive;
    private double bestSideForce;
    private double bestHeelAngle;
    private double [][] totalRes;
    private double [][] keelSideForce;


    public EquilibriumCalculator(ResistanceCalculator rc, AeroCalculator ac) {
        this.rc=rc;
        this.ac=ac;
    }
    
    public void calculateBestSolution(double leewayAngle, double froudeNumber, double flattener, double reef){
        totalRes=rc.calculateTotalResistance(leewayAngle);
        keelSideForce=rc.calculateSideForce(leewayAngle);
        ac.setFlattener(flattener);
        ac.setReefFun(reef);
        rc.calculateSlopeAndIntercept();
        double slope=rc.getSlope();
        double intecept=rc.getIntercept();
        double tc=rc.getTc();
        double keelSpan=rc.getSpan();
        
        ac.calculateHeelingArm(tc, keelSpan);
        
        double best[]=new double[4];
        best[3]=100;
        double tempBestSpeed=0;
        double tempAngle=0;
        double basicAngle=20*Math.PI/180;
        while(tempAngle!=basicAngle){
            best=ac.calculateDriveAndSideForce(0, froudeNumber, rc.getLengthWL(), basicAngle, slope, intecept);//result[0]=total drive force; result[1]=total side force; result[2]=new heel angle; result[3]=side force to drive force ratio
            tempAngle=basicAngle;
            basicAngle=best[2]*Math.PI/180;
        }
        bestSpeed=froudeNumber*Math.sqrt(9.81*rc.getLengthWL())/0.5144;//knots
        bestHeelAngle=best[2];
        bestDrive=best[0];
        bestSideForce=best[1];
        
    }

    public double getBestSpeed() {
        return bestSpeed;
    }

    public double getBestDrive() {
        return bestDrive;
    }

    public double getBestSideForce() {
        return bestSideForce;
    }

    public double getBestHeelAngle() {
        return bestHeelAngle;
    }
    
    public double getTrueWindSpeed(){
        return ac.getTrueWindSpeed();
    }
    public double getTrueWindAngle(){
        return ac.getTrueWindAngle();
    }

    public double[][] getTotalRes() {
        return totalRes;
    }
    public double getLengthWL(){
        return rc.getLengthWL();
    }

    public double[][] getKeelSideForce() {
        return keelSideForce;
    }
    
    
}
