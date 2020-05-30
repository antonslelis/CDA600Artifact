/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.mycompany.vpp.core;

import static com.mycompany.vpp.core.ResiduaryCoef.APPARENT_WIND_ANGLES;
import static com.mycompany.vpp.core.ResiduaryCoef.SAIL_LIFT_COEF;
import static com.mycompany.vpp.core.ResiduaryCoef.SAIL_VIS_DRAG_COEF;

/**
 *
 * @author anton
 */
public class AeroCalculator {
    //
    //MAIN PARAMETERS
    //
    private double mainHeight;
    private double mainWidth;
    private double mainBAD;//height of main boom above sheer
    private double mainArea;
    //
    //JIB PARAMETERS
    //
    private double jibHeight;
    private double jibWidth;
    private double jibLPP;//perpendicular of longest jib
    private double jibArea;
    //
    //WIND PARAMETERS
    //
    private double trueWindSpeed;// m/s
    private double trueWindAngle;//degrees
    private double airDensity;
    
    //
    //GENERAL PARAMETERS
    //
    private double flattener;
    private double reefFun;
    private double mastDiameter;
    private double freeboard;
    private double hullBeam;
    private double totalArea;
    //
    //BALANCE PARAMETERS
    //
    private double heelingArm;
    
    
    public AeroCalculator(double mainHeight, double mainWidth, double mainBAD, double jibHeight, double jibWidth, double jibLPP, double trueWindSpeed, double trueWindAngle, double airDensity, double freeboard, double hullBeam, double mastDiameter, double flattener, double reefFun) {
        this.mainHeight = mainHeight;
        this.mainWidth = mainWidth;
        this.mainBAD = mainBAD;
        this.jibHeight = jibHeight;
        this.jibWidth = jibWidth;
        this.jibLPP = jibLPP;
        this.trueWindSpeed = trueWindSpeed;
        this.trueWindAngle = trueWindAngle;
        this.airDensity = airDensity;
        this.freeboard = freeboard;
        this.hullBeam = hullBeam;
        this.mastDiameter = mastDiameter;
        this.flattener=flattener;
        this.reefFun=reefFun;
        
        this.mainArea=0.5*mainHeight*mainWidth*1.15;
        double subtotal=Math.pow(jibHeight, 2)+Math.pow(jibWidth, 2);
        this.jibArea=Math.sqrt(subtotal)*0.5*jibLPP;
        this.totalArea=this.mainArea+this.jibArea;
    }

    public void setFlattener(double flattener) {
        this.flattener = flattener;
    }

    public void setReefFun(double reefFun) {
        this.reefFun = reefFun;
    }
    
    public double calculateLiftCoef(int i){
        double totalLift=(SAIL_LIFT_COEF[0][i]*mainArea+SAIL_LIFT_COEF[1][i]*jibArea)/totalArea;
        if(flattener>0){
            totalLift=totalLift*flattener;
        }
        return totalLift;
    }
    public double calculateVisDragCoef(int i){
        double totalVisDrag=(SAIL_VIS_DRAG_COEF[0][i]*mainArea+SAIL_VIS_DRAG_COEF[1][i]*jibArea)/totalArea;
        return totalVisDrag;
    }
    public double calculateInducedDragCoef(int i, double liftCoef){
        if(APPARENT_WIND_ANGLES[i]<=50){
            double subtotal=1.1*(mainHeight+mainBAD+freeboard);
            double aspectRatio=Math.pow(subtotal, 2)/totalArea;
            double totalIndDrag=Math.pow(liftCoef,2)*((1/(Math.PI*aspectRatio))+0.005);
            return totalIndDrag; 
        }else{
            double subtotal=1.1*(mainHeight+mainBAD);
            double aspectRatio=Math.pow(subtotal, 2)/totalArea;
            double totalIndDrag=Math.pow(liftCoef,2)*((1/Math.PI*aspectRatio)+0.005);
            return totalIndDrag; 
        }             
    }
    public double calculateMastDrag(){
        double totalMastDrag=1.13*((hullBeam*freeboard)+(((mainBAD+mainHeight)-(reefFun*mainHeight))*mastDiameter))/totalArea;//can later include reefing and flattering
        return totalMastDrag;
    }
    public double calculateTotalDrag(int i, double liftCoef){
        double visDragCoef=calculateVisDragCoef(i);
        double indDragCoef= calculateInducedDragCoef(i, liftCoef);
        double mastDragCoef=calculateMastDrag();
        double total=visDragCoef+mastDragCoef+indDragCoef;

        return total;
    }
    public double calculateBoatSpeed(double froudeNmbr, double lengthWL){
        double speed=Math.sqrt(9.81*lengthWL)*froudeNmbr/0.5144;
        return speed;
    }
    public double calculateApparentWindSpeed(double froudeNmbr, double lengthWL){
        double boatSpeed=calculateBoatSpeed(froudeNmbr,lengthWL);
        double angleRad=(180-trueWindAngle)*Math.PI/180;
        double subtotal=Math.pow(trueWindSpeed,2)+Math.pow(boatSpeed,2)-(2*boatSpeed*trueWindSpeed*Math.cos(angleRad));
        return Math.sqrt(subtotal);
    }
    public double calculateApparentAngle(double appWindSpeed){
        double angleRad=(180-trueWindAngle)*Math.PI/180;
        double subtotal=trueWindSpeed/appWindSpeed*Math.sin(angleRad);
        return Math.asin(subtotal);
    }
    public double[]  calculateEffectiveApparent(double appWindSpeed,double appWindAngle,double heelAngle){// HEEL IN radians
        double result[]=new double[2];
        double alongMotion=appWindSpeed*Math.cos( appWindAngle);
        double rightAngles=appWindSpeed*Math.sin( appWindAngle)*Math.cos(heelAngle);
        result[0]=Math.atan(rightAngles/alongMotion);
        double subtotal=Math.pow(alongMotion,2)+Math.pow(rightAngles,2);
        result[1]=Math.sqrt(subtotal)*0.5144;
        return result;
    }
    public double calculateCoefDrive(double liftCoef, double totalDragCoef, double effectiveApparentAngle){
        double coefDriveNew=liftCoef*Math.sin(effectiveApparentAngle)-totalDragCoef*Math.cos(effectiveApparentAngle);
        return coefDriveNew;
    }
    
    public double calculateCoefSideForce(double liftCoef, double totalDragCoef, double effectiveApparentAngle){
        double coefSideForceNew=liftCoef*Math.cos(effectiveApparentAngle)+totalDragCoef*Math.sin(effectiveApparentAngle);
        return coefSideForceNew;
    }
    
    public double calculateTotalDrive(double effectiveApparentSpeed, double coefDrive){
        double totalDriveNew=0.5*airDensity*totalArea*Math.pow(effectiveApparentSpeed,2)*coefDrive;
        if(reefFun>0){
            totalDriveNew=totalDriveNew*Math.pow(reefFun, 2);
        }
        return totalDriveNew;
    }
    
    public double calculateSideForce(double effectiveApparentSpeed, double coefSideForce){
        double sideForceNew=0.5*airDensity*totalArea*Math.pow(effectiveApparentSpeed,2)*coefSideForce;
        if(reefFun>0){
            sideForceNew=sideForceNew*Math.pow(reefFun, 2);
        }
        return sideForceNew;
    }
    public void calculateHeelingArm(double tc, double span){
        double ceMain=0.39*mainHeight*mainBAD;
        double ceJib=0.39*jibHeight;
        
        double distCE=((mainArea*ceMain)+(jibArea*ceJib))/totalArea;
        double distCLR=tc+(0.4*span);
        heelingArm=distCE+distCLR;
    }
    public double calculateHeelingMoment(double sideForce){
        return sideForce*heelingArm/1000;       
    }
    
    public double calculateLikelyHeelAngle(double heelingMoment, double slope, double intercept){
        return (heelingMoment-intercept)/slope;
    }
    
    public double[] calculateDriveAndSideForce(int coefIndex, double froudeNmbr, double lengthWL, double heelAngle, double slope, double intercept){//heel angle in radians
        double liftCoef=calculateLiftCoef(coefIndex);
        double totalDragCoef=calculateTotalDrag(coefIndex, liftCoef);
        
        double apparentWindSpeed=calculateApparentWindSpeed(froudeNmbr, lengthWL);
        double apparentWindAngle=calculateApparentAngle(apparentWindSpeed);
        double effectiveApp[]=calculateEffectiveApparent(apparentWindSpeed, apparentWindAngle, heelAngle);//effectiveApp[0]=angle; effectiveApp[1]=speed
     
        double coefDrive=calculateCoefDrive(liftCoef, totalDragCoef, effectiveApp[0]);
        double coefSideForce=calculateCoefSideForce(liftCoef, totalDragCoef, effectiveApp[0]);
    
        double totalDriveForce=calculateTotalDrive(effectiveApp[1],coefDrive);
        double totalSideForce=calculateSideForce(effectiveApp[1],coefSideForce);

        
        double heelingMoment=calculateHeelingMoment(totalSideForce);
        double likelyHeelAngle=calculateLikelyHeelAngle(heelingMoment, slope, intercept);
     
        double ssfToDriveRatio=totalSideForce/totalDriveForce;
        
        double result[]=new double [4];
        result[0]=totalDriveForce;
        result[1]=totalSideForce;
        result[2]=likelyHeelAngle;
        result[3]=ssfToDriveRatio;
        return result;//result[0]=total drive force; result[1]=total side force; result[2]=new heel angle; result[3]=side force to drive force ratio
    }

    public double getTrueWindSpeed() {
        return trueWindSpeed;
    }

    public double getTrueWindAngle() {
        return trueWindAngle;
    }

    
}
