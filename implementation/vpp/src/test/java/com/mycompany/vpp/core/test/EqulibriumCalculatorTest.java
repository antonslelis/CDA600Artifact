/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.mycompany.vpp.core.test;

import com.mycompany.vpp.core.AeroCalculator;
import com.mycompany.vpp.core.ResistanceCalculator;
import org.junit.Test;

/**
 *
 * @author anton
 */
public class EqulibriumCalculatorTest {
    @Test
    public void testResistanceResults() {
        int heelIndex=0;//0-upright; 1- 10 degrees; 2- 20 degrees; 3 - 30 degrees
        int leewayAngle=4;
        ResistanceCalculator rc=new ResistanceCalculator(14.75, 3.91, 0.7, 7.82, 0.56, 17.5, 8.11, 38.69, 0.689, 2.0006, 2.229, 2.254,0.11, 2.78, 0.265, 4.32, 573.201, 13.446, 0.55, 750, 5375, 11300, 1, 2.0025, 0.11, 1.053959, 1.171065, 2400);
        //wetted surface area
        double wsa[]=rc.calculateWettedSurfaceArea();
        System.out.println("Wetted surface area: ");
        for(int i=0;i<wsa.length;i++){
            System.out.println(wsa[i]);
        }
        //residuary resistance hull
        double rr[]=rc.calculateResiduaryResHULL();
        System.out.println("Residuary resistance hull: ");
        for(int i=0;i<rr.length;i++){
            System.out.println(rr[i]);
        }
        //frictional resistance
        double fr[][]=rc.calculateFictionalResHULL();
        System.out.println("Frictional resistance hull: ");
        for(int i=0;i<fr[heelIndex].length;i++){
            System.out.println(fr[heelIndex][i]);
        }
        //residuary resistance keel
        double rrk[]=rc.calculateResiduaryResKEEL();
        System.out.println("Residuary resistance keel: ");
        for(int i=0;i<rrk.length;i++){
            System.out.println(rrk[i]);
        }
        //viscous resistance keel
        double vrk[]=rc.calculateViscousResKEEL();
        System.out.println("Viscous resistance keel: ");
        for(int i=0;i<vrk.length;i++){
            System.out.println(vrk[i]);
        }
        //viscous resistance bulb
        double vrb[]=rc.calculateViscousResBULB();
        System.out.println("Viscous resistance bulb: ");
        for(int i=0;i<vrb.length;i++){
            System.out.println(vrb[i]);
        }
        //viscous resistance bulb
        double vrr[]=rc.calculateViscousResRUDDER();
        System.out.println("Viscous resistance rudder: ");
        for(int i=0;i<vrr.length;i++){
            System.out.println(vrr[i]);
        }
        //induced drag
        double id[][]=rc.calculateInducedDrag(leewayAngle);
        System.out.println("Induced drag: ");
        for(int i=0;i<id[heelIndex].length;i++){
            System.out.println(id[heelIndex][i]);
        }
        //induced drag
        double sf[][]=rc.calculateSideForce(leewayAngle);
        System.out.println("Side Force: ");
        for(int i=0;i<sf[heelIndex].length;i++){
            System.out.println(sf[heelIndex][i]);
        }
        //change is residuary resistance for 20 degrees
        double rrchange[]=rc.calculateResChange20();
        System.out.println("Residuary resistance change for 20 heel: ");
        for(int i=0;i<rrchange.length;i++){
            System.out.println(rrchange[i]);
        }
        double tr[][]=rc.calculateTotalResistance(leewayAngle);
        System.out.println("Total resistance: ");
        for(int i=0;i<tr[heelIndex].length;i++){
            System.out.println(tr[heelIndex][i]);
        }
    }
    @Test
    public void testAeroResults() {
        AeroCalculator ac=new AeroCalculator(21.38, 6.66, 2, 23.80, 6.06, 6.66, 16, 45, 1.225, 1.2, 4.6, 0, 0.6, 1);
        ac.calculateHeelingArm(0.689, 2.253671);
        double heelAngle=20*Math.PI/180;
        double speed=0.34;
        double lwl=14.75;
        int apparentIndex=0;//0- 27 degrees; 1- 50 degrees; 2- 80 degrees; 3- 100 degrees apparent angle
        //lift coef
        double lift=ac.calculateLiftCoef(apparentIndex);
        System.out.println("Lift coeficient: "+lift);
        //viscous drag coef
        double visCoef=ac.calculateVisDragCoef(apparentIndex);
        System.out.println("Viscous drag coeficient: "+visCoef);
        //viscous drag coef
        double indCoef=ac.calculateInducedDragCoef(apparentIndex, lift);
        System.out.println("Induced drag coeficient: "+indCoef);
        //mast drag coef
        double mastCoef=ac.calculateMastDrag();
        System.out.println("Mast drag coeficient: "+mastCoef);
        //total drag coef
        double totalDragCoef=ac.calculateTotalDrag(apparentIndex, lift);
        System.out.println("Total drag coeficient: "+totalDragCoef);
         //apparent wind speed
        double apparentWindSpeed=ac.calculateApparentWindSpeed(speed, lwl);
        System.out.println("Apparent wind speed: "+apparentWindSpeed);
         //apparent wind angle
        double apparentWindAngle=ac.calculateApparentAngle(apparentWindSpeed);
        System.out.println("Apparent wind angle: "+apparentWindAngle);
        //effective apparent wind angle and speed
        double effApparentWA[]=ac.calculateEffectiveApparent(apparentWindSpeed, apparentWindAngle, heelAngle);
        System.out.println("Effective apparent wind angle: "+effApparentWA[0]+ " Effective apparent wind speed: "+effApparentWA[1]);
        //drive coef
        double driveC=ac.calculateCoefDrive(lift, totalDragCoef, effApparentWA[0]);
        System.out.println("Drive coeficient: "+driveC);
        //side force coef
        double sideC=ac.calculateCoefSideForce(lift, totalDragCoef, effApparentWA[0]);
        System.out.println("Side force coeficient: "+sideC);
        //drive force
        double drive=ac.calculateTotalDrive(effApparentWA[1], driveC);
        System.out.println("Drive force: "+drive);
        //side force
        double side=ac.calculateSideForce(effApparentWA[1], sideC);
        System.out.println("Side force: "+side);
        //heeling moment
        double heelMoment=ac.calculateHeelingMoment(side);
        System.out.println("Heeling Moment: "+heelMoment);
        
    }
    
}
