/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.mycompany.vpp.core;

import static com.mycompany.vpp.core.ResiduaryCoef.EFFECTIVE_DRAFT_COEF;
import static com.mycompany.vpp.core.ResiduaryCoef.FROUDE_COEF;
import static com.mycompany.vpp.core.ResiduaryCoef.FROUDE_COEF_HEEL;
import static com.mycompany.vpp.core.ResiduaryCoef.FROUDE_COEF_KEEL;
import static com.mycompany.vpp.core.ResiduaryCoef.FROUDE_NUMBER;
import static com.mycompany.vpp.core.ResiduaryCoef.RN_BASE;
import static com.mycompany.vpp.core.ResiduaryCoef.SEA_WATER_DENSITY;
import static com.mycompany.vpp.core.ResiduaryCoef.SEA_WATER_VISCOSITY;
import static com.mycompany.vpp.core.ResiduaryCoef.SIDE_FORCE_COEF;
import static com.mycompany.vpp.core.ResiduaryCoef.WETTED_AREA_COEF;
import org.apache.commons.math3.stat.regression.SimpleRegression;

/**
 *
 * @author anton
 */
public class ResistanceCalculator {
    //
    //hull parameters
    //
    private double lengthWL;
    private double breadthWL;
    private double wsa[];
    private double cm;
    private double LCB;
    private double cp;
    private double displacement;
    private double lcf;
    private double awp;
    private double tc;
    //    
    //keel parameters
    //
    private double avgChord;
    private double keelTip;
    private double keelTop;
    private double span;
    private double tcKeel;
    //    
    //bulb parameters
    //
    private double chordBulb;
    private double tcBulb;
    private double wsaBulb;
    //    
    //rudder parameters
    //
    private double rudderPartNumber;
    private double rudderSpan;
    private double rudderChord;//not input
    private double rudderTc;
    private double rudderTip;
    private double rudderTop;
    private double rudderBladeDens;
    //
    //Righting data
    //
    private double mastWeight;//kg
    private double vcgMast;
    private double vcgHull;
    private double keelMass;
    private double bulbMass;
    private double bulbMaterialDens;
    private double hullDisplacement;
    //
    //Balance data
    //
    private double gm;
    private double slope;
    private double intercept;

    public ResistanceCalculator(double lengthWL, double breadthWL, double cm, double LCB, double cp, double displacement, double lcf, double awp, double tc, double keelTip, double keelTop, double span, double tcKeel, double chordBulb, double tcBulb, double wsaBulb, double mastWeight, double vcgMast,double vcgHull, double keelMass, double bulbMass, double bulbMaterialDens, double rudderPartNumber, double rudderSpan, double rudderTc, double rudderTip, double rudderTop, double rudderBladeDens) {
        this.lengthWL = lengthWL;
        this.breadthWL = breadthWL;
        this.cm = cm;
        this.LCB = LCB;
        this.cp = cp;
        this.displacement = displacement;
        this.lcf = lcf;
        this.awp = awp;
        this.tc = tc;
        this.keelTip = keelTip;
        this.keelTop = keelTop;
        this.avgChord=(keelTip+keelTop)/2;
        this.span = span;
        this.tcKeel = tcKeel;
        this.chordBulb = chordBulb;
        this.tcBulb = tcBulb;
        this.wsaBulb = wsaBulb;
        this.mastWeight=mastWeight;
        this.vcgMast=vcgMast;
        this.vcgHull=vcgHull;
        this.keelMass=keelMass;
        this.bulbMass=bulbMass;
        this.bulbMaterialDens=bulbMaterialDens;
        this.rudderPartNumber=rudderPartNumber;
        this.rudderSpan=rudderSpan;
        this.rudderTc=rudderTc;
        this.rudderTip=rudderTip;
        this.rudderTop=rudderTop;
        this.rudderBladeDens=rudderBladeDens;
        this.rudderChord=(rudderTip+rudderTop)/2;
        
        
        this.wsa=calculateWettedSurfaceArea();
        calculateGM();
    }
    //
    //SPEED AND REYNOLDS NUMBER FUNCTIONS
    //
    public double calculateSpeed(int fraudInd){
        double speed=FROUDE_NUMBER[fraudInd]*Math.pow((lengthWL*9.81), 0.5);
        return speed;
    }
    
    public double calculateRnNumberHULL(int fraudInd){
        double boatSpeed=calculateSpeed(fraudInd);
        double rnNumber=(SEA_WATER_DENSITY*lengthWL*RN_BASE*boatSpeed)/SEA_WATER_VISCOSITY;
        return rnNumber;
    }
    
    public double calculateRnNumberKEEL(int fraudInd){
        double boatSpeed=calculateSpeed(fraudInd);
        double rnNumber=(SEA_WATER_DENSITY*avgChord*boatSpeed)/SEA_WATER_VISCOSITY;
        return rnNumber;
    }
    public double calculateRnNumberBULB(int fraudInd){
        double boatSpeed=calculateSpeed(fraudInd);
        double rnNumber=(SEA_WATER_DENSITY*chordBulb*boatSpeed)/SEA_WATER_VISCOSITY;
        return rnNumber;
    }
    public double calculateRnNumberRUDDER(int fraudInd){
        double boatSpeed=calculateSpeed(fraudInd);
        double rnNumber=(SEA_WATER_DENSITY*rudderChord*boatSpeed)/SEA_WATER_VISCOSITY;
         return rnNumber;
    }
    //
    //WETTED SURFACE AREA
    //
    public double[] calculateWettedSurfaceArea(){
        double breadthToTc=breadthWL/tc;
        double wsa[]=new double[4];
        double a1=1.97+0.171*breadthToTc;
        double a2=Math.pow((0.65/cm),0.333);
        double a3=displacement/1.025*lengthWL;
        a3=Math.pow(a3, 0.5);
        wsa[0]=a1*a2*a3;
        for (int i=0; i<3;i++){
            double subtotal=WETTED_AREA_COEF[i][0]+WETTED_AREA_COEF[i][1]*breadthToTc+WETTED_AREA_COEF[i][2]*Math.pow(breadthToTc, 2)+WETTED_AREA_COEF[i][3]*cm;
            wsa[i+1]=wsa[0]*(1+0.01*subtotal);
        }
        return wsa;
    }
    //
    //UPRIGHT HULL RESISTANCES
    //
    public double[] calculateResiduaryResHULL(){
        double volume=displacement/(SEA_WATER_DENSITY/1000);
        double b0=(Math.pow(volume,(0.333))/lengthWL);
        double b1=LCB/lengthWL;
        double b3=(Math.pow(volume,(0.666))/awp);
        double b4=breadthWL/lengthWL;
        double b5=Math.pow(volume,(0.666))/wsa[0];
        double b6=LCB/lcf;
        double b7=Math.pow(b1,2);
        double b8=Math.pow(cp,2);
        
        double resResiduary[]=new double[11];
        for(int i=0;i<FROUDE_NUMBER.length;i++){
            double c1=FROUDE_COEF[i][1]*b1;
            double c2=FROUDE_COEF[i][2]*cp;
            double c3=FROUDE_COEF[i][3]*b3;
            double c4=FROUDE_COEF[i][4]*b4;
            double c5=FROUDE_COEF[i][5]*b5;
            double c6=FROUDE_COEF[i][6]*b6;
            double c7=FROUDE_COEF[i][7]*b7;
            double c8=FROUDE_COEF[i][8]*b8;

            double sum=c1+c2+c3+c4+c5+c6+c7+c8;
            double resCoef=FROUDE_COEF[i][0]+(sum)*b0;
            resResiduary[i]=displacement*9.81*SEA_WATER_DENSITY*resCoef;
            
        }
        
        return  resResiduary;
    }
    
    public double[][] calculateFictionalResHULL(){
        double resFictional[][]=new double[4][11];
        for(int j=0;j<4;j++){
            for(int i=0;i<FROUDE_NUMBER.length;i++){
                double boatspeed=calculateSpeed(i);
                double rnNumber=calculateRnNumberHULL(i);
                double forPow=Math.log10(rnNumber)-2;
                double coefFictional=0.075/Math.pow(forPow, 2);
                resFictional[j][i]=0.5*SEA_WATER_DENSITY*wsa[j]*Math.pow(boatspeed, 2)*coefFictional;
            }
        }
        return resFictional;
    }
    //
    //UPRIGHT KEEL RESISTANCES
    //
    public double[] calculateResiduaryResKEEL(){
        double keelVCG=(((keelTip*keelTip*tcKeel)*span)/((keelTip*keelTip*tcKeel)+(keelTop*keelTop*tcKeel))+tc);
        double keelVol=0.5*0.66*((keelTop*keelTop*tcKeel)+(keelTip*keelTip*tcKeel))*span;
        double resResiduary[]=new double[11];
        resResiduary[0]=0;
        resResiduary[1]=0;
        for(int i=2;i<FROUDE_NUMBER.length;i++){
            int j=i-2;
            double b1=FROUDE_COEF_KEEL[j][1]*(tc+span)/breadthWL;
            double b2=FROUDE_COEF_KEEL[j][2]*(tc+keelVCG)/Math.pow(keelVol, 0.333);
            double b3=FROUDE_COEF_KEEL[j][3]*((hullDisplacement*1000/SEA_WATER_DENSITY)/keelVol);
            double coefResiduary=FROUDE_COEF_KEEL[j][0]+b1+b2+b3;
            resResiduary[i]=keelVol*SEA_WATER_DENSITY*9.81*coefResiduary;
        }
        return  resResiduary;
    }
    
    public double[] calculateViscousResKEEL(){
        double resViscousResKEEL[]=new double[11];
        double wsaKEEL=span*avgChord*2;
        double onePlusK=1+(2*tcKeel)+(60*Math.pow(tcKeel, 4));
        for(int i=0;i<FROUDE_NUMBER.length;i++){
            
            double boatspeed=calculateSpeed(i);
            double rnNumber=calculateRnNumberKEEL(i);
            double forPow=Math.log10(rnNumber)-2;
            double coefViscous=0.075/Math.pow(forPow, 2);
            resViscousResKEEL[i]=coefViscous*onePlusK*0.5*SEA_WATER_DENSITY*wsaKEEL*Math.pow(boatspeed,2);
        }
        return resViscousResKEEL;
    }
    //
    //UPRIGHT BULB RESISTANCES
    //
    public double[] calculateViscousResBULB(){
        double resViscousResBULB[]=new double[11];
        double onePlusK=1+(1.5*tcBulb);
        for(int i=0;i<FROUDE_NUMBER.length;i++){
            double boatspeed=calculateSpeed(i);
            double rnNumber=calculateRnNumberBULB(i);
            
            double forPow=Math.log10(rnNumber)-2;
            double coefViscous=0.075/Math.pow(forPow, 2);
            
            resViscousResBULB[i]=coefViscous*onePlusK*0.5*SEA_WATER_DENSITY*wsaBulb*Math.pow(boatspeed,2);
        }
        return resViscousResBULB;
    }
    //
    //UPRIGHT RUDDER RESISTANCES
    //
    public double[] calculateViscousResRUDDER(){
        double resViscousResRUDDER[]=new double[11];
        double wsaRudder=2*rudderSpan*rudderChord*rudderPartNumber;
        double onePlusK=1+(2*rudderTc)+(60*Math.pow(rudderTc,4));
        for(int i=0;i<FROUDE_NUMBER.length;i++){
            double boatspeed=calculateSpeed(i);
            double rnNumber=calculateRnNumberRUDDER(i);
            double forPow=Math.log10(rnNumber)-2;
            double coefViscous=0.075/Math.pow(forPow, 2);
            resViscousResRUDDER[i]=coefViscous*onePlusK*0.5*SEA_WATER_DENSITY*wsaRudder*Math.pow(boatspeed,2);
            
        }
        return resViscousResRUDDER;
    }
    //
    //HEELED HULL RESISTANCE
    //
    //CHANGE FOR 20 DEGREES HEEL ANGLE
    public double[] calculateResChange20(){
        double breadthToTc=(breadthWL/tc);
        double lengthToBreadth=(lengthWL/breadthWL);
        double bttSQ=Math.pow(breadthToTc, 2);
        double lcbSQ=Math.pow(LCB, 2);
        double resResiduaryChange[]=new double[7];
        for(int i=0;i<FROUDE_NUMBER.length-4;i++){
            double u1=FROUDE_COEF_HEEL[i][1]*lengthToBreadth;
            double u2=FROUDE_COEF_HEEL[i][2]*breadthToTc;
            double u3=FROUDE_COEF_HEEL[i][3]*bttSQ;
            double u4=FROUDE_COEF_HEEL[i][4]*LCB;
            double u5=FROUDE_COEF_HEEL[i][5]*lcbSQ;
            
            double coef=FROUDE_COEF_HEEL[i][0]+u1+u2+u3+u4+u5;
            resResiduaryChange[i]=displacement*9.81*SEA_WATER_DENSITY/1000*coef;
          
        }
        return resResiduaryChange;
    }
    //CHANGE FOR ANY DEGREES HEEL ANGLE
    public double[] calculateResChangeAny(double heel, double[]change20){
        double resResiduaryChange[]=new double[7];
        double heelRad=heel*(Math.PI/180);
        double formulaCoef=6*Math.pow(heelRad, 1.7);
        
        for(int i=0; i<change20.length;i++){
            resResiduaryChange[i]=formulaCoef*change20[i];
           
        }
        return resResiduaryChange;
    }
    
    public double[][] calculateResChange(){
        double resChangeFinal[][]=new double[3][7];
        resChangeFinal[1]=calculateResChange20();
        resChangeFinal[0]=calculateResChangeAny(10,resChangeFinal[1]);
        resChangeFinal[2]=calculateResChangeAny(30,resChangeFinal[1]);
        return resChangeFinal;
    }
    //
    //INDUCED DRAG
    //
    public double[][] calculateEffectiveDraft(){
        double effectiveDraft[][]=new double[4][11];
        double totalDraft=tc+span;
        double tcToTotal=tc/totalDraft;
        for(int i=0;i<4;i++){
            double a1=EFFECTIVE_DRAFT_COEF[i][0]*tcToTotal;
            double a2=EFFECTIVE_DRAFT_COEF[i][1]*Math.pow(tcToTotal, 2);
            double a3=EFFECTIVE_DRAFT_COEF[i][2]*(breadthWL/tc);
            double a4=EFFECTIVE_DRAFT_COEF[i][3]*(keelTip/keelTop);
            double subTotal=a1+a2+a3+a4;
            for (int j=0;j<FROUDE_NUMBER.length;j++){
                double multiplier=EFFECTIVE_DRAFT_COEF[i][4]+EFFECTIVE_DRAFT_COEF[i][5]*FROUDE_NUMBER[j];
                effectiveDraft[i][j]=subTotal*multiplier*totalDraft;
            }
        }
        return effectiveDraft;
    }
    
    public double[][] calculateSideForce(double leewayAngle){
        double sideforce[][]=new double[4][11];
        double heelRad[]=new double[4];
        heelRad[0]=0;
        heelRad[1]=10*(Math.PI/180);
        heelRad[2]=20*(Math.PI/180);
        heelRad[3]=30*(Math.PI/180);
        double leewayRad=leewayAngle*(Math.PI/180);
        for(int i=0; i<4; i++){
            double draftToWSA=Math.pow(tc+span,2)/wsa[i];
            double tcToTotal=tc/(tc+span);
            double b1=SIDE_FORCE_COEF[i][0]*draftToWSA;
            double b2=SIDE_FORCE_COEF[i][1]*Math.pow(draftToWSA,2);
            double b3=SIDE_FORCE_COEF[i][2]*tcToTotal;
            double b4=SIDE_FORCE_COEF[i][3]*tcToTotal*draftToWSA;
            double coef=b1+b2+b3+b4;
            for (int j=0;j<FROUDE_NUMBER.length;j++){
                double q=0.5*SEA_WATER_DENSITY*Math.pow(calculateSpeed(j), 2);
                sideforce[i][j]=wsa[i]*q*coef*leewayRad/Math.cos(heelRad[i]);
                
            }
            
        }
        return sideforce;
    }
    
    public double [][] calculateInducedDrag(double leewayAngle){
        double indDrag[][]=new double[4][11];
        double sideforce[][]=calculateSideForce(leewayAngle);
        double effectiveDraft[][]=calculateEffectiveDraft();
        for(int i=0;i<4;i++){
            for(int j=0;j<FROUDE_NUMBER.length;j++){
                double q=0.5*SEA_WATER_DENSITY*Math.pow(calculateSpeed(j), 2);
                indDrag[i][j]=Math.pow(sideforce[i][j],2)/(Math.PI*q*Math.pow(effectiveDraft[i][j], 2));
            }
        }
        return indDrag;
    }
    //
    //TOTAL RESISTANCE
    //
    public double[][] calculateTotalResistance(double leewayAngle){
        double[] resResistanceUpright=calculateResiduaryResHULL();
        double[][]ficResistance= calculateFictionalResHULL();
        double[] keelResResistance=calculateResiduaryResKEEL();
        double[] keelVisResistance=calculateViscousResKEEL();
        double[] bulbVisResistane=calculateViscousResBULB();
        double[] rudderVisResistane=calculateViscousResRUDDER();
        double[][] changeResResistance=calculateResChange();
        double [][] inducedDrag=calculateInducedDrag(leewayAngle);
        double [][] subtotalResistance=new double[4][11];
        double [][] totalResistance=new double[4][7];
        for(int i=0;i<4;i++){
            for(int j=0;j<11;j++){
                subtotalResistance[i][j]=resResistanceUpright[j]+ficResistance[i][j]+keelResResistance[j]+rudderVisResistane[j]+keelVisResistance[j]+bulbVisResistane[j]+inducedDrag[i][j];
            }
        }
        
        for(int i=0;i<4;i++){
            if(i==0){
                for(int j=0;j<7;j++){
                    totalResistance[i][j]=subtotalResistance[i][j+3];
                }
            }else{
                for(int j=0;j<7;j++){
                    totalResistance[i][j]=subtotalResistance[i][j+3]+changeResResistance[i-1][j];
                   
                }
            }

            
        }
        return totalResistance;//replace subtotalResistance with totalResistance if you want to include resudiary change with heel angle
    }
    public void calculateGM(){
        double hullWeight=(displacement*1000)*(1-0.35)-mastWeight;
        double vcgKeel=(((keelTip*keelTip*tcKeel)*span)/((keelTip*keelTip*tcKeel)+(keelTop*keelTop*tcKeel))+tc)*-1;
        double vcgBulb=-(tc+span);
        double vcgRudder=-((rudderTip*rudderTip*rudderTc*rudderSpan)/((rudderTip*rudderTip*rudderTc)+(rudderTop*rudderTop*rudderTc)));
        double keelVolume=0.5*0.66*((keelTip*keelTip*tcKeel)+(keelTop*keelTop*tcKeel))*span;
        double bulbVolume=bulbMass/bulbMaterialDens;
        double rudderVolume=rudderPartNumber*0.5*((rudderTop*rudderTop*rudderTc)+(rudderTip*rudderTip*rudderTc))*rudderSpan;
        double rudderMass=rudderBladeDens*rudderVolume;     
        double hullVolumeDismplacement=(displacement/1.025)-keelVolume-bulbVolume-rudderVolume;
        hullDisplacement=hullVolumeDismplacement;
        
        double lToVol=lengthWL/Math.pow( hullVolumeDismplacement, 0.333);
        double waterPlaneCooeficient=((1.313*cp)+(0.0371*lToVol)-(0.0857*cp*lToVol));
        double transArea=((lengthWL*Math.pow(breadthWL, 3))/12)*Math.pow(waterPlaneCooeficient, 2);
        
        double kg=((mastWeight*vcgMast)+(hullWeight*vcgHull)+(keelMass*vcgKeel)+(bulbMass*vcgBulb)+(rudderMass*vcgRudder))/(displacement*1000);
        double kb=-0.35*tc;
        double bm=transArea/ hullVolumeDismplacement;

        gm=kb+bm-kg;
    }
    
    public double calculateRightingMoment(double heelAngle){
        double heelRad=Math.PI*heelAngle/180;
        double gz=gm*Math.sin(heelRad)*Math.cos(heelRad);
        double rightingMoment=gz*9.81*displacement;
        return rightingMoment;
    }
    
    public void calculateSlopeAndIntercept(){
        SimpleRegression sr=new SimpleRegression(true);
        for (int i=0;i<13;i++){
            double x=i*2;
            double y=calculateRightingMoment(x);
            sr.addData(x,y);
            
        }
        intercept=sr.getIntercept();
        slope=sr.getSlope();
    }

    public double getTc() {
        return tc;
    }

    public double getSpan() {
        return span;
    }

    public double getLengthWL() {
        return lengthWL;
    }
    
    public double getSlope() {
        return slope;
    }

    public double getIntercept() {
        return intercept;
    }
    
}
