package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.List;

@Autonomous
public class LimeLight {
    private Limelight3A limelight3A;
    public double dx;
    public double dy;


    public double x,y;
    public double heading;


    //ty = llResult.getTy()     tx = llResult.getTx()
    public double HeadingPerpend(double tx){
        double theta = follower.getPose().getHeading();
        return (theta+Math.toRadians(tx));
    }
    public double[] coordonate(float tx, float ty) {
        double alpha= 30;
        double c=1;
        double h=10;

        double dy = h / java.lang.Math.tan(Math.toRadians( ty + alpha)) * c;//c e constanta pt cm->inchi daca folosim cm pentru h
        double dx = -java.lang.Math.tan(Math.toRadians(tx)) * dy;
        double theta = follower.getPose().getHeading();
        double x = follower.getPose().getX() + dy * Math.cos(theta) - dx * Math.sin(theta);
        double y = follower.getPose().getY() + dy * Math.sin(theta) + dx * Math.cos(theta);
        return new double[] {x,y};
    }//
        public double distanta (double x,double y){
        double cx=23;
        double cy=22;
        double d=Math.sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
        return d;
        }//calculeaza distanta de al un punct pe harta la obiect
/*
        public int indice(List<double[]> dist){
        double i,c=0,DistMax=;
        int count = dist.size();
        for (i = 0; i <count; i++)
            {
                if (DistMax<)
                {

                }
            }
        }//am nevoie de traiectorie
    //e ora 2;30 si eu deja nu mai pot...






        /*


            Pose Obiect = new Pose(x,y,theta+Math.toRadians(tx));
        Pose Start = follower.getPose();
        PathChain chain = follower.pathBuilder().addPath(new BezierLine(Start,Obiect)).build();

         */
}

