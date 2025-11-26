package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.List;

@Autonomous
public class LimeLight {
    private Limelight3A limelight3A;
        private double x,y;


    //ty = llResult.getTy()     tx = llResult.getTx()
        public double HeadingPerpend(double tx){
            double theta = follower.getPose().getHeading();
            return (theta+Math.toRadians(tx));
        }
        public double[] coordonate(float tx, float ty) {

            double alpha= 30;
            double c=1;
            double h=10;

            double dy = h / Math.tan(Math.toRadians( ty + alpha)) * c;//c e constanta pt cm->inchi daca folosim cm pentru h
            double dx = -Math.tan(Math.toRadians(tx)) * dy;
            double theta = follower.getPose().getHeading();
            double x = follower.getPose().getX() + dy * Math.cos(theta) - dx * Math.sin(theta);
            double y = follower.getPose().getY() + dy * Math.sin(theta) + dx * Math.cos(theta);
            return new double[] {x,y};
        }//
        public double[] distante (List <double[]>coord){
            double[] result =new double[coord.size()];
            double cx=23;
            double cy=22;//set de coordonate punct de unde avem voie sa aruncam
            int sz = coord.size();

            for (int i=0;sz==i;i++) {
                x=coord.get(i)[0];
                y=coord.get(i)[1];
                double d=Math.sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
                result[i]=d;
            }

            return result;
        }//calculeaza distanta de al un punct pe harta la obiect

        public double[] coord(List<double[]> coord){
            double[] result =new double[coord.size()];
            double cx=23;
            double cy=22;//set de coordonate punct de unde avem voie sa aruncam
            int sz = coord.size();

            for (int i=0;sz==i;i++) {
                x=coord.get(i)[0];
                y=coord.get(i)[1];
                double d=Math.sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
                result[i]=d;
            }
            double c=0,ind=0,DistMin=1000000000;
            int i;
            int count = result.length;
            for (i = 0; i <count; i++)
                {
                    if (result[i] < DistMin)
                    {
                        ind = i;
                    }
                }
        return (coord.get(i));
        }//calculeaza coord celui mai apropiat obj fata de un set de coord

        //am nevoie de traiectorie
    //e ora 2;30 si eu deja nu mai pot...






        /*


            Pose Obiect = new Pose(x,y,theta+Math.toRadians(tx));
        Pose Start = follower.getPose();
        PathChain chain = follower.pathBuilder().addPath(new BezierLine(Start,Obiect)).build();

         */
}

