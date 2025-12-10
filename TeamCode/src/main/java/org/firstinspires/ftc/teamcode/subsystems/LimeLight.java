package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Autonomous
public class LimeLight extends SubsystemBase {
        private Limelight3A limelight;
        private double x,y;
        public double huh;

        //ty = llResult.getTy()     tx = llResult.getTx()
        public double HeadingPerpend(double tx){
            double theta = follower.getPose().getHeading();
            return (theta+Math.toRadians(tx));
        }
        public List<double[]> RelativeCoords() {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                return new ArrayList<>();   // return empty list if no detections
            }

            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            List<double[]> points = new ArrayList<>();

            for (int i = 0; i < detections.size(); i++) {
                LLResultTypes.DetectorResult det = detections.get(i);

                double tx = det.getTargetXDegrees();
                double ty = det.getTargetYDegrees();

                double[] xy = coordonate(tx, ty);
                points.add(xy);
            }

            return points;
        }
        public double[] coordonate(double tx, double ty) {

            double alpha= 30;//unghiu dintre camera si sol
            double c=1;
            double h=10;

            double dy = h / Math.tan(Math.toRadians( ty + alpha)) * c;//c e constanta pt cm->inchi daca folosim cm pentru h
            double dx = -Math.tan(Math.toRadians(tx)) * dy;
            double theta = follower.getPose().getHeading();
            double x = follower.getPose().getX() + dy * Math.cos(theta) - dx * Math.sin(theta);
            double y = follower.getPose().getY() + dy * Math.sin(theta) + dx * Math.cos(theta);
            return new double[] {x,y, theta+tx};
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
        }//calculeaza distanta cea mai mica de la un punct pe harta la obiect

        public double[] cord(List<double[]> coord){
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
            int i;
            double c=0,DistMin=1000000000;
            int ind=0,count = result.length;
            for (i = 0; i <count; i++)
                {
                    if (result[i] < DistMin)
                    {
                        ind = i;
                    }
                }
        return (coord.get(ind));
        }//calculeaza coord celui mai apropiat obj fata de un set de coord
    public Pose close_obj (List<double[]> coord) {
        double[] NuMaiPoooooottt = cord(RelativeCoords());
        return new Pose(NuMaiPoooooottt[0],NuMaiPoooooottt[1],NuMaiPoooooottt[2]);
    }//pozitia celui mai apropiat dintre obiecte








        /*


            Pose Obiect = new Pose(x,y,theta+Math.toRadians(tx));
        Pose Start = follower.getPose();
        PathChain chain = follower.pathBuilder().addPath(new BezierLine(Start,Obiect)).build();

         */
}

