package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;

@Autonomous
public class LimeLight extends SubsystemBase {
        private Limelight3A limelight;
        private double x,y;
        LLResult result = getResult();
        public LimeLight(HardwareMap hardwareMap, Telemetry telemetry) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);   // pipeline default
            limelight.setPollRateHz(100);  // update rapid
            limelight.start();             // pornește camera
        }
        private LLResult getResult() {
            LLResult r = limelight.getLatestResult();
            return (r != null && r.isValid()) ? r : null;
        }
        public void periodic() {
            result.getPipelineIndex();
        }

        //ty = llResult.getTy()     tx = llResult.getTx()
        public double HeadingPerpend(double tx){
            double theta = follower.getPose().getHeading();
            return (theta+Math.toRadians(tx));
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
            return new double[] {x,y, theta+ Math.toRadians(tx)};
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
        }//returneaza toate coordonatelede la obiecte
        public List<double[]> RelativeCoords2(int y) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                return new ArrayList<>();   // return empty list if no detections
            }

            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            List<double[]> points = new ArrayList<>();

            for (int i = 0; i < detections.size(); i++) {
                LLResultTypes.DetectorResult det = detections.get(i);
                double tx, ty;
                double[] xy;
                switch (y) {
                    case 1:
                        if (det.getClassName() == "green"){
                        tx = det.getTargetXDegrees();
                        ty = det.getTargetYDegrees();
                        xy = coordonate(tx, ty);
                        points.add(xy);}

                    case 2:
                        if (det.getClassName() == "purple"){
                            tx = det.getTargetXDegrees();
                            ty = det.getTargetYDegrees();
                            xy = coordonate(tx, ty);
                            points.add(xy);}
                }
            }

            return points;
        }//returneaza coord obj cul. verd
        public List<double[]> pozitie_culori(int i){
            switch(i) {
                case 1:
                    limelight.pipelineSwitch(1);
                case 2:
                    limelight.pipelineSwitch(2);
                case 3:
                    limelight.pipelineSwitch(3);
            }
        return RelativeCoords();
        }//returneaza toate coordonatelede la obiecte de culoare pipeline specifica
        public double[] distante (List <double[]>coord,Pose poz){
            double cx=poz.getX();
            double cy=poz.getY();

            double[] result =new double[coord.size()];
            int sz = coord.size();

            for (int i=0;sz==i;i++) {
                x=coord.get(i)[0];
                y=coord.get(i)[1];
                double d=Math.sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
                result[i]=d;
            }

            return result;
        }//calculeaza distantele
        private List<Pose> CoordToPose(List<double[]>coord){
            List<Pose> coor = new ArrayList<>();
            for(int i=0;i<coord.size();i++)
            {
                double[] matematica = coord.get(i);
                coor.add(new Pose(matematica[0],matematica[1],Math.toRadians(matematica[0])+follower.getHeading()));
            }
            return coor;
        }//transforma din lista de coord in lista de vectori de pozitie
        public double[] devmin(List<double[]> coord){
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
        }//calculeaza coord celui mai apropiat obj fata de un set de coord    A,si utilizeazo cand ma duc undeva si am nevoie de o anumita minge cu o anumita culoare(tre sa setez si pipelineindexu or smth)
        /*metoda ajutatoare ca am nev*/
        private int pozitie_la_minim(double[] numar){
            OptionalDouble min = Arrays.stream(numar).min();
            int c=0;
            int[] u = new int[numar.length];
            for (int i=0;i<numar.length;i++)
            {
                if (min.getAsDouble()==numar[i])
                {
                u[c]=i;
                c++;
                }
            }
            return u[0];
        }
        public Pose coordClose(Pose pozitie){
            List<double[]> coord = RelativeCoords();
            List<Pose> cord = CoordToPose(coord);
            double[] mort = new double[coord.size()];
            for (int i=0;i<cord.size();i++)
            {   Pose Poz = cord.get(i);
                double d = Poz.distanceFrom(follower.getPose()) + Poz.distanceFrom(pozitie);
                mort [i]=d;
            }
            return cord.get(pozitie_la_minim(mort));
        } //da poze-ul la care tre sa se duca robotul in auto, daca nu conteaza culoarea
        public Pose close_obj (int i) {
            double[] NuMaiPoooooottt = new double[0];
            if (i == 0)
                NuMaiPoooooottt = devmin(RelativeCoords());
            else
                NuMaiPoooooottt = devmin(RelativeCoords2(i));
            //pozitia celui mai apropiat dintre obiecte cu 0 avem toate obiectele cu 1 avem green si pe 2 avem purple
            return new Pose(NuMaiPoooooottt[0], NuMaiPoooooottt[1], NuMaiPoooooottt[2]);
        /*


            Pose Obiect = new Pose(x,y,theta+Math.toRadians(tx));
        Pose Start = follower.getPose();
        PathChain chain = follower.pathBuilder().addPath(new BezierLine(Start,Obiect)).build();

         */
        }
}