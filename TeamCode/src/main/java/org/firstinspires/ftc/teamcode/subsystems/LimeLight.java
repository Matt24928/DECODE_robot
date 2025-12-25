package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.OptionalInt;


public class LimeLight extends SubsystemBase {
    private Limelight3A limelight;
    LLResult result;
    private int pipeline = 0;
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
    @Override
    public void periodic() {
        result = getResult();
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

        if (result == null || !result.isValid()) {
            return new ArrayList<>();   // return empty list if no detections
        }

        List<LLResultTypes.DetectorResult> detections = this.result.getDetectorResults();
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
        if (result == null || !result.isValid()) {
            return new ArrayList<>();   // return empty list if no detections
        }
        List<LLResultTypes.DetectorResult> detections = this.result.getDetectorResults();
        List<double[]> points = new ArrayList<>();
        for (int i = 0; i < detections.size(); i++) {
            LLResultTypes.DetectorResult det = detections.get(i);
            double tx, ty;
            double[] xy;
            switch (y) {
                case 1:
                    if (Objects.equals(det.getClassName(), "green")){
                        tx = det.getTargetXDegrees();
                        ty = det.getTargetYDegrees();
                        xy = coordonate(tx, ty);
                        points.add(xy);}
                    break;
                case 2:
                    if (Objects.equals(det.getClassName(), "purple")){
                        tx = det.getTargetXDegrees();
                        ty = det.getTargetYDegrees();
                        xy = coordonate(tx, ty);
                        points.add(xy);}
                    break;
            }
        }
        return points;
    }//returneaza coord obj cul. verd
    public void changepipeline(int i){
        if(i == pipeline) return;
        pipeline=i;
        limelight.pipelineSwitch(i);
    }//schimba pipeline

    private List<Pose> CoordToPose(List<double[]>coord){
        List<Pose> coor = new ArrayList<>();
        for(int i=0;i<coord.size();i++)
        {
            double[] matematica = coord.get(i);
            coor.add(new Pose(matematica[0],matematica[1],matematica[2]));
        }
        return coor;
    }//transforma din lista de coord in lista de vectori de pozitie
    public double[] devmin(List<double[]> coord){
        if (coord == null || coord.isEmpty()) return null;
        double[] dist = new double[coord.size()];
        double cx = 23, cy = 22;

        for (int i = 0; i < coord.size(); i++) {
            double x = coord.get(i)[0];
            double y = coord.get(i)[1];

            if (x > 80) dist[i] = Math.hypot(x - cx, y - cy);
            else dist[i] = Double.POSITIVE_INFINITY; // tx gpt we love you
        }

        double val = Double.POSITIVE_INFINITY;
        int cont = -1;

        for (int i = 0; i < dist.length; i++) {
            if (dist[i] < val) {
                val = dist[i];
                cont = i;
            }
        }
        return (cont >= 0 && val != Double.POSITIVE_INFINITY) ? coord.get(cont) : null;
    }
    //calculeaza coord celui mai apropiat obj fata de un set de coord    A,si utilizeazo cand ma duc undeva si am nevoie de o anumita minge cu o anumita culoare(tre sa setez si pipelineindexu or smth)
    /*metoda ajutatoare ca am nev*/        /*PROBLEMA*/
    public Pose close_obj (OptionalInt i) {
        double[] NuMaiPoooooottt;
        if (i.isEmpty())
            NuMaiPoooooottt = devmin(RelativeCoords());
        else
            NuMaiPoooooottt = devmin(RelativeCoords2(i.getAsInt()));
        //pozitia celui mai apropiat dintre obiecte cu 0 avem toate obiectele cu 1 avem green si pe 2 avem purple
        return new Pose(NuMaiPoooooottt[0], NuMaiPoooooottt[1], NuMaiPoooooottt[2]);
    }

    //AICI VINE APRIL TAGU
    //IMPORTANT********************************************
    // DACA SE CITESC APRIL TAGURI MULTIPLE DE CULORI, TREBUIE VERIFICAT!!!!!!!
    private List<LLResultTypes.FiducialResult> getTags() {
        changepipeline(3); //tre sa schimb ca nush pipelinu folosit
        LLResult r = this.result;
        if (r == null || !r.isValid()) return new ArrayList<>();
        return r.getFiducialResults();

    }
    public int[] collor()
    {
        List <int[]> culoare = new ArrayList<>();
        List<LLResultTypes.FiducialResult> ficu = getTags();
        for(int i=0; i<ficu.size();i++)
        {
            int id = ficu.get(i).getFiducialId();
            switch (id){
                case 21:
                    culoare.add(new int[]{2, 1, 1});
                    break;
                case 22:
                    culoare.add(new int[]{1, 2, 1});
                    break;
                case 23:
                    culoare.add(new int[]{1, 1, 2});
                    break;
            }
        }
        return culoare.get(culoare.size()-1);
    }//returneaza 0 pt bila verde si 1 pt bila mov, incepe cu ultimu tag()
}