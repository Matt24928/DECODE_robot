package subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class LimeLight  extends OpMode {
    private Limelight3A limelight3A;
    public double dx;
    public double dy;
    private double alpha;
    public double Heading;
    private double h;
    public double x,y;
    public double heading;
    double c;

    @Override
    public void init() {
        h=10;
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);//0 este mov si 1 este verde ca asa vreau eu nu cum a zis omu de pe tutorial ca e blue si red

    }

    @Override
    public void start() {
        limelight3A.start();
        alpha = 30;
    }

    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            dy = h/java.lang.Math.tan(Math.toRadians(llResult.getTy()+ alpha))*c;//c e constanta pt cm->inchi daca folosim cm pentru h
            dx = -java.lang.Math.tan(Math.toRadians(llResult.getTx()))*dy;
            double theta = follower.getPose().getHeading();
            double x = follower.getPose().getX()+dy*Math.cos(theta)-dx*Math.sin(theta);
            double y = follower.getPose().getY()+dy*Math.sin(theta)+dx*Math.cos(theta);
            Pose Obiect = new Pose(x,y,theta+Math.toRadians(llResult.getTx()));
            Pose Start = follower.getPose();
            PathChain chain = follower.pathBuilder().addPath(new BezierLine(Start,Obiect)).build();
            telemetry.addData("Target X offset", llResult.getTx());
            telemetry.addData("Target Y offset", llResult.getTy());
            telemetry.addData("Target Area offset", llResult.getTa());
        }
    }
}
