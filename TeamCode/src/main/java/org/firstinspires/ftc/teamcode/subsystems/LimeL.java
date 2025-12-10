package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class LimeL  extends SubsystemBase {
    private Limelight3A limelight3A;
    private Telemetry telemetry;

    public LimeL(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);   // pipeline default
        limelight3A.setPollRateHz(100);  // update rapid
        limelight3A.start();             // pornește camera

    } private LLResult getResult() {
        LLResult r = limelight3A.getLatestResult();
        return (r != null && r.isValid()) ? r : null;
    }


    public void periodic() {
        LLResult result = getResult();
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);//0 este mov si 1 este verde ca asa vreau eu nu cum a zis omu de pe tutorial ca e blue si red , gen swtich to pipeline number 1
        limelight3A.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight3A.start(); // This tells Limelight to start looking!
        result.getPipelineIndex();


    }


    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            LLResult result = null;
            double tx = llResult.getTx(); // How far left or right the target is (degrees)
            double ty = llResult.getTy(); // How far up or down the target is (degrees)
            // How big the target looks (0%-100% of the image)
            double ta = result.getTa();
            telemetry.addData("Target X offset",llResult.getTx());
            telemetry.addData("Target Y offset", llResult.getTy());
            telemetry.addData("Target Area offset", llResult.getTa());
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

    }

    public double getTx() {
        LLResult result = limelight3A.getLatestResult();
        return (result != null && result.isValid()) ? result.getTx() : 0;
    }

    public double getTy() {
        LLResult result = limelight3A.getLatestResult();
        return (result != null && result.isValid()) ? result.getTy() : 0;
    }

    public boolean hasTarget() {
        LLResult result = limelight3A.getLatestResult();
        return result != null && result.isValid();
    }
}