package org.firstinspires.ftc.teamcode.Configs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@TeleOp(name = "Outtake Test", group = "Configs")
public class OuttakeTest extends OpMode {

    public DcMotorEx shooter;

    public double highVelocity = 1500;
    public double lowVelocity = 900;
    Outtake outtake;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init(){
        shooter = hardwareMap.get(DcMotorEx.class, "MotorShooter_2");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake = new Outtake(hardwareMap);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete yay");
    }

    @Override
    public void loop(){
        if(gamepad1.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if(gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1)% stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad2.xWasPressed()) {
            outtake.Jump2();
        }

        //Setting new coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //Setting our velocity
        shooter.setVelocity(curTargetVelocity);

        double curVelocity = shooter.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine();
        telemetry.addData("Tuning P", "%.4f (D-Pad Up or Down)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad Left or Right)", F);
        telemetry.addData("Step Size", "%.4f (B button)", stepSizes[stepIndex]);

    }

}