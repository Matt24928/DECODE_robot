package org.firstinspires.ftc.teamcode.Configs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
@TeleOp
public class IntakeTest extends OpMode {
    Intake intake;
    @Override
    public void init() {
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.leftBumperWasPressed()){
            intake.depowAlittle();
        }
        if(gamepad1.rightBumperWasPressed()){
            intake.powAlittle();
        }
        if(gamepad1.dpadRightWasPressed()){
            intake.setEat();
        }
        if(gamepad1.dpadLeftWasPressed()){
            intake.setSpit();
        }
        if(gamepad1.aWasPressed()){
            intake.eat();
        }
        if(gamepad1.bWasPressed()){
            intake.spit();
        }
        telemetry.addData("IntakePow",intake.intake.getPower());

    }
}
