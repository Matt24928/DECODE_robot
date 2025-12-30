package org.firstinspires.ftc.teamcode.TeleOps;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Sorter;

@TeleOp
@Configurable
public class Teleop extends OpMode {
   // Telemetry telemetry;

    public Sorter sorter;

    public Outtake outtake;

    public Intake intake;
    private boolean automatedDrive = false;

    // Slow mode (opțional)
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        sorter = new Sorter(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
    }

    @Override
    public void loop() {
        outtake.getDetectedColor1(telemetry);
        telemetry.addLine();
        outtake.getDetectedColor2(telemetry);
        telemetry.update();

        // Dpad Up - moveP
        if (gamepad2.dpadUpWasPressed()) {
            outtake.moveF2();
        }
        if (gamepad2.dpadDownWasPressed()) {
            outtake.moveB2();
        }
        if (gamepad2.dpadLeftWasPressed()) {
            outtake.SetJ();
        }
        if (gamepad2.dpadRightWasPressed()) {
            outtake.SetL();
        }
        if (gamepad2.xWasPressed()) {
            outtake.Jump_2();
        }
        if (gamepad2.aWasPressed()) {
            outtake.Lower_2();
        }
        if(gamepad2.rightBumperWasPressed()){
            outtake.motor_shooter_1.setPower(0.7);
        }
        if(gamepad2.leftBumperWasPressed()){
            outtake.motor_shooter_1.setPower(0);
        }


        // Dpad Down - moveB
        if (gamepad1.dpadDownWasPressed()) {
            sorter.Middle();
        }

        // Dpad Left - SetS
        if (gamepad1.dpadLeftWasPressed()) {
            sorter.Left();
        }

        // Dpad Right - SetB
        if (gamepad1.dpadRightWasPressed()) {
            sorter.Right();
        }



        // X - retract
       if (gamepad1.xWasPressed()) {
            intake.eat();
        }

        // A - exemplu
        if (gamepad1.aWasPressed()) {
            intake.spit();
        }

        if(gamepad1.bWasPressed()) {
            intake.zero();
        }


        //double pos = outtake.jumper1.getPosition();
        double pos2 = sorter.getPos();
        double putere = intake.intake.getPower();
        //telemetry.addData("poz: ",pos);
        telemetry.addData("poz2: ", pos2);
        telemetry.addData("putere: ", putere);
       // telemetry.addData("caprita: ",outtake.jumper2.getPosition());
        // Slow mode toggle (opțional)
        telemetry.update();
        // Exemplu de utilizare slow mode la comenzi servo-uri (dacă vrei)
        // bounce.moveP(slowMode ? 0.5 : 1.0); // dacă ai overloading
    }
}
