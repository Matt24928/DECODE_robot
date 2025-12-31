package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Sorter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Configurable
public class Teleop extends OpMode {
   // Telemetry telemetry;
   public static Pose startingPose;
    public Sorter sorter;
    public Outtake outtake;
    public Intake intake;
    private boolean automatedDrive = false;

    // Slow mode (opțional)
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();
    boolean fastShooter = false;
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        sorter = new Sorter(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
    }

    @Override
    public void loop() {

        try {
            // Copiază gamepadurile pentru comparație între frame-uri
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);


        } catch (Exception e) {
            telemetry.addLine("Exception  Assigning Gamepads. " + e);
        }

        follower.update();
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true );}




        // Dpad Up - moveP
        if (gamepad1.yWasPressed()) {
            intake.eat();
        }

        if (gamepad1.bWasPressed()) {
            intake.intake.setPower(0);
        }
        if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
            outtake.Jump1();
        }
        if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
            outtake.Jump2();
        }




        if (gamepad1.right_bumper) {
            outtake.motor_shooter_2.setPower(0);
        }
        if (gamepad1.right_trigger>0.3) {
            outtake.motor_shooter_2.setPower(1);
        }
        if(gamepad1.left_bumper){
            outtake.motor_shooter_1.setPower(0);
        }
        if(gamepad1.left_trigger>0.3){
            outtake.motor_shooter_1.setPower(1);
        }


        outtake.OuttakeData(telemetry);
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
