package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    DcMotor motor_shooter_1;
    DcMotor motor_shooter_2;
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    Servo servo;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public Outtake (HardwareMap hw){
        motor_shooter_1 = hw.dcMotor.get("MotorShooter_1");
        motor_shooter_2 = hw.dcMotor.get("MotorShooter_2");
        servo = hw.servo.get("Servo_1");

        colorSensor1 = hw.get(NormalizedColorSensor.class,"SensorColor_1");
        colorSensor2 = hw.get(NormalizedColorSensor.class, "SensorColor_2");
    }


    public DetectedColor getDetectedColor1(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor1.getNormalizedColors();

        float normRed, normGreen, normBlue, normPurple;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        normPurple = (2*normRed + normBlue) / 2/colors.alpha;

        telemetry.addData("red_1", normRed);
        telemetry.addData("green_1",normGreen);
        telemetry.addData("blue_1", normBlue);
        telemetry.addData("purple_1", normPurple);
        /*
        Need to add good values that will give good accuracy to the color!!!
         */


        return DetectedColor.UNKNOWN; // Idk if we need something returned, soooo I'll leave it like that



    }

    public DetectedColor getDetectedColor2(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor1.getNormalizedColors();

        float normRed, normGreen, normBlue, normPurple;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        normPurple = (2*normRed + normBlue) / 2/colors.alpha;

        telemetry.addData("red_2", normRed);
        telemetry.addData("green_2",normGreen);
        telemetry.addData("blue_2", normBlue);
        telemetry.addData("purple_2", normPurple);
        /*
        Need to add good values that will give good accuracy to the color!!!
         */


        return DetectedColor.UNKNOWN; // We don't really need something returned at the momennt



    }


    void Shooter_ON_1() {
        motor_shooter_1.setPower(1);
    }
    void Shooter_ON_2() {
        motor_shooter_2.setPower(1);
    }
    void Shooter_OFF_1() {
        motor_shooter_1.setPower(0);
    }
    void Shooter_OFF_2() {
        motor_shooter_2.setPower(0);
    }
    void Pos_1(){
        servo.setPosition(0.6);
    }

    void Pos_2(){
        servo.setPosition(0.5);
    }

    void Pos_3(){
        servo.setPosition(0.75);
    }

}
