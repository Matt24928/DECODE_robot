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
    Servo jumper1;
    Servo jumper2;

    float normRed_1, normGreen_1, normBlue_1, normPurple_1;
    float normRed_2, normGreen_2, normBlue_2, normPurple_2;

    double jump;
    //please baga valoare
    //HAHAHAHA DOUBLE JUMP, GET IT?

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public Outtake (HardwareMap hw){
        motor_shooter_1 = hw.dcMotor.get("MotorShooter_1");
        motor_shooter_2 = hw.dcMotor.get("MotorShooter_2");
        jumper1 = hw.servo.get("Jumper_1");
        jumper2 = hw.servo.get("Jumper_2");

        colorSensor1 = hw.get(NormalizedColorSensor.class,"SensorColor_1");
        colorSensor2 = hw.get(NormalizedColorSensor.class, "SensorColor_2");
    }


    public DetectedColor getDetectedColor1(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor1.getNormalizedColors();


        normRed_1 = colors.red / colors.alpha;
        normGreen_1 = colors.green / colors.alpha;
        normBlue_1 = colors.blue / colors.alpha;
        normPurple_1 = (2*normRed_1 + normBlue_1) / 2/colors.alpha;

        telemetry.addData("red_1", normRed_1);
        telemetry.addData("green_1",normGreen_1);
        telemetry.addData("blue_1", normBlue_1);
        telemetry.addData("purple_1", normPurple_1);
        /*
        Need to add good values that will give good accuracy to the color!!!
         */


        return DetectedColor.UNKNOWN; // Idk if we need something returned, soooo I'll leave it like that



    }

    public DetectedColor getDetectedColor2(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor1.getNormalizedColors();


        normRed_2 = colors.red / colors.alpha;
        normGreen_2 = colors.green / colors.alpha;
        normBlue_2 = colors.blue / colors.alpha;
        normPurple_2 = (2*normRed_2 + normBlue_2) / 2/colors.alpha;

        telemetry.addData("red_2", normRed_2);
        telemetry.addData("green_2",normGreen_2);
        telemetry.addData("blue_2", normBlue_2);
        telemetry.addData("purple_2", normPurple_2);
        /*
        Need to add good values that will give good accuracy to the color!!!
         */


        return DetectedColor.UNKNOWN; // We don't really need something returned at the momennt



    }


    public boolean CheckForGreen(int sensorNumber) {
        float greenValue = (sensorNumber == 1) ? normGreen_1 : normGreen_2;
        float redValue   = (sensorNumber == 1) ? normRed_1   : normRed_2;

        // Idk, I THINK this needs to be adjusted after checking the real values?
        boolean isGreen = greenValue > redValue && greenValue > 0.4;

        if(isGreen){
            if(sensorNumber == 1) {
                Shooter_ON_1();
                Jump_1();
            }
            else {
                Shooter_ON_2();
                Jump_2();
            }
        }

        return isGreen;
    }

    public boolean CheckForPurple(int sensorNumber) {
        float purpleValue = (sensorNumber == 1) ? normPurple_1 : normPurple_2;
        float greenValue  = (sensorNumber == 1) ? normGreen_1  : normGreen_2;

        // I think the same here, idk please check guys
        boolean isPurple = purpleValue > greenValue && purpleValue > 0.4;

        if(isPurple){
            if(sensorNumber == 1) {Shooter_ON_1();}
            else {Shooter_ON_2();}
        }

        return isPurple;
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
    void Jump_1(){
        jumper1.setPosition(jump);
    }
    void Jump_2(){
        jumper2.setPosition(jump);
    }
    void Lower_1(){
        jumper1.setPosition(0);
    }
    void Lower_2(){
        jumper2.setPosition(0);
    }
}