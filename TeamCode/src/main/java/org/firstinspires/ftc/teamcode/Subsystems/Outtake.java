package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Outtake {
    public DcMotorEx motor_shooter_1,motor_shooter_2;
    double k1=1.03,k2=1.03,r=0.048;

    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    Servo jumper1;
    Servo jumper2;
    double P1 = 0,P2 = 0,F1=0,F2=0;
    double jump,low;
    //please baga valoare
    //HAHAHAHA DOUBLE JUMP, GET IT?
    public float hue1,hue2,sat1,sat2,val1,val2;
    double current1,currentAlert1,Ticks1,RPM1,AngleSpeed1,LiniarSpeed1;
    double current2,currentAlert2,Ticks2,RPM2,AngleSpeed2,LiniarSpeed2;
    DetectedColor color1,color2;
    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public Outtake (HardwareMap hw){
        motor_shooter_1 = hw.get(DcMotorEx.class, "MotorShooter_1");
        motor_shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P1,0,0,F1);
        motor_shooter_1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);


        motor_shooter_2 = hw.get(DcMotorEx.class, "MotorShooter_2");
        motor_shooter_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_shooter_2.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(P2,0,0,F2);
        motor_shooter_2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients2);

        jumper1 = hw.servo.get("Jumper_1");
        jumper2 = hw.servo.get("Jumper_2");

        colorSensor1 = hw.get(NormalizedColorSensor.class,"SensorColor_1");
        colorSensor2 = hw.get(NormalizedColorSensor.class, "SensorColor_2");
        colorSensor1.setGain(2f);
        colorSensor2.setGain(2f);
    }
    private DetectedColor detectColor(
            NormalizedColorSensor sensor,
            Telemetry telemetry,
            String name
    ) {
        char[] c = new char[3];

        for(int i = 0; i < 3; i++) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            float[] hsv = new float[3];
            Color.colorToHSV(colors.toColor(), hsv);

            float hue = hsv[0];
            float sat = hsv[1];
            float val = hsv[2];

            /*telemetry.addLine("--" + name + "--");
            telemetry.addData("Hue_" + i, hue);
            telemetry.addData("Sat_" + i, sat);
            telemetry.addData("Val_" + i, val);*/

            if (val < 0.20 || sat < 0.25)
                c[i] = 'z';
            else if (hue > 95 && hue < 145)
                c[i] = 'G';
            else if (hue > 265 && hue < 295)
                c[i] = 'P';
            else
                c[i] = 'z';
        }

        if(c[0]=='G' && c[1]=='G' && c[2]=='G')
            return DetectedColor.GREEN;
        if(c[0]=='P' && c[1]=='P' && c[2]=='P')
            return DetectedColor.PURPLE;

        return DetectedColor.UNKNOWN;
    }

    public DetectedColor getDetectedColor1(Telemetry t){
        return detectColor(colorSensor1, t, "Sensor 1");
    }
    public DetectedColor getDetectedColor2(Telemetry t){
        return detectColor(colorSensor2, t, "Sensor 2");
    }
    public void OuttakeData(Telemetry t){
         current1 = motor_shooter_1.getCurrent(CurrentUnit.MILLIAMPS);
         currentAlert1 = motor_shooter_1.getCurrentAlert(CurrentUnit.MILLIAMPS);
         Ticks1 = motor_shooter_1.getVelocity();
         RPM1 = (Ticks1/28)*60;
         AngleSpeed1 = motor_shooter_1.getVelocity(AngleUnit.RADIANS);
         LiniarSpeed1 = k1*AngleSpeed1*r;

        t.addLine("--Motor 1--");
        t.addData("Current (mA)", "%.1f", current1);
        t.addData("Alert Current (mA)", "%.1f", currentAlert1);
        t.addData("Ticks/sec", "%.1f", Ticks1);
        t.addData("RPM", "%.1f", RPM1);
        t.addData("Angular Speed (rad/s)", "%.2f", AngleSpeed1);
        t.addData("Linear Speed (m/s)", "%.2f", LiniarSpeed1);


         current2 = motor_shooter_2.getCurrent(CurrentUnit.MILLIAMPS);
         currentAlert2 = motor_shooter_2.getCurrentAlert(CurrentUnit.MILLIAMPS);
         Ticks2 = motor_shooter_2.getVelocity();
         RPM2 = (Ticks2/28)*60;
         AngleSpeed2 = motor_shooter_2.getVelocity(AngleUnit.RADIANS);
         LiniarSpeed2 = k2*AngleSpeed2*r;

        t.addLine("--Motor 2--");
        t.addData("Current (mA)", "%.1f", current2);
        t.addData("Alert Current (mA)", "%.1f", currentAlert2);
        t.addData("Ticks/sec", "%.1f", Ticks2);
        t.addData("RPM", "%.1f", RPM2);
        t.addData("Angular Speed (rad/s)", "%.2f", AngleSpeed2);
        t.addData("Linear Speed (m/s)", "%.2f", LiniarSpeed1);

         color1 = getDetectedColor1(t);
         color2 = getDetectedColor2(t);

        t.addData("Sensor 1", "%s", color1);
        t.addData("Sensor 2", "%s", color2);
    }
    public void Shooter_ON_1() {
        motor_shooter_1.setPower(0.7);
    }
    public void Shooter_ON_2() {
        motor_shooter_2.setPower(1);
    }
    public void Shooter_OFF_1() {
        motor_shooter_1.setPower(0);
    }
    public void Shooter_OFF_2() {
        motor_shooter_2.setPower(0);
    }
    public void Jump_1(){
        jumper1.setPosition(jump);
    }
    public void Jump_2(){
        jumper2.setPosition(jump);
    }
    public void Lower_1(){
        jumper1.setPosition(low);
    }
    public void Lower_2(){
        jumper2.setPosition(low);
    }
    public void moveF2(){
        jumper2.setPosition(jumper2.getPosition()+0.05);
    }
    public void moveB2(){
        jumper2.setPosition(jumper2.getPosition()-0.05);
    }
    public void SetJ(){
        jump = jumper2.getPosition();
    }
    public void SetL(){
        low = jumper2.getPosition();
    }
}