package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
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

import java.util.TreeMap;

@Configurable
public class Outtake {
    public DcMotorEx motor_shooter_1,motor_shooter_2;
    double k1=1.03,k2=1.03,r=0.048;

    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    Servo jumper1,jumper2,angler;
    double P1 = 20,P2 = 20,F1=30,F2=30;
    double LOW1 = 0.25,JUMP1 = 0.45;
    double LOW2 = 0.25,JUMP2 = 0.45;
    //please baga valoare
    //HAHAHAHA DOUBLE JUMP, GET IT?
    public float hue1,hue2,sat1,sat2,val1,val2;
    double current1,currentAlert1,Ticks1,RPM1,AngularSpeed1,LiniarSpeed1;
    double current2,currentAlert2,Ticks2,RPM2,AngularSpeed2,LiniarSpeed2;
    double anglePoz;
    private TreeMap<Double, Double> AnglerPozToHoodAngle = new TreeMap<>();
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
        angler = hw.servo.get("Angler");

        colorSensor1 = hw.get(NormalizedColorSensor.class,"SensorColor_1");
        colorSensor2 = hw.get(NormalizedColorSensor.class, "SensorColor_2");
        colorSensor1.setGain(2f);
        colorSensor2.setGain(2f);

        AnglerPozToHoodAngle.put(0.1,30.0); //ohoho cat avem de scris aici(dependenta intre unghiul hoodului si pozitia servoului)
    }

    private DetectedColor detectColor(
            NormalizedColorSensor sensor,
            Telemetry telemetry,
            String name
    ) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            float[] hsv = new float[3];
            Color.colorToHSV(colors.toColor(), hsv);

            float hue = hsv[0];
            float sat = hsv[1];
            float val = hsv[2];

            telemetry.addLine("--" + name + "--");
            telemetry.addData("Hue_", hue);
            telemetry.addData("Sat_", sat);
            telemetry.addData("Val_", val);

            if (hue > 95 && hue < 180)
                return DetectedColor.GREEN;
            else if (hue > 220 && hue < 295)
                return DetectedColor.PURPLE;
            else return DetectedColor.UNKNOWN;
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
         AngularSpeed1 = motor_shooter_1.getVelocity(AngleUnit.RADIANS);
         LiniarSpeed1 = k1*AngularSpeed1*r;

        t.addLine("--Motor 1--");
        t.addData("Current (mA)", "%.1f", current1);
        t.addData("Alert Current (mA)", "%.1f", currentAlert1);
        t.addData("Ticks/sec", "%.1f", Ticks1);
        t.addData("RPM", "%.1f", RPM1);
        t.addData("Angular Speed (rad/s)", "%.2f", AngularSpeed1);
        t.addData("Linear Speed (m/s)", "%.2f", LiniarSpeed1);


         current2 = motor_shooter_2.getCurrent(CurrentUnit.MILLIAMPS);
         currentAlert2 = motor_shooter_2.getCurrentAlert(CurrentUnit.MILLIAMPS);
         Ticks2 = motor_shooter_2.getVelocity();
         RPM2 = (Ticks2/28)*60;
         AngularSpeed2 = motor_shooter_2.getVelocity(AngleUnit.RADIANS);
         LiniarSpeed2 = k2*AngularSpeed2*r;

        t.addLine("--Motor 2--");
        t.addData("Current (mA)", "%.1f", current2);
        t.addData("Alert Current (mA)", "%.1f", currentAlert2);
        t.addData("Ticks/sec", "%.1f", Ticks2);
        t.addData("RPM", "%.1f", RPM2);
        t.addData("Angular Speed (rad/s)", "%.2f", AngularSpeed2);
        t.addData("Linear Speed (m/s)", "%.2f", LiniarSpeed1);

         color1 = getDetectedColor1(t);
         color2 = getDetectedColor2(t);

        t.addData("Sensor 1", "%s", color1);
        t.addData("Sensor 2", "%s", color2);

        anglePoz = AnglePoz();
        t.addData("Shooter Poz","%f",anglePoz);
        t.addData("Jumper2","%f",jumper2.getPosition());

    }

    /*public double getHoodAngle(double anglePoz){
        if (AnglerPozToHoodAngle.isEmpty()) {
            // map-ul e gol → returnezi un default
            return 0.0;
        }

        Double floorKey = AnglerPozToHoodAngle.floorKey(anglePoz);
        Double ceilKey = AnglerPozToHoodAngle.ceilingKey(anglePoz);

        // daca una e null, folosim cealalta
        if (floorKey == null) return AnglerPozToHoodAngle.get(ceilKey);
        if (ceilKey == null) return AnglerPozToHoodAngle.get(floorKey);

        // alegem cea mai apropiata
        if (anglePoz - floorKey < ceilKey - anglePoz) {
            return AnglerPozToHoodAngle.get(floorKey);
        } else {
            return AnglerPozToHoodAngle.get(ceilKey);
        }
    }*/ //nu folosim cred



    public double AnglePoz(){
        return angler.getPosition();
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
        jumper1.setPosition(JUMP1);
    }
    public void Jump_2(){
        jumper2.setPosition(JUMP2);
    }
    public void Lower_1(){
        jumper1.setPosition(LOW1);
    }
    public void Lower_2(){
        jumper2.setPosition(LOW2);
    }
    public void moveF2(){
        jumper2.setPosition(jumper2.getPosition()+0.05);
    }
    public void moveB2(){
        jumper2.setPosition(jumper2.getPosition()-0.05);
    }

}