package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.TreeMap;

@Configurable
public class Outtake extends SubsystemBase {
    public DcMotorEx motor_shooter_1,motor_shooter_2;
    double k1=1.03,k2=1.03,r=0.048;

    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    Servo jumper1,jumper2,angler;
    double P1 = 20,P2 = 20,F1=30,F2=30;
    double LOW1 = 0.55,JUMP1 = 0.37;
    double LOW2 = 0.25,JUMP2 = 0.45;
    //please baga valoare
    //HAHAHAHA DOUBLE JUMP, GET IT?
    public float hue1,hue2,sat1,sat2,val1,val2;
    public double current1,currentAlert1,Ticks1,RPM1,AngularSpeed1,LiniarSpeed1;
    public double current2,currentAlert2,Ticks2,RPM2,AngularSpeed2,LiniarSpeed2;
    double anglePoz;
    private TreeMap<Double, Double> AnglerPozToHoodAngle = new TreeMap<>();
    DetectedColor color1,color2;
    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public enum JumpState{
        IDLE,
        JUMPING
    }
    public enum MotorState{
        OFF,
        STANDBY,
        READY

    }
    public enum ShootState{
        SHOOTED_GREEN,
        SHOOTED_PURPLE,
        UNAVAILABLE,
        WAIT

    }
    public ShootState Shoot1State,Shoot2State;
    public MotorState Motor1State = MotorState.OFF;
    public MotorState Motor2State = MotorState.OFF;
    private ElapsedTime Motor1Timer, Motor2Timer;
    public JumpState Jump1State = JumpState.IDLE;
    public JumpState Jump2State = JumpState.IDLE;
    private ElapsedTime Jump1Timer,Jump2Timer;
    private ElapsedTime Shoot1Timer,Shoot2Timer;
    public Outtake (HardwareMap hw){
        Jump1Timer = new ElapsedTime();
        Jump2Timer = new ElapsedTime();
        Motor1Timer = new ElapsedTime();
        Motor2Timer = new ElapsedTime();
        Shoot1Timer = new ElapsedTime();
        Shoot2Timer = new ElapsedTime();

        motor_shooter_1 = hw.get(DcMotorEx.class, "MotorShooter_1");
        motor_shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P1,0,0,F1);
        motor_shooter_1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);

        motor_shooter_2 = hw.get(DcMotorEx.class, "MotorShooter_2");
        motor_shooter_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(P2,0,0,F2);
        motor_shooter_2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients2);

        jumper1 = hw.servo.get("Jumper_1");
        jumper2 = hw.servo.get("Jumper_2");
        angler = hw.servo.get("Angler");

        colorSensor1 = hw.get(NormalizedColorSensor.class,"SensorColor_1");
        colorSensor2 = hw.get(NormalizedColorSensor.class, "SensorColor_2");
        colorSensor1.setGain(2f);
        colorSensor2.setGain(2f);

        jumper1.setPosition(LOW1);
        jumper2.setPosition(LOW2);

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

            if (hue > 90 && hue < 180)
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
        t.addData("Jumper1","%f",jumper1.getPosition());

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

    public void Update(){ //valorile cu secundele trebuie reglate
        if(Jump1State == JumpState.JUMPING && Jump1Timer.seconds()>1.0){
            Low1();
            Jump1State = JumpState.IDLE;
        }
        if(Jump2State == JumpState.JUMPING && Jump2Timer.seconds()>1.0){
            Low2();
            Jump2State = JumpState.IDLE;
        }
        if(Motor1State == MotorState.READY && Motor1Timer.seconds()>5.0){
            Stop1();
            Motor1State = MotorState.OFF;
        }
        if(Motor2State == MotorState.READY && Motor2Timer.seconds()>5.0){ //cat timp sta motorul pornit
            Stop2();
            Motor2State = MotorState.OFF;
        }
        if(OneCanShootGreen() && Shoot1State == ShootState.WAIT && Shoot1Timer.seconds()>2.0){ // cat asteapta ca motorul sa ajung la viteza necesara, o sa verificam si strict
            StartJump1();
            Shoot1State = ShootState.SHOOTED_GREEN;
        }
        if(TwoCanShootGreen() && Shoot2State == ShootState.WAIT && Shoot2Timer.seconds()>2.0){
            StartJump2();
            Shoot2State = ShootState.SHOOTED_GREEN;
        }
        if(OneCanShootPurple() && Shoot1State == ShootState.WAIT && Shoot1Timer.seconds()>2.0){
            StartJump1();
            Shoot1State = ShootState.SHOOTED_PURPLE;
        }
        if(TwoCanShootPurple() && Shoot2State == ShootState.WAIT && Shoot2Timer.seconds()>2.0){
            StartJump2();
            Shoot2State = ShootState.SHOOTED_PURPLE;
        }
    }

    public void ShootGreen(){
        if(OneCanShootGreen()){
            Shoot1Timer.reset();
            AutoShoot1();
            Shoot1State = ShootState.WAIT;
        }else if(TwoCanShootGreen()){
            Shoot2Timer.reset();
            AutoShoot2();
            Shoot2State = ShootState.WAIT;
        }
    }
    public void ShootPurple(){
        if(OneCanShootPurple()){
            Shoot1Timer.reset();
            AutoShoot1();
            Shoot1State = ShootState.WAIT;
        }else if(TwoCanShootPurple()){
            Shoot2Timer.reset();
            AutoShoot2();
            Shoot2State = ShootState.WAIT;
        }
    }

    public void AutoShoot1(){
        if(Motor1State == MotorState.READY) return;
        Shoot1();
        Motor1Timer.reset();
        Motor1State = MotorState.READY;
    }
    public void AutoShoot2(){
        if(Motor2State == MotorState.READY) return;
        Shoot2();
        Motor2Timer.reset();
        Motor2State = MotorState.READY;
    }


    public void StartJump1(){
        if (Jump1State != JumpState.IDLE) return;
        Jump1();
        Jump1Timer.reset();
        Jump1State = JumpState.JUMPING;
    }
    public void StartJump2(){
        if (Jump2State != JumpState.IDLE) return;
        Jump2();
        Jump2Timer.reset();
        Jump2State = JumpState.JUMPING;
    }



    public double AnglePoz(){
        return angler.getPosition();
    }

    public void Shoot1(){
        motor_shooter_1.setPower(1);
    }
    public void Shoot2(){
        motor_shooter_2.setPower(1);
    }
    public void Stop1(){
        motor_shooter_1.setPower(0);
    }
    public void Stop2(){
        motor_shooter_2.setPower(0);
    }
    public void Low2(){
        jumper2.setPosition(LOW2);
    }
    public void Jump2() {
        jumper2.setPosition(JUMP2);
    }
    public void Low1(){
        jumper1.setPosition(LOW1);
    }
    public void Jump1() {
        jumper1.setPosition(JUMP1);
    }


    public boolean OneCanShootGreen(){
        if (color1 == DetectedColor.GREEN){
            return true;
        }else return false;
    }
    public boolean OneCanShootPurple(){
        if(color1 == DetectedColor.GREEN){
            return true;
        }else return false;
    }
    public boolean TwoCanShootGreen(){
        if(color2 == DetectedColor.GREEN){
            return true;
        }else return false;
    }
    public boolean TwoCanShootPurple(){
        if(color2 == DetectedColor.PURPLE){
            return true;
        }else return false;
    }
}