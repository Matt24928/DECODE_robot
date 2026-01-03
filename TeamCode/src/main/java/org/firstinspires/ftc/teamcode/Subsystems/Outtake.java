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
    double P1 = 14,P2 = 15,F1=21,F2=17.1020;
    double LOW1 = 0.59,JUMP1 = 0.34;
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
        READY,
        SPEEDING_UP,
        SLOWING_DOWN

    }
    public enum ShootState{
        READY,
        SHOT_GREEN,
        SHOT_PURPLE,
        UNAVAILABLE,
        WAIT

    }
    public enum Patterns{
        GPP,
        PGP,
        PGG,
        IDLE
    }
    public enum PatternState{
        IDLE,
        SHOOT_GREEN,
        SHOT_GREEN,
        SHOOT_PURPLE1,
        SHOT_PURPLE1,
        SHOOT_PURPLE2,
        SHOT_PURPLE2
    }
    public double SHOT_DELAY = 2.0;
    public double SHOT_DELAY2 = 3.5;
    public Patterns Pattern = Patterns.IDLE;
    public PatternState patternState = PatternState.IDLE;

    public boolean IsAuto = true;
    public ShootState Shoot1State,Shoot2State;
    public MotorState Motor1State = MotorState.OFF;
    public MotorState Motor2State = MotorState.OFF;
    public ElapsedTime Motor1Timer, Motor2Timer;
    public JumpState Jump1State = JumpState.IDLE;
    public JumpState Jump2State = JumpState.IDLE;
    private ElapsedTime Jump1Timer,Jump2Timer;
    private ElapsedTime Shoot1Timer,Shoot2Timer;
    public ElapsedTime PatternTime;
    public Outtake (HardwareMap hw){
        Jump1Timer = new ElapsedTime();
        Jump2Timer = new ElapsedTime();
        Motor1Timer = new ElapsedTime();
        Motor2Timer = new ElapsedTime();
        Shoot1Timer = new ElapsedTime();
        Shoot2Timer = new ElapsedTime();
        PatternTime = new ElapsedTime();

        motor_shooter_1 = hw.get(DcMotorEx.class, "MotorShooter_2");
        motor_shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_shooter_1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P1,0,0,F1);
        motor_shooter_1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);

        motor_shooter_2 = hw.get(DcMotorEx.class, "MotorShooter_1");
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

        t.addData("PatternState",patternState);
        t.addData("ShootState1",Shoot1State);
        t.addData("ShootState2",Shoot2State);
        t.addData("Pattern",Pattern);

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
        if(Jump1State == JumpState.JUMPING && Jump1Timer.seconds()>0.3){
            Low1();
            Jump1State = JumpState.IDLE;
        }
        if(Jump2State == JumpState.JUMPING && Jump2Timer.seconds()>0.3){
            Low2();
            Jump2State = JumpState.IDLE;
        }
        if(Motor1State == MotorState.READY && Shoot1State == ShootState.READY && Shoot1Timer.seconds()>2.5){ // timerul e pt cat timp sa astepte pana verifica daca poate arunca din nou
            if(OneCanShootGreen() && !IsAuto){
                ShootGreen();
            }else {
                Stop1();
                Motor1State = MotorState.OFF;
                Shoot1State = ShootState.WAIT;
            }
        }

        if(Motor1State == MotorState.READY && Shoot1State == ShootState.SHOT_GREEN && Shoot1Timer.seconds()>2.5){ // timerul e pt cat timp sa astepte pana verifica daca poate arunca din nou
                Stop1();
                Motor1State = MotorState.OFF;
                Shoot1State = ShootState.WAIT;

        }
        if(Motor1State == MotorState.READY && Shoot1State == ShootState.SHOT_PURPLE && Shoot1Timer.seconds()>2.5){ // timerul e pt cat timp sa astepte pana verifica daca poate arunca din nou
            Stop1();
            Motor1State = MotorState.OFF;
            Shoot1State = ShootState.WAIT;

        }
        if(Motor2State == MotorState.READY && Shoot2State == ShootState.SHOT_GREEN && Shoot2Timer.seconds()>2.5){ // timerul e pt cat timp sa astepte pana verifica daca poate arunca din nou
            Stop2();
            Motor2State = MotorState.OFF;
            Shoot2State = ShootState.WAIT;

        }
        if(Motor2State == MotorState.READY && Shoot2State == ShootState.SHOT_PURPLE && Shoot2Timer.seconds()>2.5){ // timerul e pt cat timp sa astepte pana verifica daca poate arunca din nou
            Stop2();
            Motor2State = MotorState.OFF;
            Shoot2State = ShootState.WAIT;

        }

        if(Motor1State == MotorState.READY && Shoot1State == ShootState.READY && Shoot1Timer.seconds()>2.5){
            if(OneCanShootPurple() && !IsAuto){
                ShootPurple();
            }else{
                Stop1();
                Motor1State = MotorState.OFF;
                Shoot1State = ShootState.WAIT;

            }
        }
        if(Motor2State == MotorState.READY && Shoot2State == ShootState.READY && Shoot2Timer.seconds()>2.5){
            if(TwoCanShootGreen() && !IsAuto){
                ShootGreen();
            }
            Stop2();
            Motor2State = MotorState.OFF;
            Shoot2State = ShootState.WAIT;
        }
        if (Motor2State == MotorState.READY && Shoot2State == ShootState.READY && Shoot2Timer.seconds() > 3.5) {
            if(TwoCanShootPurple() && !IsAuto){
                ShootPurple();
            }
            Stop2();
            Motor2State = MotorState.OFF;
            Shoot2State = ShootState.WAIT;

        }
        if(Motor1State == MotorState.SPEEDING_UP && RPM1 > 2800){
            Motor1State = MotorState.READY;
        }
        if(Motor2State == MotorState.SPEEDING_UP && RPM2 > 2800){
            Motor2State = MotorState.READY;
        }
        if(OneCanShootGreen() && Shoot1State == ShootState.WAIT && Motor1State == MotorState.READY){// cat asteapta ca motorul sa ajung la viteza necesara, o sa verificam si strict
            Shoot1Timer.reset();
            StartJump1();
            if(patternState == PatternState.IDLE){
                Shoot1State = ShootState.SHOT_GREEN;
            }
          //  Shoot1State = ShootState.SHOT_GREEN;
        }
        if(TwoCanShootGreen() && Shoot2State == ShootState.WAIT && Motor2State == MotorState.READY){
            Shoot2Timer.reset();
            StartJump2();
            if(patternState == PatternState.IDLE){
                Shoot2State = ShootState.SHOT_GREEN;
            }
         //   Shoot2State = ShootState.SHOT_GREEN;
        }
        if(OneCanShootPurple() && Shoot1State == ShootState.WAIT && Motor1State == MotorState.READY){
            Shoot1Timer.reset();
            StartJump1();
            if(patternState == PatternState.IDLE){
                Shoot1State = ShootState.SHOT_PURPLE;
            }
           // Shoot1State = ShootState.SHOT_PURPLE;
        }
        if(TwoCanShootPurple() && Shoot2State == ShootState.WAIT && Motor2State == MotorState.READY){
            Shoot2Timer.reset();
            StartJump2();
            if(patternState == PatternState.IDLE){
                Shoot2State = ShootState.SHOT_PURPLE;
            }
           // Shoot2State = ShootState.SHOT_PURPLE;
        }

        switch (Pattern) {
            case GPP:
                handleGPP();
                break;

        }
    }
    private void handleGPP(){
        switch (patternState){

            case IDLE:

                break;
            case SHOOT_GREEN:
                ShootGreen();
                PatternTime.reset();        // resetezi la începutul pattern-ului
                patternState = PatternState.SHOT_GREEN;
                break;

            case SHOT_GREEN:
                if(PatternTime.seconds() > SHOT_DELAY){
                    patternState = PatternState.SHOOT_PURPLE1;
                }
                break;

            case SHOOT_PURPLE1:
                ShootPurple();
                PatternTime.reset();        // resetam timerul doar la START-ul SHOT_PURPLE1
                patternState = PatternState.SHOT_PURPLE1;
                break;

            case SHOT_PURPLE1:
                if(PatternTime.seconds() > SHOT_DELAY){
                    patternState = PatternState.SHOOT_PURPLE2;
                }
                break;

            case SHOOT_PURPLE2:
                ShootPurple();
              /*  if(Jump1State == JumpState.IDLE){
                    StartJump1();
                } else if(Jump2State == JumpState.IDLE){
                    StartJump2();
                }

               */
                PatternTime.reset();
                patternState = PatternState.SHOT_PURPLE2;
                break;

            case SHOT_PURPLE2:
                if(PatternTime.seconds() > SHOT_DELAY2){
                    patternState = PatternState.IDLE;
                    Pattern = Patterns.IDLE; // pattern terminat
                    Shoot1State = ShootState.READY;
                    Shoot2State = ShootState.READY;
                }
                break;


//            case SHOOT_GREEN:
//                ShootGreen();
//                PatternTime.reset();
//                patternState = PatternState.SHOT_GREEN;
//                break;
//
//            case SHOT_GREEN:
//                if(PatternTime.seconds() > SHOT_DELAY){
//                    patternState = PatternState.SHOOT_PURPLE1;
//                }
//                break;
//
//            case SHOOT_PURPLE1:
//                ShootPurple();
//                PatternTime.reset();
//                patternState = PatternState.SHOT_PURPLE1;
//                break;
//
//            case SHOT_PURPLE1:
//                if(PatternTime.seconds() > SHOT_DELAY){
//                    patternState = PatternState.SHOOT_PURPLE2;
//                }
//                break;
//
//            case SHOOT_PURPLE2:
//                ShootPurple();
//                if(OneCanShootPurple()){
//                    Jump1();
//                }else if(TwoCanShootPurple()){
//                    Jump2();
//                }
//                PatternTime.reset();
//                patternState = PatternState.SHOT_PURPLE2;
//                break;
//
//            case SHOT_PURPLE2:
//                if(PatternTime.seconds() > SHOT_DELAY2){
//                    patternState = PatternState.IDLE;
//                    Pattern = Patterns.IDLE; // pattern terminat
//                }
//                break;
        }
    }



    public void ShootGreen(){
        if(OneCanShootGreen()){
            AutoShoot1();
            Shoot1Timer.reset();
            Shoot1State = ShootState.WAIT;
        }else if(TwoCanShootGreen()){
            AutoShoot2();
            Shoot2Timer.reset();
            Shoot2State = ShootState.WAIT;
        }
    }
    public void ShootPurple(){
        if(OneCanShootPurple()){
            AutoShoot1();
            Shoot1Timer.reset();
            Shoot1State = ShootState.WAIT;
        }else if(TwoCanShootPurple()){
            AutoShoot2();
            Shoot2Timer.reset();
            Shoot2State = ShootState.WAIT;
        }
    }

    public void AutoShoot1(){
        if(RPM1 > 2000){
            Motor1State = MotorState.READY;
            return;
        }
        Shoot1();
        //Motor1Timer.reset();
        Motor1State = MotorState.SPEEDING_UP;
    }

    public void AutoShoot2(){
        if(RPM2 > 2000) {
            Motor2State = MotorState.READY;
            return;
        }
        Shoot2();
        //Motor2Timer.reset();
        Motor2State = MotorState.SPEEDING_UP;
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
        motor_shooter_1.setVelocity(550,AngleUnit.RADIANS);
    }
    public void Shoot2(){
        motor_shooter_2.setVelocity(550,AngleUnit.RADIANS);
    }
    public void Stop1(){
        motor_shooter_1.setVelocity(0,AngleUnit.RADIANS);
    }
    public void Stop2(){
        motor_shooter_2.setVelocity(0,AngleUnit.RADIANS);
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

    public void moveFJ1() {
        jumper1.setPosition(jumper1.getPosition()+ 0.01);
    }

    public void moveBJ1() {
        jumper1.setPosition(jumper1.getPosition()- 0.01);
    }

    public void moveFJ2() {
        jumper2.setPosition(jumper2.getPosition()+ 0.01);
    }

    public void moveBJ2() {
        jumper2.setPosition(jumper2.getPosition()- 0.01);
    }

    public double getJ1() {
        return jumper1.getPosition();
    }

    public double getJ2() {
        return jumper2.getPosition();
    }


    public boolean OneCanShootGreen(){
        if (color1 == DetectedColor.GREEN){
            return true;
        }else return false;
    }
    public boolean OneCanShootPurple(){
        if(color1 == DetectedColor.PURPLE){
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