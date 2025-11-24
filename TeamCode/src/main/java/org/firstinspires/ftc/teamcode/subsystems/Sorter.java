package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sorter extends SubsystemBase {
    Servo servo;
    double pos1=0.45;
    double pos2=1;
    double pos3=1.5;
    void pos1(){
        servo.setPosition(pos1);
    }
    void pos2(){
        servo.setPosition(pos2);
    }
    void pos3(){
        servo.setPosition(pos3);
    }

}

