package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sorter {

    private final Servo servo;

    private final double pos1 = 0.45;
    private final double pos2 = 1.0;
    private final double pos3 = 0.0;

    public Sorter(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class,"sorter");
    }

    public void goPos1() { servo.setPosition(pos1); }
    public void goPos2() { servo.setPosition(pos2); }
    public void goPos3() { servo.setPosition(pos3); }
}