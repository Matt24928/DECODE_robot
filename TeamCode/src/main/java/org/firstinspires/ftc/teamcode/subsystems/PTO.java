package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PTO {
    Servo pto1;
    Servo pto2;
    public PTO(HardwareMap hw) {//PTO= Power Take Off
        pto1 = hw.servo.get("servoPTO1");+
        pto2 = hw.servo.get("servoPTO2");
    }
    public void PtoN()//N de la Normal; Sa schimbam valorile ca nu cred ca-s bune
    {
        pto1.setPosition(1);
        pto2.setPosition(1);
    }

    public void PtoA()//A de la Activ
    {
        pto1.setPosition(0);
        pto2.setPosition(0);

    }

}