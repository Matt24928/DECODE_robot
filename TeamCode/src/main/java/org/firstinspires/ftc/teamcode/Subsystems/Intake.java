package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    public DcMotor intake;
    double eat,spit;

    public Intake(HardwareMap hw){
        intake = hw.get(DcMotor.class,"Intake_m");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
     public void eat(){
        intake.setPower(eat);
     }
     public void spit(){
        intake.setPower(spit);
     }
     public void powAlittle(){
        intake.setPower(intake.getPower()+0.1);
     }
    public void depowAlittle(){
        intake.setPower(intake.getPower()-0.1);
    }
    public void setSpit(){
        spit = intake.getPower();
    }
    public void setEat(){
        eat = intake.getPower();
    }

     public void zero(){
        intake.setPower(0);
        intake.getZeroPowerBehavior();
     }
}
