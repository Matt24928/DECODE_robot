package org.firstinspires.ftc.teamcode;

import static com.sun.tools.doclint.Entity.theta;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;



@Autonomous
public class AutoLimeLight extends OpMode{
    private Limelight3A limelight3A;
    @Override
    public void init() {   limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

    }
    @Override
    public void start(){

        limelight3A.start();
    }

    @Override
    public void loop() {


    }
}
// plan de actiune citim qr culori vedem unde sunt bilele, stocam coordonate, calculam care e cea mai apropiata de cos, de culoarea specifica necesara pentru puncte in plus, sau , mai tarziu nu culoare specifica,apoi ne indreptam spre ia luand in considerare ca nu ar trebui sa ne atingem de alte bile in faza cu

