package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Configurable
@TeleOp
public class ColorSensorTest extends OpMode {

    Outtake sensor_1;
    Outtake sensor_2;

    @Override
    public void init() {
        sensor_1 = new Outtake(hardwareMap);
        sensor_2 = new Outtake(hardwareMap);
    }

    public void loop(){
        sensor_1.getDetectedColor1(telemetry);
        sensor_2.getDetectedColor2(telemetry);

        if(gamepad1.dpadUpWasPressed()){
            sensor_1.CheckForGreen(1);
            sensor_2.CheckForGreen(2);
        } // This will check for Green (hopefully)

        if(gamepad1.dpadDownWasPressed()){
            sensor_1.CheckForPurple(1);
            sensor_2.CheckForPurple(2);
        } // This will check for Purple (hopefully)

        /*
        Tatal nostru
        Care esti in ceruri
        Sfinteasca-se numele Tau
        Vie imparatia Ta
        Faca-se voia Ta
        Precum in cer asa si pre Pamant
        Painea noastra cea de toate zilele
        Da-ne-o noua astazi
        Si ne iarta noua gresalele noastre
        Precum si noi iertam gresitilor nostri
        Si nu ne duce pre noi in ispita
        Ci ne izbaveste de cel rau
        */

        telemetry.update();
    }

}