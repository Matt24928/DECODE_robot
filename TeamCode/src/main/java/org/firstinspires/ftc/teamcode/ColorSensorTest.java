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
    }
}
