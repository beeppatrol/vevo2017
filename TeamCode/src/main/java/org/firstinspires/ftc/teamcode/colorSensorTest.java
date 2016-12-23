package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotorController;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


/**
 * Created by vasudevfamily on 10/27/16.
 */

@Autonomous(name = "colorSensorTest", group = "Autonomous_OpMode")
public class colorSensorTest extends OpMode {
    HardwarePushbot robot = new HardwarePushbot();

    private ColorSensor colorSensor;
    //colorSensor = hardwareMap.colorSensor.get("colorSensor")
@Override
    public void loop() {
    colorSensor = hardwareMap.colorSensor.get("colorSensor");
    colorSensor.blue();
    telemetry.addData("colors: ", colorSensor);

}
    @Override
    public void init() {

    }}





