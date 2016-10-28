package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;




/**
 * Created by vasudevfamily on 10/27/16.
 */

@Autonomous(name = "Big_Bear_Blue_1", group = "Autonomous_OpMode")
public class Big_Bear_Blue_1 extends OpMode{
    static double timer =0;
    DcMotor rightMotorFront;
    DcMotor rightMotorBack;
    DcMotor leftMotorFront;
    DcMotor leftMotorBack;


    @Override
    public void init() {

    }

    @Override
    public void loop() {
        int v_state = 0;
        rightMotorFront = hardwareMap.dcMotor.get("rightMotor1");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotor2");
        leftMotorFront = hardwareMap.dcMotor.get("leftMotor1");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotor2");


        switch (v_state)
        {
            case 0:
                v_state = 2;
                break;

            case 2:

        }



    }
}
