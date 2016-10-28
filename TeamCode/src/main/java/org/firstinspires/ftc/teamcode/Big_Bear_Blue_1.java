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

    public void driveForwards()
    {
        rightMotorFront.setPower(1.0f);
        rightMotorBack.setPower(1.0f);
        leftMotorFront.setPower(1.0f);
        leftMotorBack.setPower(1.0f);
    }
    public void turnLeft()
    {
        rightMotorFront.setPower(1.0f);
        rightMotorBack.setPower(1.0f);
        leftMotorFront.setPower(-1.0f);
        leftMotorBack.setPower(-1.0f);
    }
    public void turnRight(){
        rightMotorFront.setPower(-1.0f);
        rightMotorBack.setPower(-1.0f);
        leftMotorFront.setPower(1.0f);
        leftMotorBack.setPower(1.0f);
    }
    public void driveBackwards()
    {
        rightMotorFront.setPower(-1.0f);
        rightMotorBack.setPower(-1.0f);
        leftMotorFront.setPower(-1.0f);
        leftMotorBack.setPower(-1.0f);
    }
    public void stopDrive()
    {
        rightMotorFront.setPower(0.0f);
        rightMotorBack.setPower(0.0f);
        leftMotorFront.setPower(0.0f);
        leftMotorBack.setPower(0.0f);
    }
    public void shootParticle(){
        //the code that will do this
    }

    @Override
    public void loop() {
        int v_state = 0;
        rightMotorFront = hardwareMap.dcMotor.get("rightMotor1");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotor2");
        leftMotorFront = hardwareMap.dcMotor.get("leftMotor1");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotor2");




        switch (v_state) {
            case 0:
                v_state = 2;
                break;
            //nothing

            case 2:
                driveForwards();
                timer++;
                if (timer == 30) {
                    stopDrive();
                    timer = 0;
                    v_state = 4;
                    break;
                }// start program by going forwards

            case 4:
                timer = 0;
                v_state = 6;
                break;
            // reset timers


            case 6:
                turnRight();
                timer++;
                if(timer==100)
                {
                    stopDrive();
                    timer = 0;
                    v_state=8;
                    break;
                }
                //turning before advancing towards beacon
            case 8:
                timer = 0;
                v_state = 10;
                break;
            // reset timers
            case 10:
                //color sensing required
                v_state = 12;
                break;
            //advancing to beacon and a little further
            case 12:
                turnLeft();
                timer++;
                if (timer == 100){
                   stopDrive();

                }
                v_state = 14;
                break;
            //turning into line TO DO: MAKE TURN BASED OFF COLOR SENSOR
            case 14:
                driveForwards();
                timer++;
                if(timer ==10)
                {
                   stopDrive();
                    v_state = 16;
                    break;
                }
            case 16:
                //colour sensor required
                v_state = 18;
                break;
            case 18:
                //requires parts we don't have on robot yet yay! (the yay is sarcastic)
                v_state = 20;
                break;
            case 22:
                driveBackwards();
                timer++;
                if(timer ==10)
                {
                   stopDrive();
                    v_state = 20;
                    break;
                }
            case 20:
                shootParticle();
                v_state = 24;
                break;
            case 24:
                turnRight();
                timer++;
                if(timer ==80)
                {
                stopDrive();
                    v_state =26;
                    break;
                }
            case 26:
                //colour sensor required





        }




    }
}
