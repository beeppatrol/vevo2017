package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;




//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!



/**
 * Created by Avery on 10/22/2016.
 */
@Autonomous (name = "little_turtle_Blue_1", group = "Autonomous OpMode")
public class little_turtle_Blue_1 extends OpMode{
    static double timer=0;
    DcMotor leftMotorBack;
    DcMotor leftMotorFront;
    DcMotor rightMotorBack;
    DcMotor rightMotorFront;
    DcMotor vacuumMotor;
    DcMotor elevatorMotor;
    DcMotor shooterMotor;



    public void runOpMode() throws InterruptedException{
        leftMotorBack = hardwareMap.dcMotor.get("leftMotor1");
        leftMotorFront = hardwareMap.dcMotor.get("leftMotor2");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotor1");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotor2");
        vacuumMotor = hardwareMap.dcMotor.get("");
        elevatorMotor = hardwareMap.dcMotor.get("");
        shooterMotor = hardwareMap.dcMotor.get("");


//EDIT THESE VALUES TO MAKE THE PROPER STRENGTH
    }



    public void init(){
//EDIT THESE VALUES TO MAKE THE PROPER STRENGTH
}
public void loop(){
    int v_state = 0;
    switch (v_state)
    {
        case 0://EDIT THESE VALUES TO MAKE THE PROPER STRENGTH

            //set_drive_power(1.0, 1.0f);
           //uncomment this \when it works
           // run_using_encoders ();
            //reset_drive_encoders();
            v_state=2;
            break;

        case 2://EDIT THESE VALUES TO MAKE THE PROPER STRENGTH
           if (timer >= 30){
               elevatorMotor.setPower(1.0f);

               timer++;
           }else if(timer>=60 && timer <=30){
               elevatorMotor.setPower(0.0f);
               shooterMotor.setPower((1.0f));
               timer++;
           }
            shooterMotor.setPower(0.0f);
            v_state=4;
            timer = 0;
            break;
        case 4:
            if (timer >= 30){
            leftMotorBack.setPower(0.9f);
            leftMotorFront.setPower(0.9f);
            rightMotorBack.setPower(1.0f);
            rightMotorFront.setPower(1.0f);

            timer++;
            }leftMotorBack.setPower(0.0f);
            leftMotorFront.setPower(0.0f);
            rightMotorBack.setPower(0.0f);
            rightMotorFront.setPower(0.0f);
            v_state=6;
            break;
        case 6:
            if (timer >= 30){
                leftMotorBack.setPower(1.0f);
                leftMotorFront.setPower(1.0f);
                rightMotorBack.setPower(-1.0f);
                rightMotorFront.setPower(-1.0f);

                timer++;
            }leftMotorBack.setPower(0.0f);
            leftMotorFront.setPower(0.0f);
            rightMotorBack.setPower(0.0f);
            rightMotorFront.setPower(0.0f);
            v_state = 8;
            break;
        case 8:
            if (timer >= 30){
                leftMotorBack.setPower(1.0f);
                leftMotorFront.setPower(1.0f);
                rightMotorBack.setPower(1.0f);
                rightMotorFront.setPower(1.0f);

                timer++;
            }leftMotorBack.setPower(0.0f);
            leftMotorFront.setPower(0.0f);
            rightMotorBack.setPower(0.0f);
            rightMotorFront.setPower(0.0f);
            v_state = 10;
            break;
            //EDIT THESE VALUES TO MAKE THE PROPER STRENGTH





    }
}


}
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
