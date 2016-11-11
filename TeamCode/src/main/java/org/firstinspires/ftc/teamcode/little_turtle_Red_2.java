package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorController;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;



//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!
//THIS IS CURRENTLY A SKELETON PROGRAM. VALUES NEED TO CHANGE!


/**
 * Created by Avery on 10/22/2016.
 */
@Autonomous (name = "little_turtle_Red_2", group = "Autonomous OpMode")
public class little_turtle_Red_2 extends OpMode{
    HardwarePushbot robot   = new HardwarePushbot();
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
} public void turnLeft(float Power)
    {
        rightMotorFront.setPower(Power);
        rightMotorBack.setPower(Power);

        leftMotorFront.setPower(-Power);
        leftMotorBack.setPower(-Power);
    }

    public void turnRight(float Power){
        rightMotorFront.setPower(-Power);
        rightMotorBack.setPower(-Power);
        leftMotorFront.setPower(Power);
        leftMotorBack.setPower(Power);
    }
    public void driveBackwards(float Power)
    {
        rightMotorFront.setPower(-Power);
        rightMotorBack.setPower(-Power);
        leftMotorFront.setPower(-Power);
        leftMotorBack.setPower(-Power);
    }
    public void stopDrive()
    {
        rightMotorFront.setPower(0.0f);
        rightMotorBack.setPower(0.0f);
        leftMotorFront.setPower(0.0f);
        leftMotorBack.setPower(0.0f);
    }
    public void DriveForwards(float Power)
    {
        rightMotorFront.setPower(Power);
        rightMotorBack.setPower(Power);
        leftMotorFront.setPower(Power);
        leftMotorBack.setPower(Power);
    }
    public void shootParticle(){
        //the code that will do this
    }public void driveForwards( double rightAmount, double leftAmount) {
        //probably not needed
       /* rightMotorFront.setPower(1.0f);
        rightMotorBack.setPower(1.0f);
        leftMotorFront.setPower(1.0f);
        leftMotorBack.setPower(1.0f); */
        final double COUNTS_PER_MOTOR_REV = 1440;
        final double DRIVE_GEAR_REDUCTION = 2.0;
        final double WHEEL_DIAMETER_INCHES = 4.0;
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

        final double DRIVE_SPEED = 0.6;
        final double TURN_SPEED = 0.5;

        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftAmount * COUNTS_PER_INCH);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightAmount * COUNTS_PER_INCH);
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.leftMotor2.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);
        robot.rightMotor2.setTargetPosition(newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setPower(1.0f);
        robot.leftMotor2.setPower(1.0f);
        robot.rightMotor.setPower(1.0f);
        robot.rightMotor2.setPower(1.0f);

        if (leftMotorBack.isBusy() && leftMotorFront.isBusy() && rightMotorBack.isBusy() && rightMotorFront.isBusy()) {
            robot.leftMotor.setPower(0.0f);
            robot.rightMotor.setPower(0.0f);
            robot.leftMotor2.setPower(0.0f);
            robot.rightMotor2.setPower(0.0f);
        }
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
            leftMotorBack.setPower(1.0f);
            leftMotorFront.setPower(1.0f);
            rightMotorBack.setPower(0.9f);
            rightMotorFront.setPower(0.9f);

            timer++;
            }
            leftMotorBack.setPower(0.0f);
            leftMotorFront.setPower(0.0f);
            rightMotorBack.setPower(0.0f);
            rightMotorFront.setPower(0.0f);
            v_state=6;
            break;
        case 6:
            if (timer >= 30){
                leftMotorBack.setPower(-1.0f);
                leftMotorFront.setPower(-1.0f);
                rightMotorBack.setPower(1.0f);
                rightMotorFront.setPower(1.0f);

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
