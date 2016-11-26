package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


/**
 * Created by vasudevfamily on 10/27/16.
 */

@Autonomous(name = "Big_Bear_Blue_1", group = "Autonomous_OpMode")
public class Big_Bear_Blue_1 extends OpMode{
    HardwarePushbot robot   = new HardwarePushbot();
    static double timer =0;
    DcMotor rightMotor;
   // DcMotor rightMotorBack;
    DcMotor leftMotorFront;
    DcMotor leftMotorBack;
    public boolean finishedRunning = false;
ColorSensor colorSensor;
    Servo elevatorVacuum;


    @Override
    public void init() {

    }

    public void driveForwards( double rightAmount, double leftAmount)
    {
        //probably not needed
       /* rightMotorFront.setPower(1.0f);
        rightMotorBack.setPower(1.0f);
        leftMotorFront.setPower(1.0f);
        leftMotorBack.setPower(1.0f); */
        final double     COUNTS_PER_MOTOR_REV    = 1440 ;
        final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
        final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
        final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

        final double     DRIVE_SPEED             = 0.6;
        final double     TURN_SPEED              = 0.5;

        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftAmount * COUNTS_PER_INCH);
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
        robot.rightMotor2.setPower (1.0f);

        if(leftMotorBack.isBusy() && leftMotorFront.isBusy() && rightMotor.isBusy()){
            leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            finishedRunning = true;
            //rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }}
    public void driveBackwards( double rightAmount, double leftAmount) {//THESE VALUES HAVE NOT BEEN CHANGED
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

        robot.leftMotor.setPower(-1.0f);
        robot.leftMotor2.setPower(-1.0f);
        robot.rightMotor.setPower(-1.0f);
        robot.rightMotor2.setPower(-1.0f);

        if (leftMotorBack.isBusy() && leftMotorFront.isBusy() && rightMotor.isBusy()) {
            leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            finishedRunning =true;
            //rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

       /* rightMotorBack.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorFront.RunMode(DcMotor.RunMode.RUN_USING_ENCODER); */




        //if (){

        }



    public void turnLeft(double rightAmount, double leftAmount)
    {
        final double Counts_Per_Motor_Rev  = 1440;
        final double Drive_Gear_Reduction = 2.0;
        final double Wheel_Diameter_Inches = 4.0;
        final double Counts_Per_Inch = (Drive_Gear_Reduction * Counts_Per_Motor_Rev);
        final double Drive_Speed = 0.6;
        final double Turn_Speed = 0.5;

        int newLeftTarget;
        int newRightTarget;
        //get target
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftAmount * Counts_Per_Inch);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightAmount * Counts_Per_Inch);
        //inform motor of there target
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);
        robot.leftMotor2.setTargetPosition(newLeftTarget);
        robot.rightMotor2.setTargetPosition(newRightTarget);
        //tell motors to go to the target
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //give motors the means to go to the target
        rightMotor.setPower(1.0f);
        //rightMotorBack.setPower(1.0f);
        leftMotorFront.setPower(-1.0f);
        leftMotorBack.setPower(-1.0f);

        if(leftMotorFront.isBusy() && leftMotorBack.isBusy() && rightMotor.isBusy())
        {
            leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            finishedRunning = true;
            //rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        }
    }





    public void turnRight(double rightAmount, double leftAmount){

        final double Counts_Per_Motor_Rev  = 1440;
        final double Drive_Gear_Reduction = 2.0;
        final double Wheel_Diameter_Inches = 4.0;
        final double Counts_Per_Inch = (Drive_Gear_Reduction * Counts_Per_Motor_Rev);
        final double Drive_Speed = 0.6;
        final double Turn_Speed = 0.5;

        int newLeftTarget;
        int newRightTarget;
        //get target
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftAmount * Counts_Per_Inch);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightAmount * Counts_Per_Inch);
        //inform motor of there target
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);
        robot.leftMotor2.setTargetPosition(newLeftTarget);
        robot.rightMotor2.setTargetPosition(newRightTarget);
        //tell motors to go to the target
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //give motors the means to go to the target


        rightMotor.setPower(-1.0f);
       // rightMotorBack.setPower(-1.0f);
        leftMotorFront.setPower(1.0f);
        leftMotorBack.setPower(1.0f);

        if(leftMotorFront.isBusy() && leftMotorBack.isBusy() && rightMotor.isBusy())
        {
            //JUST IN CASE WE NEED IT oops all caps`
            /*robot.leftMotor.setPower(0.0f);
            robot.rightMotor.setPower(0.0f);
            robot.leftMotor2.setPower(0.0f);
            robot.rightMotor2.setPower(0.0f);*/
            leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            finishedRunning = true;
           // rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        }



    }






    public void shootParticle(){
        //the code that will do this
    }




    @Override
    public void loop() {
        int v_state = 0;
        rightMotor = hardwareMap.dcMotor.get("rightMotor1");
       // rightMotorBack = hardwareMap.dcMotor.get("rightMotor2");
        leftMotorFront = hardwareMap.dcMotor.get("leftMotor1");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotor2");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        elevatorVacuum = hardwareMap.servo.get ("vacuumandElevator");


        switch (v_state) {
            case 0:
                v_state  ++;
                break;
            //nothing

            case 1:
                driveForwards(3, 3);
                if(finishedRunning ==true) {

                    v_state ++;
                    break;

                }


                // start program by going forwards

            case 2:
                finishedRunning =false;
                if(finishedRunning ==false){
                    v_state ++;
                    break;
                }

            // reset timers


            case 3:
                turnRight(0.38 , 0.38);

                if(finishedRunning == true)
                {


                    v_state ++;
                    break;
                }
                //turning before advancing towards beacon
            case 4:
                finishedRunning = false;
                if(finishedRunning ==false) {
                    v_state++;
                    break;
                }
            // reset timers
            case 5:
                //color sensing required
                v_state ++;
                break;
            //advancing to beacon and a little further
            case 6:
                turnLeft(1,1);
                timer++;
                if (timer == 100){

                }
                v_state ++;
                break;
            //turning into line TO DO: MAKE TURN BASED OFF COLOR SENSOR
            case 7:
                driveForwards(1000, 1000);
                timer++;
                if(timer ==10)
                {

                    v_state ++;
                    break;
                }
            case 8:
                //colour sensor required
                v_state ++;
                break;
            case 9:
                //requires parts we don't have on robot yet yay! (the yay is sarcastic)
                v_state ++;
                break;
            case 22:
                driveBackwards(1,1);
                timer++;
                if(timer ==10)
                {

                    v_state ++;
                    break;
                }
            case 20:
                shootParticle();
                v_state ++;
                break;
            case 24:
                turnRight(48743,8);
                timer++;
                if(timer ==80)
                {
               
                    v_state ++;
                    break;
                }
            case 26:
                //colour sensor required





        }




    }
}
