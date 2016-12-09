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
   // DcMotor rightMotorBack;
//   DcMotor rightMotor;
//    DcMotor leftMotor;

    public boolean finishedRunning = false;

    public int numCalls = 0;


    private int v_state;

    @Override
    public void init() {

        robot.rightMotor = hardwareMap.dcMotor.get("right_drive");

        robot.leftMotor = hardwareMap.dcMotor.get("left_drive");

        robot.colorSensor = hardwareMap.colorSensor.get("colorSensor");

        robot.lightSensor = hardwareMap.opticalDistanceSensor.get("oDS");

        robot.linearSlide = hardwareMap.crservo.get("linearSlide");

        v_state = 0;
    }

    public void driveForwards( double rightAmount, double leftAmount, double speed)
    {
        //probably not needed
       /* rightMotorFront.setPower(1.0f);
        rightMotorBack.setPower(1.0f);
        leftMotorFront.setPower(1.0f);
        leftMotorBack.setPower(1.0f); */
        numCalls++;
        telemetry.addData("say", "Drive forwards VROOOM!");


        final double     COUNTS_PER_MOTOR_REV    = 1440 ;
        final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
        final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
        final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

        final double     DRIVE_SPEED             = 0.6;
        final double     TURN_SPEED              = 0.5;

        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftAmount *-1 * COUNTS_PER_INCH);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightAmount * COUNTS_PER_INCH);
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);
        telemetry.addData("say:", newLeftTarget);
        telemetry.addData("say:", newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);


      /*  if (!robot.rightMotor.isBusy()){

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            finishedRunning = true; }*/
            //rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    public void driveBackwards( double rightAmount, double leftAmount, double speed) {//THESE VALUES HAVE NOT BEEN CHANGED
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
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightAmount*-1 * COUNTS_PER_INCH);
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);


        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.leftMotor.setPower(speed * -1);
        robot.rightMotor.setPower(speed * -1);




       /* rightMotorBack.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorFront.RunMode(DcMotor.RunMode.RUN_USING_ENCODER); */




        //if (){

        }



    public void turnLeft(double rightAmount, double leftAmount, double speed)
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

        //tell motors to go to the target
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //give motors the means to go to the target
        robot.rightMotor.setPower(speed);
        //rightMotorBack.setPower(1.0f);
        robot.leftMotor.setPower(speed *-1);



    }





    public void turnRight(double rightAmount, double leftAmount, double speed){

        final double Counts_Per_Motor_Rev  = 1440;
        final double Drive_Gear_Reduction = 2.0;
        final double Wheel_Diameter_Inches = 4.0;
        final double Counts_Per_Inch = (Drive_Gear_Reduction * Counts_Per_Motor_Rev);
        final double Drive_Speed = 0.6;
        final double Turn_Speed = 0.5;

        int newLeftTarget;
        int newRightTarget;
        //get target
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftAmount *-1 * Counts_Per_Inch);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightAmount *-1 * Counts_Per_Inch);
        //inform motor of there target
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.rightMotor.setPower(speed *-1);
       // rightMotorBack.setPower(-1.0f);
        robot.leftMotor.setPower(speed);






    }






    public void shootParticle(){
        //the code that will do this
    }


    private boolean controlVar = true;
     private double whiteLine;
    private int red;
    @Override
    public void loop() {

        if(controlVar) {
            telemetry.addData("say", "loop is running");
          controlVar = false;
        }

        //robot.rightMotor = hardwareMap.dcMotor.get("right_drive");

        //robot.leftMotor = hardwareMap.dcMotor.get("left_drive");

        int testRed;
        int testGreen;
        int testBlue;
        //boolean taskComplete;
        boolean onlyOnce = false;
        boolean taskComplete = true;
        switch (v_state) {
            case 0:
               // boolean taskComplete;
                telemetry.addData("say", "case 0 loop");
                robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(!onlyOnce) {
                    v_state=1;
                    onlyOnce = true;
                }
                finishedRunning = false;
                taskComplete = false;
                numCalls=0;
            //nothing

            case 1:


                driveForwards(3.2,3.2,1);
                v_state = 2;
               /* telemetry.addData("case 1 loop", numCalls);
            //test = robot.colorSensor.red();
                robot.colorSensor.enableLed(true);
               // telemetry.addData("red: ", test );
                double test = robot.lightSensor.getLightDetected();
                telemetry.addData("lightDetected = : ", test);
                //
                robot.rightMotor.setPower(0.2);
                robot.leftMotor.setPower(-0.2);

                double whiteLine = robot.lightSensor.getLightDetected();
                telemetry.addData("whiteLine: ", whiteLine);
                if( whiteLine >= 0.5) {
                    robot.rightMotor.setPower(0.0);
                    robot.leftMotor.setPower(0.0);
                    taskComplete = true;
                    v_state=2;} */
            case 2:

                if(!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy()){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    v_state = 3;
                }

            case 3:

                turnLeft(0.1, 0.1, 0.3);
                v_state = 4;

           /* case 4:

                if(!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy()){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    v_state = 5;
                }
            case 5:

                robot.rightMotor.setPower(0.2);
                robot.leftMotor.setPower(-0.2);

                double whiteLine = robot.lightSensor.getLightDetected();
                telemetry.addData("whiteLine: ", whiteLine);
                if( whiteLine >= 0.5) {
                    robot.rightMotor.setPower(0.0);
                    robot.leftMotor.setPower(0.0);

                    v_state=6;}
            case 6:
                driveBackwards(0.1,0.1,.5);
                v_state = 7;

            case 7:
                robot.linearSlide.setPower(1.0);



                //v_state++;


         /*   case 2:
                double whiteLine = robot.lightSensor.getLightDetected();
                telemetry.addData("whiteLine: ", whiteLine);
                if( whiteLine >= 0.8) {
                    robot.rightMotor.setPower(0.0);
                    robot.leftMotor.setPower(0.0);
                    telemetry.addData("reached if statement", "turtles");
                    taskComplete = true;

                    */
                //}
                /*PV
                if(finishedRunning ==true) {


                    v_state ++;

                }
                */

                // start program by going forwards

           /* case 2:

                if(!taskComplete) {
                   v_state = 1;
                }
                if(taskComplete) {

                  telemetry.addData("case 2 loop", numCalls);
                  telemetry.addData("left tgt", robot.leftMotor.getTargetPosition());
                  telemetry.addData("right tgt", robot.rightMotor.getTargetPosition());
                  telemetry.addData("left pos", robot.leftMotor.getCurrentPosition());
                  telemetry.addData("right pos", robot.rightMotor.getCurrentPosition());
                  numCalls++;

                  driveBackwards(.01, .01, .5);
                 if(taskComplete) {


                     v_state++;
                 }
              }
            case 3:

                    if (!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy()) {
                        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        v_state++;
                        telemetry.addData("state 3", "stop/rset");
                        v_state++;
                        //taskComplete =true;
                    }
*/

                /*PV
                telemetry.addData("say", "case 2 loop");

                finishedRunning =false;
                if(finishedRunning ==false){
                    v_state ++;
                }
                */
            // reset timers




            /*case 3:
                turnRight(0.38 , 0.38, 1);

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
                turnLeft(1,1, 1);
                timer++;
                if (timer == 100){

                }
                v_state ++;
                break;
            //turning into line TO DO: MAKE TURN BASED OFF COLOR SENSOR
            case 7:
                driveForwards(1000, 1000, 1);
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
                driveBackwards(1,1,1);
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
                turnRight(48743,8,1);
                timer++;
                if(timer ==80)
                {
               
                    v_state ++;
                    break;
                }
            case 26:
                //colour sensor required

*/
          /*  default:
                testRed = robot.colorSensor.red();
                telemetry.addData("red", testRed);
                robot.colorSensor.enableLed(true);
                testGreen = robot.colorSensor.green();
                telemetry.addData("green:", testGreen);
                testBlue = robot.colorSensor.blue();
                telemetry.addData("blue:", testBlue);
                telemetry.addData("Invalid Case", v_state);
               // robot.lightSensor.enableLed(true);
                v_state =1; */

        }


    telemetry.addData("V_STATE", v_state);

    }
}

//1: 61
//2: 67
//3: 67
//4: 61
//5: 67