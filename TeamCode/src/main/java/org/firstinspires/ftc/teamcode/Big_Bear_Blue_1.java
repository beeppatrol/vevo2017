package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class Big_Bear_Blue_1 extends OpMode{
    HardwarePushbot robot   = new HardwarePushbot();
    static double timer =0;
   // DcMotor rightMotorBack;
//   DcMotor rightMotor;
//    DcMotor leftMotor;

    public boolean finishedRunning = false;

    public int numCalls = 0;



    @Override
    public void init() {

        robot.rightMotor = hardwareMap.dcMotor.get("right_drive");

        robot.leftMotor = hardwareMap.dcMotor.get("left_drive");

       robot.colorSensor = hardwareMap.colorSensor.get("colorSensor");

       /* robot.lightSensor = hardwareMap.opticalDistanceSensor.get("oDS");
        robot.lightSensor2 = hardwareMap.opticalDistanceSensor.get("oDS2");*/

       // robot.linearSlide = hardwareMap.crservo.get("linearSlide");
        robot.motorShooter = hardwareMap.dcMotor.get("motorShooter");
        robot.particle_grabber = hardwareMap.dcMotor.get("particle_grabber");


    }

    public void driveForwards( double rightAmount, double leftAmount, double speed) {
        //probably not needed
       /* rightMotorFront.setPower(1.0f);
        rightMotorBack.setPower(1.0f);
        leftMotorFront.setPower(1.0f);
        leftMotorBack.setPower(1.0f); */
        numCalls++;
        telemetry.addData("say", "Drive forwards VROOOM!");


        final int     COUNTS_PER_MOTOR_REV    = 1440 ;
        final int     DRIVE_GEAR_REDUCTION    = 1 ;
        final int     WHEEL_DIAMETER_INCHES   = 4 ;
        final int     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

        final double     DRIVE_SPEED             = 0.6;
        final double     TURN_SPEED              = 0.5;

        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftAmount *-1 * COUNTS_PER_INCH);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightAmount * COUNTS_PER_INCH);
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

    public void driveBackwards( double rightAmount, double leftAmount, double speed) {
        //probably not needed
       /* rightMotorFront.setPower(1.0f);
        rightMotorBack.setPower(1.0f);
        leftMotorFront.setPower(1.0f);
        leftMotorBack.setPower(1.0f); */
        final int COUNTS_PER_MOTOR_REV = 1440;
        final int DRIVE_GEAR_REDUCTION = 1;
        final int WHEEL_DIAMETER_INCHES = 4;
        final int COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

        final double DRIVE_SPEED = 0.6;
        final double TURN_SPEED = 0.5;

        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftAmount * COUNTS_PER_INCH);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightAmount*-1 * COUNTS_PER_INCH);
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


        }



    public void turnLeft(double rightAmount, double leftAmount, double speed) {

        // compare to turnRight method, and make changes accordingly (change doubles to ints)
        final int Counts_Per_Motor_Rev  = 1440;
        final int Drive_Gear_Reduction = 2;
        final int Wheel_Diameter_Inches = 4;
        final int Counts_Per_Inch = (Drive_Gear_Reduction * Counts_Per_Motor_Rev);
        final double Drive_Speed = 0.6;
        final double Turn_Speed = 0.5;

        int newLeftTarget;
        int newRightTarget;
        // get target
        // getCurrentPosition() method below returns an int, so ...
        // rightAmount/leftAmount must have consistent datatypes
        newLeftTarget = robot.leftMotor.getCurrentPosition() +(int) (leftAmount * Counts_Per_Inch);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightAmount * Counts_Per_Inch);
        //inform motor of there target
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);
        telemetry.addData("target: ", newLeftTarget);
        telemetry.addData("targetRight: ", newRightTarget);
        //tell motors to go to the target
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //give motors the means to go to the target
        robot.rightMotor.setPower((double)speed);  // setPower method needs double
        //rightMotorBack.setPower(1.0f);
        robot.leftMotor.setPower((double)speed *-1);

    }

    public void turnRight(double rightAmount, double leftAmount, double speed){

        final int Counts_Per_Motor_Rev  = 1440;
        final int Drive_Gear_Reduction = 2;
//         final double Wheel_Diameter_Inches = 4.0;   // not used?
        final int Counts_Per_Inch = (Drive_Gear_Reduction * Counts_Per_Motor_Rev);
//        final double Drive_Speed = 0.6;  // not used ?
//        final double Turn_Speed = 0.5;   // not used ?

        int newLeftTarget;
        int newRightTarget;
        // get target
        // getCurrentPosition() method below returns an int, so ...
        // rightAmount/leftAmount must have consistent datatypes
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftAmount *-1 * Counts_Per_Inch);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightAmount *-1 * Counts_Per_Inch);
        //inform motor of there target
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.rightMotor.setPower((double)speed *-1);
       // rightMotorBack.setPower(-1.0f);
        robot.leftMotor.setPower((double)speed);

    }






    public void shootParticle(){
        //the code that will do this
    }

    boolean onlyOnce = false;
    private boolean controlVar = true;
     private double whiteLine;
    private int red;
    boolean impossibleVar = false;
    int yellowLight = 0;

    private int v_state=0;
    @Override
    public void loop() {

        if(controlVar) {
            telemetry.addData("say", "loop is running");
          controlVar = false;
        }



        whiteLine = robot.lightSensor.getLightDetected();
        double whiteLine2 = robot.lightSensor2.getLightDetected();

        telemetry.addData("white: ", whiteLine);
        telemetry.addData("white2: ", whiteLine2);
updateTelemetry(telemetry);



        //robot.rightMotor = hardwareMap.dcMotor.get("right_drive");

        //robot.leftMotor = hardwareMap.dcMotor.get("left_drive");

        int testRed;
        int testGreen;
        int testBlue;
        //boolean taskComplete;

        boolean taskComplete = true;

      /*  switch (v_state) {
            case 0:
                if(!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy()){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                v_state = 1;
                break;
            case 1:
                if(robot.leftMotor.getCurrentPosition() > 0 || robot.rightMotor.getCurrentPosition() >0){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                v_state = 2;
                break;
            case 2:
               driveForwards(1.25,1.25,1);
            /*    if(yellowLight < 1000000){
                    yellowLight++;
                }*/
              /*  v_state = 3;
                break;

            case 3:
                if(!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy()){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                v_state = 4;
                break;
            case 4:
                if(robot.leftMotor.getCurrentPosition() > 0 || robot.rightMotor.getCurrentPosition() >0) {
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                this.resetStartTime();

                v_state = 5;
                break;
            case 5:
                robot.motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (robot.motorShooter.getCurrentPosition() > 0) {
                    robot.motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                if (this.getRuntime() > 2.0) {

                    v_state = 6;
                }
                break;
            case 6:
                robot.motorShooter.setTargetPosition(1440);
                robot.motorShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorShooter.setPower(1.0);
                if (robot.motorShooter.getCurrentPosition() >= 1440){
                    robot.motorShooter.setPower(0.0);
                    v_state = 7;
                }

                break;
            case 7:
                if(robot.motorShooter.getCurrentPosition() > 0) {
                    robot.motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                this.resetStartTime();
                v_state = 8;
                break;
            case 8:
                robot.particle_grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.particle_grabber.setPower(-1.0);

                if (this.getRuntime() > 3.0)
                {
                    v_state=9;
                }
                break;
            case 9:
                robot.particle_grabber.setPower(0.0);
                robot.motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                v_state = 10;

                break;
            case 10:
                if (robot.motorShooter.getCurrentPosition() > 0){
                    robot.motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                v_state = 11;
                break;
            case 11:
                robot.motorShooter.setTargetPosition(2880);
                robot.motorShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorShooter.setPower(1.0);
                if (robot.motorShooter.getCurrentPosition() >= 2880){
                    robot.motorShooter.setPower(0.0);
                    v_state = 12;
                }
                break;

          /*  case 12:
                driveForwards(2,2,1);
                v_state = 13;
                break;
            case 13:
                if(!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy()){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                v_state = 14;
                break;
            case 14:
                if(robot.leftMotor.getCurrentPosition() > 0 || robot.rightMotor.getCurrentPosition() >0) {
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                v_state = 15;
                break;

           /* case 9:
                robot.motorShooter.setTargetPosition(1440);
                robot.motorShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorShooter.setPower(1.0);

                v_state = 11;
                break; */





































                // boolean taskComplete;
               /* telemetry.addData("say", "case 0 loop");
                robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(robot.rightMotor.getCurrentPosition() >0 || robot.leftMotor.getCurrentPosition() >0){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if(!onlyOnce) {
                    v_state=1;
                    onlyOnce = true;
                }
                finishedRunning = false;
                taskComplete = false;
                numCalls=0;
                break;*/
            //nothing

          /*  case 1:

                telemetry.addData("yellowLight", yellowLight);
                if(yellowLight < 1000000){
                    yellowLight++;
                }
                //if(impossibleVar)


               /* robot.rightMotor.setPower(0.5*-1);
                robot.leftMotor.setPower(0.5);*/
                //driveBackwards(2,2,1);

               /* v_state = 2;
                break;*/

               /* telemetry.addData("case 1 loop", numCalls);
            //test = robot.colorSensor.red();

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
          //  case 2:
               /* double whiteLine = robot.lightSensor.getLightDetected();
                if(whiteLine >= 0.5)
            {
                robot.rightMotor.setPower(0.0);
                robot.leftMotor.setPower(0.0);
                    /*robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    onlyOnce = true;*/
               /* if(!robot.rightMotor.isBusy()  && !robot.leftMotor.isBusy()){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    yellowLight = 0;
                    v_state = 3;

                }*/
//break;
           /*     case 3:

                    if(robot.rightMotor.getCurrentPosition() >0 || robot.leftMotor.getCurrentPosition() > 0){
                        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    }
                    v_state= 4;
                    break;

            case 4:
                    if(yellowLight < 10000000){
                        yellowLight++;
                    }
                    turnRight(0.4, 0.4, 0.5);

                    v_state= 5;
                break;

            case 5:
                if(!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy()){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    yellowLight = 0;

                }
               v_state =6;
                break;

            case 6:
                if(robot.rightMotor.getCurrentPosition() > 0 || robot.leftMotor.getCurrentPosition() > 0){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



                }
                v_state = 7;
                break;
            case 7:
                if(yellowLight <1000000){
                    yellowLight++;
                }

                driveBackwards(2,2,1);
                v_state = 8;
                break;

            case 8:
                if(!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy()){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    yellowLight = 0;
                }
                v_state = 9;
                break;
            case 9:
                if(robot.rightMotor.getCurrentPosition() > 0 || robot.leftMotor.getCurrentPosition() > 0){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                v_state = 10;
                break;

            case 10:
                if(yellowLight <1000000){

                }
                driveBackwards(1,1,1);
                v_state= 11;
                break;
            case 11:
                if(!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy()){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    yellowLight = 0;
                }
                v_state = 12;
                break;
            case 12:
            if(robot.rightMotor.getCurrentPosition() > 0 || robot.leftMotor.getCurrentPosition() > 0){
                robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            v_state = 13;
            break;
            case 13:
                if(yellowLight < 10000000){
                    yellowLight++;
                }
                turnRight(0.4, 0.4, 0.5);
                v_state = 14;
            case 14:
                if(!robot.rightMotor.isBusy() && !robot.leftMotor.isBusy()){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    yellowLight = 0;
                }
                v_state = 15;
                break;
            case 15:
                if(robot.rightMotor.getCurrentPosition() > 0 || robot.leftMotor.getCurrentPosition() > 0){
                    robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                v_state = 16;
                break;
            case 16:
                telemetry.addData("well done", "Good job");





          /*  case 3:
                telemetry.addData("yellowLight: ", yellowLight);

if (!onlyOnce) {
v_state = 2;

}
if(onlyOnce) {
    turnLeft(1, 1, 3);


}*/
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


  //  telemetry.addData("V_STATE", v_state);

    }
//}

//1: 61
//2: 67
//3: 67
//4: 61
//5: 67