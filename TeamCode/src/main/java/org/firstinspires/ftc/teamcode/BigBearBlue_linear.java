package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


/**
 * Created by vasudevfamily on 10/27/16.
 */

@Autonomous(name = "BigBearBlue", group = "LinearOpMode")

public class BigBearBlue_linear extends LinearOpMode {
boolean targetColor = true;


    HardwarePushbot robot = new HardwarePushbot();


    public void driveForwards(double rightAmount, double leftAmount, double speed) {

        telemetry.addData("say", "Drive forwards VROOOM!");


        final int COUNTS_PER_MOTOR_REV = 1440;
        final int DRIVE_GEAR_REDUCTION = 1;
        final int WHEEL_DIAMETER_INCHES = 4;
        final int COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

        final double DRIVE_SPEED = 0.6;
        final double TURN_SPEED = 0.5;

        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftAmount *-1 * COUNTS_PER_INCH);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightAmount *-1 * COUNTS_PER_INCH);
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);
        telemetry.addData("say:", newLeftTarget);
        telemetry.addData("say:", newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);
    }

    public void driveBackwards(double rightAmount, double leftAmount, double speed) {
        final int COUNTS_PER_MOTOR_REV = 1440;
        final int DRIVE_GEAR_REDUCTION = 1;
        final int WHEEL_DIAMETER_INCHES = 4;
        final int COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

        final double DRIVE_SPEED = 0.6;
        final double TURN_SPEED = 0.5;

        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftAmount * COUNTS_PER_INCH);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightAmount * COUNTS_PER_INCH);
        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);


        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.leftMotor.setPower(speed * -1);
        robot.rightMotor.setPower(speed * -1);

    }


    public void turnLeft(double rightAmount, double leftAmount, double speed) {

        final int Counts_Per_Motor_Rev = 1440;
        final int Drive_Gear_Reduction = 2;
        final int Wheel_Diameter_Inches = 4;
        final int Counts_Per_Inch = (Drive_Gear_Reduction * Counts_Per_Motor_Rev);
        final double Drive_Speed = 0.6;
        final double Turn_Speed = 0.5;

        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftAmount * -1 * Counts_Per_Inch);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightAmount * Counts_Per_Inch);

        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);
        telemetry.addData("target: ", newLeftTarget);
        telemetry.addData("targetRight: ", newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.rightMotor.setPower((double) speed);

        robot.leftMotor.setPower((double) speed);

    }

    public void turnRight(double rightAmount, double leftAmount, double speed) {

        final int Counts_Per_Motor_Rev = 1440;
        final int Drive_Gear_Reduction = 2;

        final int Counts_Per_Inch = (Drive_Gear_Reduction * Counts_Per_Motor_Rev);


        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftAmount * 1 * Counts_Per_Inch);
        newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightAmount * -1 * Counts_Per_Inch);

        robot.leftMotor.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.rightMotor.setPower((double) speed * -1);

        robot.leftMotor.setPower((double) speed);

    }

    public void stopMoving(){

        robot.rightMotor.setPower(0.0);
        robot.leftMotor.setPower(0.0);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void driveUntilLine(){
        double whiteValue = robot.lightSensor.getLightDetected();
        telemetry.addData("ODS reads: ", whiteValue);
        while (opModeIsActive() && whiteValue < 0.8) {
            robot.rightMotor.setPower(1.0);
            robot.leftMotor.setPower(1.0);
        }

        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);



    }
    public void driveUntilLineReverse(){
        double whiteValue = robot.lightSensor.getLightDetected();
        double whiteValue2 = robot.lightSensor2.getLightDetected();
        telemetry.addData("ODS reads: ", whiteValue);
        while (opModeIsActive() && whiteValue < 0.8) {
            robot.rightMotor.setPower(-.30);
            robot.leftMotor.setPower(-.30);
        }

        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);



    }
public void squareOnLineReverse(){
    double whiteLine = robot.lightSensor.getLightDetected();
    double whiteLine2 = robot.lightSensor2.getLightDetected();

    while(whiteLine < 0.8 && whiteLine2 < 0.8 ){
        telemetry.addData("WhiteLine=: ", whiteLine);
        telemetry.addData("WhiteLine2=: ", whiteLine2);
        if(whiteLine < 0.8 && whiteLine2 < 0.8) {
            robot.rightMotor.setPower(-1.0);
            robot.leftMotor.setPower(-1.0);
        }
        if(whiteLine < 0.8 && whiteLine2 >= 0.8){
            robot.rightMotor.setPower(-1.0);
        }
        if(whiteLine >= 0.8 && whiteLine2 < 0.8){
            robot.leftMotor.setPower(-1.0);
        }
        robot.rightMotor.setPower(0.0);
        robot.leftMotor.setPower(0.0);
    }







}
    public void squareOnLine(){
        double white = 0;
        double white2 = 0;


        while( white < 0.8 && white2 <0.8){
             white = robot.lightSensor.getLightDetected();
             white2 = robot.lightSensor2.getLightDetected();
            robot.leftMotor.setPower(0.30);
            robot.rightMotor.setPower(0.30);
        }


          /*  while(white2 <0.8){
                 white = robot.lightSensor.getLightDetected();
                 white2 = robot.lightSensor2.getLightDetected();
                robot.rightMotor.setPower(0.30);
            }
*/

        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);




        }
public void squareOnLine2(){
    double white = 0;
    double white2 = 0;
    boolean stillGoing = true;
    boolean right = true;
    boolean left = true;

    telemetry.addData("checkpoint1",stillGoing);
    updateTelemetry(telemetry);
    robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    while(left || right) {
        telemetry.addData("checkpoint2","!");
        updateTelemetry(telemetry);


        telemetry.addData("right: ", right);
        telemetry.addData("left: ", left);
        telemetry.addData("white: ", white);
        telemetry.addData("white2: ", white2);
        updateTelemetry(telemetry);



       if(right){
           robot.rightMotor.setPower(-0.10);
           telemetry.addData("Moving R","");
           updateTelemetry(telemetry);
       }
       if(left){
           robot.leftMotor.setPower(-0.10);
           telemetry.addData("Moving L","");
           updateTelemetry(telemetry);
       }
        white = robot.lightSensor.getLightDetected();
        white2 = robot.lightSensor2.getLightDetected();
        if(white >.1){
            robot.leftMotor.setPower(0.0);
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left = false;
             telemetry.addData("checkpoint3","!");
            updateTelemetry(telemetry); 
        }
        if( white2 > .1){
            telemetry.addData("checkpoint4","!");
            updateTelemetry(telemetry);
            robot.rightMotor.setPower(0.0);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right = false;
        }
        sleep(100);
        if(!left && !right){
            stillGoing = false;
             telemetry.addData("checkpoint5","!");
            updateTelemetry(telemetry);
        }



    }



}




    public void getBeaconColor() {
        double red = robot.colorSensor.red();
        double blue = robot.colorSensor.blue();
        telemetry.addData("color sensor reads red as: ", red);
        telemetry.addData("color sensor reads blue as:", blue);

        while (red < 2 || blue < 2) {

            robot.linearSlide.setPower(1.0);
        }
        robot.linearSlide.setPower(0.0);
        targetColor = false;
        if (red >2)
        {targetColor = true;}
    }

    public void linearSlide(){
        if(targetColor == false){
            driveForwards(0.2, 0.2, 0.9);
            sleep(500);

            stopMoving();
            sleep(500);

            targetColor =true;
        }
        if (targetColor == true){
            robot.linearSlide.setPower(1.0);
            sleep(500);
        }
    }

    public void linearSlideReverse(){
        robot.linearSlide.setPower(-1.0);
        sleep(1000);
        robot.linearSlide.setPower(0.0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        int i = 0;
        robot.init(hardwareMap);

        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("ready to go", "all is ready");

        /*while(i<1000000) {
            telemetry.addData("light1=", robot.lightSensor.getLightDetected());
            telemetry.addData("light2=", robot.lightSensor.getLightDetected());
            updateTelemetry(telemetry);
            i++;
        }*/



        waitForStart();




       driveForwards(2,2,2);
        sleep(1500);


        turnLeft(0.3,0.3, 0.5);
        sleep(600);

        driveForwards(10.0, 10.0, 0.3);
        sleep(3500);

        driveForwards(3.0, 3.0, 0.1);
        sleep(1000);

        driveBackwards(0.3, 0.3, 0.8);
        sleep(500);

        turnRight(0.35, 0.35, 0.35);
        sleep(500);
        telemetry.addData("got here", "!");
        updateTelemetry(telemetry);

        squareOnLine2();
        sleep(2000);

        telemetry.addData("here too","!");
        updateTelemetry(telemetry);
        sleep(10000);




/*


        getBeaconColor();
        sleep(500);

        linearSlide();
        sleep(1000);

        linearSlideReverse();
        driveUntilLine();
        sleep(500);

        stopMoving();
        sleep(500);

        getBeaconColor();
        sleep(500);

        linearSlide();
        sleep(500);

        turnLeft(0.6, 0.6, 0.4);
        sleep(500);

        driveForwards(1.0, 1.0, 1.0);
        sleep(500);

        robot.motorShooter.setPower(1.0);
        sleep(1000);

        robot.motorShooter.setPower(0.0);
        robot.particle_grabber.setPower(-1);
        sleep(500);
*/













    }
}
























