package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
@Autonomous (name = "little_turtle_Blue_1", group = "Autonomous OpMode")
public class little_turtle_Blue_1 extends OpMode{
    HardwarePushbot robot   = new HardwarePushbot();
    static double timer=0;
    //public DcMotor leftMotor;
    //public DcMotor leftMotor;
  // public DcMotor rightMotor;
   //public DcMotor rightMotor;
    public DcMotor vacuumMotor;
    public DcMotor elevatorMotor;
   public DcMotor shooterMotor;
    //HardwarePushbot robot   = new HardwarePushbot();
    //static double timer =0;
    DcMotor rightMotor;
    // DcMotor rightMotor;
   // DcMotor leftMotor;
    DcMotor leftMotor;
    public boolean finishedRunning = false;
    ColorSensor colorSensor;
    Servo elevatorVacuum;

    public void driveForwards( double rightAmount, double leftAmount)
    {
        //probably not needed
       /* rightMotor.setPower(1.0f);
        rightMotor.setPower(1.0f);
        leftMotor.setPower(1.0f);
        leftMotor.setPower(1.0f); */
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
       // robot.leftMotor2.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);
        robot.rightMotor2.setTargetPosition(newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setPower(1.0f);
       // robot.leftMotor2.setPower(1.0f);
        robot.rightMotor.setPower(1.0f);
        robot.rightMotor2.setPower (1.0f);

        if(leftMotor.isBusy() && leftMotor.isBusy() && rightMotor.isBusy()){
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            finishedRunning = true;
            //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }}
    public void driveBackwards( double rightAmount, double leftAmount) {//THESE VALUES HAVE NOT BEEN CHANGED
        //probably not needed
       /* rightMotor.setPower(1.0f);
        rightMotor.setPower(1.0f);
        leftMotor.setPower(1.0f);
        leftMotor.setPower(1.0f); */
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
        //robot.leftMotor2.setTargetPosition(newLeftTarget);
        robot.rightMotor.setTargetPosition(newRightTarget);
        robot.rightMotor2.setTargetPosition(newRightTarget);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftMotor.setPower(-1.0f);
       // robot.leftMotor2.setPower(-1.0f);
        robot.rightMotor.setPower(-1.0f);
        robot.rightMotor2.setPower(-1.0f);

        if (leftMotor.isBusy() && leftMotor.isBusy() && rightMotor.isBusy()) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            finishedRunning =true;
            //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

       /* rightMotor.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.RunMode(DcMotor.RunMode.RUN_USING_ENCODER); */




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
       // robot.leftMotor2.setTargetPosition(newLeftTarget);
        robot.rightMotor2.setTargetPosition(newRightTarget);
        //tell motors to go to the target
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       // robot.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //give motors the means to go to the target
        rightMotor.setPower(1.0f);
        //rightMotor.setPower(1.0f);
        leftMotor.setPower(-1.0f);
        leftMotor.setPower(-1.0f);

        if(leftMotor.isBusy() && leftMotor.isBusy() && rightMotor.isBusy())
        {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            finishedRunning = true;
            //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



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
       // robot.leftMotor2.setTargetPosition(newLeftTarget);
        robot.rightMotor2.setTargetPosition(newRightTarget);
        //tell motors to go to the target
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //  robot.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //give motors the means to go to the target


        rightMotor.setPower(-1.0f);
        // rightMotor.setPower(-1.0f);
        leftMotor.setPower(1.0f);
        leftMotor.setPower(1.0f);

        if(leftMotor.isBusy() && leftMotor.isBusy() && rightMotor.isBusy())
        {
            //JUST IN CASE WE NEED IT oops all caps`
            /*robot.leftMotor.setPower(0.0f);
            robot.rightMotor.setPower(0.0f);
            robot.leftMotor2.setPower(0.0f);
            robot.rightMotor2.setPower(0.0f);*/
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            finishedRunning = true;
            // rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        }



    }





    public void runOpMode() throws InterruptedException{
        leftMotor = hardwareMap.dcMotor.get("leftMotor1");
        leftMotor = hardwareMap.dcMotor.get("leftMotor2");
        rightMotor = hardwareMap.dcMotor.get("rightMotor1");
        rightMotor = hardwareMap.dcMotor.get("rightMotor2");
        vacuumMotor = hardwareMap.dcMotor.get("");
        elevatorMotor = hardwareMap.dcMotor.get("");
        shooterMotor = hardwareMap.dcMotor.get("");


//EDIT THESE VALUES TO MAKE THE PROPER STRENGTH
    }


    public void driveForwards(float amount)
    {
        rightMotor.setPower(1.0f);
        rightMotor.setPower(1.0f);
        leftMotor.setPower(1.0f);
        leftMotor.setPower(1.0f);

       /* rightMotor.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        new DcMotor.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
         leftMotor.RunMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/


        //if (){

    }
    public void init(){
//EDIT THESE VALUES TO MAKE THE PROPER STRENGTH

} public void turnLeft(float Power)
    {
        rightMotor.setPower(Power);
        rightMotor.setPower(Power);

        leftMotor.setPower(-Power);
        leftMotor.setPower(-Power);
    }

    public void turnRight(float Power){
        rightMotor.setPower(-Power);
        rightMotor.setPower(-Power);
        leftMotor.setPower(Power);
        leftMotor.setPower(Power);
    }
    public void driveBackwards(float Power)
    {
        rightMotor.setPower(-Power);
        rightMotor.setPower(-Power);
        leftMotor.setPower(-Power);
        leftMotor.setPower(-Power);
    }
    public void stopDrive()
    {
        rightMotor.setPower(0.0f);
        rightMotor.setPower(0.0f);
        leftMotor.setPower(0.0f);
        leftMotor.setPower(0.0f);
    }
    public void DriveForwards(float Power)
    {
        rightMotor.setPower(Power);
        rightMotor.setPower(Power);
        leftMotor.setPower(Power);
        leftMotor.setPower(Power);
    }
    public void shootParticle(){
        //the code that will do this
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

            driveForwards(1.0f, 1.0f);
            if (finishedRunning == true){
                v_state++;
            }
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
           driveForwards(0.9f, 1.0f);

            timer++;
            }stopDrive();
            v_state=6;
            break;
        case 6:
            if (timer >= 30){
                leftMotor.setPower(1.0f);
                leftMotor.setPower(1.0f);
                rightMotor.setPower(-1.0f);
                rightMotor.setPower(-1.0f);

                timer++;
            }leftMotor.setPower(0.0f);
            leftMotor.setPower(0.0f);
            rightMotor.setPower(0.0f);
            rightMotor.setPower(0.0f);
            v_state = 8;
            break;
        case 8:
            if (timer >= 30){
                leftMotor.setPower(1.0f);
                leftMotor.setPower(1.0f);
                rightMotor.setPower(1.0f);
                rightMotor.setPower(1.0f);

                timer++;
            }leftMotor.setPower(0.0f);
            leftMotor.setPower(0.0f);
            rightMotor.setPower(0.0f);
            rightMotor.setPower(0.0f);
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
//TURTLES ARE ABSOLUTLY AWESOME BECAUSE THEY NUKE WHALES!
//dooo dooo do do do doo dooo doooo do do do doo dooo doooo do do do do dododo do do do dododo