/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Beep", group="TeleOp")
// @Disabled
public class PushbotTeleopTank_Iterative extends OpMode{




    public void calibrateGyro() {
        ModernRoboticsI2cGyro gyro;   // Hardware Device Object
        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;
        boolean lastResetState = false;
        boolean curResetState = false;

        // get a reference to a Modern Robotics GyroSensor object.
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();
        gyro.resetZAxisIntegrator();
        // make sure the gyro is calibrated.
        /*while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();*/
        }


    long lastTime;
    double Input, Output, Setpoint;
    double errSum, lastErr;
    double kp, ki, kd;


    public void ComputePID() {
        long now = System.currentTimeMillis();
        double timeChange = (double) (now - lastTime);
        double error = Setpoint - Input;
        errSum += (error * timeChange);
        double dErr = (error - lastErr);

        Output = kp * error + ki * errSum + kd * dErr;
        lastErr = error;
        lastTime = now;
    }

    public void SetTunings (double Kp, double Ki, double Kd)
    {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }



    public double turnGyro(float targetHeading) {
        int original_anglez = 0;
        ModernRoboticsI2cGyro gyro;
        int xVal, yVal, zVal = 0;
        int heading = 0;
        int angleZ = 0;
        float MIDPOWER = 0;
        double DRIVEGAIN = 1;
        double TOLERANCE = .1;
        int timer = 0;
        double currentHeading, headingError, driveSteering, leftPower, rightPower, oldCurrentHeading = 0.0;
        long startTime = 0;
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        //calibrateGyro();
        gyro.resetZAxisIntegrator();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //telemetry.addData("Current Pos", currentHeading);
        //updateTelemetry(telemetry);

        startTime = System.currentTimeMillis();
        //currentHeading = gyro.getHeading();
        SetTunings(.01, 0, 0.2);

        Setpoint = targetHeading;
        Input = gyro.getHeading();
        //Input = currentHeading;

        do {

            ComputePID();
            robot.leftMotor.setPower(-Output);
            robot.rightMotor.setPower(Output);
            timer++;
            //sleep(1000);
            Input = gyro.getHeading();
            //sleep(1000);
            telemetry.addData("curHeading", Input);
            telemetry.addData("tarHeading", Setpoint);
            updateTelemetry(telemetry);
            //} while (Input < targetHeading && (System.currentTimeMillis() < (startTime + 6000)));
        } while ((Math.abs(Input - Setpoint) > TOLERANCE)   && (System.currentTimeMillis() < (startTime + 6000)));

        telemetry.addData("curHeading", Input);
        telemetry.addData("tarHeading", Setpoint);
        telemetry.addData("leftPwr", -Output);
        telemetry.addData("rightPwr", Output);
        //telemetry.addData("headingErr", headingError);
        //telemetry.addData("driveSteer", driveSteering);
        //telemetry.addData("DRIVEGAIN", DRIVEGAIN);
        updateTelemetry(telemetry);


        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return Input;
    }

    public double turnGyro2(float targetHeading) {
        int original_anglez = 0;
        ModernRoboticsI2cGyro gyro;
        int xVal, yVal, zVal = 0;
        int heading = 0;
        int angleZ = 0;
        float MIDPOWER = 0;
        double DRIVEGAIN = 1;
        double TOLERANCE = .1;
        int timer = 0;
        double currentHeading, headingError, driveSteering, leftPower, rightPower, oldCurrentHeading = 0.0;
        long startTime = 0;
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        //calibrateGyro();
        gyro.resetZAxisIntegrator();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //telemetry.addData("Current Pos", currentHeading);
        //updateTelemetry(telemetry);

        startTime = System.currentTimeMillis();
        //currentHeading = gyro.getHeading();
        SetTunings(.01, 0, 0.2);

        Setpoint = targetHeading;
        Input = gyro.getHeading();
        //Input = currentHeading;

        do {

            ComputePID();
            robot.leftMotor.setPower(Output);
            robot.rightMotor.setPower(-Output);
            timer++;
            //sleep(1000);
            Input = gyro.getHeading();
            //sleep(1000);
            telemetry.addData("curHeading", Input);
            telemetry.addData("tarHeading", Setpoint);
            updateTelemetry(telemetry);
            //} while (Input < targetHeading && (System.currentTimeMillis() < (startTime + 6000)));
        } while ((Math.abs(Input - Setpoint) > TOLERANCE)   && (System.currentTimeMillis() < (startTime + 6000)));

        telemetry.addData("curHeading", Input);
        telemetry.addData("tarHeading", Setpoint);
        telemetry.addData("leftPwr", -Output);
        telemetry.addData("rightPwr", Output);
        //telemetry.addData("headingErr", headingError);
        //telemetry.addData("driveSteer", driveSteering);
        //telemetry.addData("DRIVEGAIN", DRIVEGAIN);
        updateTelemetry(telemetry);


        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return Input;
    }


    /* Declare OpMode members. */
    private HardwarePushbot robot = new HardwarePushbot();

    ColorSensor colorSensor;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Say", "init start");
        try {
            robot.lifter = hardwareMap.dcMotor.get("lifter");
            robot.rightMotor = hardwareMap.dcMotor.get("right_drive");
            robot.leftMotor = hardwareMap.dcMotor.get("left_drive");
            //motorVacuum = hardwareMap.dcMotor.get("vacuum");
            robot.motorShooter = hardwareMap.dcMotor.get("motorShooter");

            robot.particle_grabber= hardwareMap.dcMotor.get ("particle_grabber");
           // robot.linearSlide = hardwareMap.crservo.get("linearSlide");
            robot.lightSensor = hardwareMap.opticalDistanceSensor.get("oDS");
            robot.lightSensor2 = hardwareMap.opticalDistanceSensor.get("oDS2");
            robot.gyro = hardwareMap.gyroSensor.get("gyro");
            telemetry.addData("Say", "motors ready");

            robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);

            robot.init(hardwareMap);

            telemetry.addData("Say", "robot ready");    //
        } catch (Exception e) {
            telemetry.addData("init loop exception: ", e.getMessage());
            throw e;
        } finally {
            telemetry.addData("Say", "init end");

        }

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Say", "init_loop");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.addData("Say", "start fcn");
        telemetry.addData("Say", "start fcn");
    }

    private boolean halfPower = false;
    private boolean sliding = false;
    private boolean reverse = false;// booleans are always false by default
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    public void loop() {
        telemetry.addData("Say", "loop_DEL");
        telemetry.addData("Start loop", "1");

        //wheels
        float leftStick = gamepad1.left_stick_y;
        float rightStick = gamepad1.right_stick_y;
        telemetry.addData("Start loop", "1.1");

        //halfPower must be = to false
        if (!halfPower) {  // if not false means if true

           if (leftStick > 1.0) {
               leftStick = 1;
           }
           if (leftStick < -1.0) {
               leftStick = -1;
           }
           if (rightStick > 1.0) {
               rightStick = 1;
           }
           if (rightStick < -1.0) {
               rightStick = -1;
           }
       }
        //halfPower = true
        if (halfPower){
            if (leftStick > 0.5f){
                leftStick = 0.5f;
            }
            if (leftStick < -0.5f){
                leftStick = -0.5f;
            }
            if (rightStick > 0.5f){
                rightStick = 0.5f;
            }
            if (rightStick < -0.5f){
                rightStick = -0.5f;
            }
        }
        if(gamepad2.y){
            robot.gyro.getHeading();
        }

        if (gamepad1.right_bumper && !halfPower){
            halfPower = true;
        }
        if (gamepad1.right_bumper && halfPower){
            halfPower = false;
        }

        telemetry.addData("Start loop", "1.2");

        if (null==robot) {
            telemetry.addData("robot is null", "");
        }
        if (null==robot.leftMotor) {
            telemetry.addData("robot left motor is null", "");
        }
        if (null==robot.rightMotor) {
            telemetry.addData("robot rightMotor motor is null", "");
        }
        robot.leftMotor.setPower(leftStick);
        robot.rightMotor.setPower(rightStick);

        telemetry.addData("Start loop", "2");
        telemetry.addData("Start loop", "3");

        float grabberPower = gamepad2.right_trigger;

        if(reverse ==false) {
            robot.particle_grabber.setPower(grabberPower * -1);
        }
        if(reverse ==true){
            robot.particle_grabber.setPower(grabberPower);
        }
        // need to change this so that if NOT a or b, then setpower to 0 (off)

        if(gamepad2.a && reverse == true){
           reverse = false;
        }
        if(gamepad2.a && reverse ==false){  // we don't need a separate button to turn off (after above change).
           reverse = true;
            telemetry.addData("Say", robot.motorShooter.getDeviceName());
        }
            // TODO: fix this:

        telemetry.addData("Start loop", "4");
         float shooterPower = gamepad2.left_trigger;

robot.motorShooter.setPower(shooterPower);
/*
        if (gamepad2.y){
            robot.linearSlide.setPower(1.0);
            telemetry.addData("y registered", sliding);
            sliding = true;
        }
        if (gamepad2.right_stick_button){
            telemetry.addData("y registered", sliding);
            robot.linearSlide.setPower(0.0);
            sliding = false;
        }
        if(gamepad2.left_stick_button){
            robot.linearSlide.setPower(-1.0);
            sliding = true;
        }*/
        //telemetry.addData("linearSlide's power: ", robot.linearSlide.getPower());
        telemetry.addData("left", "%.2f", leftStick);
        telemetry.addData("right", "%.2f", rightStick);

        //DRIVER CONTROLED ENHANSMENTS

        //telemetry.addData("heading: ", "")


        //FROM THIS POINT ON IS NEW. IF NULL POINTER IT IS PROBABLY HERE!!!



        //self streignening
        if(gamepad1.x) {
            float gyroReading = robot.gyro.getHeading();
            float quad1Correction = 0;
            float quad2Correction = 90;
            float quad3Correction = 180;
            float quad4Correction = 360;
            float turnAmount1 = 0;

            boolean calc1Complete = false;
            boolean calc2Complete = false;
            float turnError = 94.5f;
            float gyroError = 5.5f;
            if (robot.gyro.getHeading() > 90 && robot.gyro.getHeading() < 180) {

                turnAmount1 =  (gyroReading - gyroError)- (quad2Correction - gyroError);
                calc1Complete = true;
            } else if (robot.gyro.getHeading() > 180 && robot.gyro.getHeading() > 270) {
                turnAmount1 = (gyroReading - gyroError) - (quad3Correction - gyroError);
                calc1Complete = true;
            } else if (robot.gyro.getHeading() > 270 && robot.gyro.getHeading() < 360) {
                turnAmount1 = (gyroReading - gyroError) - (quad4Correction - gyroError);
                calc1Complete = true;
            } else if (robot.gyro.getHeading() > 0 && robot.gyro.getHeading() < 90) {
                turnAmount1 = (gyroReading - gyroError)- (quad1Correction - gyroError);
                calc1Complete = true;
            } else {
                calc1Complete = false;
            }
            if (calc1Complete) {
                turnGyro2(turnAmount1);
            }
        }

        if(gamepad1.y) {
            float gyroReading = robot.gyro.getHeading();
            float quad1Correction = 0;
            float quad2Correction = 180;
            float quad3Correction = 270;
            float quad4Correction = 360;
            float turnAmount1 = 0;
            float turnAmount2 = 0;
            boolean calc1Complete = false;
            boolean calc2Complete = false;
            float turnError = 94.5f;
            float gyroError = 5.5f;
            if (robot.gyro.getHeading() < 90 && robot.gyro.getHeading() > 0) {
                //do nothing
                turnAmount1 =  quad1Correction - (gyroReading - gyroError);
                calc1Complete = true;
            } else if (robot.gyro.getHeading() < 180 && robot.gyro.getHeading() > 90) {
                turnAmount1 = quad2Correction - (gyroReading -gyroError);
                calc1Complete = true;
            } else if (robot.gyro.getHeading() < 270 && robot.gyro.getHeading() > 180) {
                turnAmount1 = quad3Correction - (gyroReading - gyroError);
                calc1Complete = true;
            } else if (robot.gyro.getHeading() < 360 && robot.gyro.getHeading() > 270) {
                turnAmount1 = quad4Correction - (gyroReading -gyroError);
                calc1Complete = true;
            } else {
                calc1Complete = false;
            }

            if(calc1Complete){
                turnAmount2 = turnError - turnAmount1;
                calc2Complete = true;
            }

            if (calc2Complete) {
                turnGyro(turnAmount2);
            }
        }

//lifter
        float liftingStick = gamepad2.right_stick_y;
        if (liftingStick > 1.0) {
            liftingStick = 1;
        }
        if (liftingStick < -1.0) {
            liftingStick = -1;
        }

        robot.lifter.setPower(liftingStick);

//gyro telemetry
        telemetry.addData("gyro Reading: ", robot.gyro.getHeading());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
