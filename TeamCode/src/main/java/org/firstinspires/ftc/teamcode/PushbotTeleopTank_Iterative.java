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

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.robot.Robot;
//import com.qualcomm.robotcore.util.Range;

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

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();
    DcMotor motorRight;
    DcMotor motorLeft;
//    DcMotor motorShooter;
//    DcMotor motorElevator;
//    DcMotor motorVacuum;
    DcMotor motorRight2;
    DcMotor motorLeft2;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        motorRight = hardwareMap.dcMotor.get("right_drive");
        motorLeft = hardwareMap.dcMotor.get("left_drive");
        //motorVacuum = hardwareMap.dcMotor.get("vacuum");
        //motorShooter = hardwareMap.dcMotor.get("shooter");
        //motorElevator = hardwareMap.dcMotor.get("elevator");
        motorRight2 = hardwareMap.dcMotor.get("right_drive2");
        motorLeft2 = hardwareMap.dcMotor.get("left_drive2");
        telemetry.addData("Say", "motors ready");    //

        motorRight.setDirection(DcMotor.Direction.REVERSE);

        robot.init(hardwareMap);

        telemetry.addData("Say", "robot ready");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }


    boolean vacuumOn = false;
    float vacuumDirection = 0.0f;
    boolean elevatorOn = false;
    float elevatorDirection = 0.0f;
    boolean emergencyCode = false;
    int timer = 0;
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loop() {

            telemetry.addData("Start loop", "1");

            //wheels
            float leftStick = -gamepad1.left_stick_y;
            float rightStick = -gamepad1.right_stick_y;
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

            robot.leftMotor.setPower(leftStick);
            robot.leftMotor2.setPower(leftStick);
            robot.rightMotor.setPower(rightStick);
          //  robot.rightMotor2.setPower(rightStick); // is null
               // is null

            telemetry.addData("Start loop", "2");



            telemetry.addData("Start loop", "3");

            //vacuum
            //!vacuumOn is the same as vacuumOn ==false
            if (gamepad1.right_bumper && !vacuumOn) {
                vacuumOn = true;
            }
            if (gamepad1.right_bumper && vacuumOn) {
                vacuumOn = false;
            }
            if (vacuumOn && !emergencyCode) {
                vacuumDirection = 1.0f;
            }
            if (!vacuumOn && !emergencyCode) {
                vacuumDirection = 0.0f;
            }

            // TODO: fix this:
            // robot.motorVacuum.setPower(vacuumDirection);

            //shooter
            telemetry.addData("Start loop", "4");

            if (gamepad2.left_bumper) {
//            robot.motorShooter.setPower(1.0f);
                timer++;
                if (timer == 30) {
                    telemetry.addData("say", "Timer is finished");
//                robot.motorShooter.setPower(0.0f);

                    timer = 0;

                }

            }

            //elevator
//
//        if(gamepad1.left_bumper && !elevatorOn)
//        {
//            elevatorOn=true;
//        }
//        if(gamepad1.left_bumper && elevatorOn)
//        {
//            elevatorOn = false;
//        }
//        if(elevatorOn)
//        {
//            elevatorDirection = 1.0f;
//        }
//        if(!elevatorOn)
//        {
//            elevatorDirection = 0.0f;
//        }
//        robot.motorElevator.setPower(elevatorDirection);

            //reversing the elvator and vacuum if we grab the wrong ball

/*
        if(gamepad1.a && !emergencyCode)
        {
            emergencyCode = true;
        }
        if(gamepad1.a && emergencyCode)
        {
            emergencyCode =false;
        }

        if(emergencyCode)
        {
            elevatorDirection = -1.0f;
            vacuumDirection = -1.0f;

        }
*/


            // Use gamepad buttons to move the arm up (Y) and down (A)


            // Send telemetry message to signify robot running;

            telemetry.addData("left", "%.2f", leftStick);
            telemetry.addData("right", "%.2f", rightStick);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
