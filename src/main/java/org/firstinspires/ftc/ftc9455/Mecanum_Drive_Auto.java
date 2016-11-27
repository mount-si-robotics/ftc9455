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
package org.firstinspires.ftc.ftc9455;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="CAP-BALL-AUTO", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class Mecanum_Drive_Auto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = -1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    DcMotor LBMotor = null;
    DcMotor RBMotor = null;
    DcMotor LFMotor = null;
    DcMotor RFMotor = null;

    float LFPower = 0.0f;
    float RFPower = 0.0f;
    float LBPower = 0.0f;
    float RBPower = 0.0f;

    @Override
    public void runOpMode() {

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        LFMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                LFMotor.getCurrentPosition(),
                RFMotor.getCurrentPosition(),
                RBMotor.getCurrentPosition(),
                LBMotor.getCurrentPosition());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


            sleep(10000); //ten second delay to avoid illegaly crossing the line
            encoderDrive(DRIVE_SPEED, 55, 55, 55, 55, 5.0);  // S1: Forward 55 Inches with 5 Sec timeout

            telemetry.addData("Path", "Complete"); //info for the drivers
            telemetry.update();
        }


        //This is the fun stuff that defines encoderDrive, lifted from the sample code with some extras for the mecanum drive
            public void encoderDrive(double speed,
            double leftFInches, double rightFInches,
            double leftBInches, double rightBInches, double timeoutS) {

            int newLeftFrontTarget;
            int newRightFrontTarget;
            int newLeftBackTarget;
            int newRightBackTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = LFMotor.getCurrentPosition() + (int) (leftFInches * COUNTS_PER_INCH);
                newRightFrontTarget = RFMotor.getCurrentPosition() + (int) (rightFInches * COUNTS_PER_INCH);
                newLeftBackTarget = LBMotor.getCurrentPosition() + (int) (leftBInches * COUNTS_PER_INCH);
                newRightBackTarget = RBMotor.getCurrentPosition() + (int) (rightBInches * COUNTS_PER_INCH);
                LFMotor.setTargetPosition(newLeftFrontTarget);
                RFMotor.setTargetPosition(newRightFrontTarget);
                LBMotor.setTargetPosition(newLeftBackTarget);
                RBMotor.setTargetPosition(newRightBackTarget);

                // Turn On RUN_TO_POSITION
                LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                LFMotor.setPower(Math.abs(speed));
                RFMotor.setPower(Math.abs(speed));
                LBMotor.setPower(Math.abs(speed));
                RBMotor.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (LFMotor.isBusy() && RFMotor.isBusy()) && (LBMotor.isBusy()) && (RBMotor.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            LFMotor.getCurrentPosition(),
                            RFMotor.getCurrentPosition(),
                            LBMotor.getCurrentPosition(),
                            RBMotor.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                LFMotor.setPower(0);
                RFMotor.setPower(0);
                LBMotor.setPower(0);
                RBMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move

                }
            }
        }