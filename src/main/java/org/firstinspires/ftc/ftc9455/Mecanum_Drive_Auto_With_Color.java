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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="Autonomous_In_Progress", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class Mecanum_Drive_Auto_With_Color extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = -2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    DcMotor LBMotor = null;
    DcMotor RBMotor = null;
    DcMotor LFMotor = null;
    DcMotor RFMotor = null;

    ColorSensor colorSensor;

    /*
    1in: Red: 5     Blue: 7-8
    2in: Red: 4     Blue: 6
     */

    OpticalDistanceSensor odsSensor;
/*
1in: Black: .15 to .2     Line: .8 to .9
2in: Black: .07 to .1     Line: .3 to .35
3in: Black: .06 to .07    Line: .1 to .2
 */
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
        LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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


        double correction, leftpower, rightpower;
        final double PCV = 0.2;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = false;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("color");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //sleep(10000); // Wait 10 seconds

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // run until the end of the match (driver presses STOP)

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        //Add in a 10 second delay

        //sleep(10000); //ten second delay to avoid illegaly crossing the line
        //encoderDrive(DRIVE_SPEED, 60, 60, 5.0, 60, 60);  // S1: Forward 55 Inches with 5 Sec timeout
        // 30 over
        // move to line
        //
        //sleep(100);
        //encoderDrive(DRIVE_SPEED, -20, 20, 5.0, 20, -20);
        //sleep(100);
        while (opModeIsActive() && (odsSensor.getRawLightDetected() < 0.2)) {
            LFMotor.setPower(-10);
            LBMotor.setPower(-10);
            RFMotor.setPower(-10);
            RBMotor.setPower(-10);
        }
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);

        sleep(100);

        while (opModeIsActive() && (odsSensor.getRawLightDetected() < 0.2)) {

            LFMotor.setPower(10);
            LBMotor.setPower(10);
            RFMotor.setPower(-10);
            RBMotor.setPower(-10);
        }
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);

        sleep(3000);

            while (opModeIsActive() && (colorSensor.alpha() < 2)) {

                correction = (PCV - odsSensor.getLightDetected());

                if (correction <= 0){
                    leftpower = .075d - correction;
                    rightpower = .075d;
                } else {
                    leftpower = .075d;
                    rightpower = .075d + correction;
                }
                LFMotor.setPower(leftpower);
                LBMotor.setPower(leftpower);
                RBMotor.setPower(rightpower);
                RFMotor.setPower(rightpower);
            }

        }



    public void encoderDrive(double speed,
    double leftFInches, double rightFInches,
    double timeoutS, double leftBInches, double rightBInches) {

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