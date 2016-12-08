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
other materials provided with the distribution

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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

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

@TeleOp(name="TELE-OP", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class Mecanum_Drive extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor LBMotor = null;
    DcMotor RBMotor = null;
    DcMotor LFMotor = null;
    DcMotor RFMotor = null;
    DcMotor Launcher = null;


    double X1 = 0.0f;
    double X2 = 0.0f;
    double Y1 = 0.0f;
    double leftx;
    double lefty;
    double rightx;

    double LFPower = 0.0f;
    double RFPower = 0.0f;
    double LBPower = 0.0f;
    double RBPower = 0.0f;

    float deadzone = 0.1f;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        Launcher = hardwareMap.dcMotor.get("Launcher");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // Motors on left have to be going in same direction
        // Motors on right have to be going in opposite direction
        // If controls are reversed (drives backwards instead of forwards,
        // switch FORWARD and REVERSE on left and right sides below.0
        LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.FORWARD );
        gamepad1.setJoystickDeadzone(deadzone);
        gamepad2.setJoystickDeadzone(deadzone);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


            //code for the launcher, which is yet to be attached
            // if(gamepad1.x)
            //launcher.setpower(100);
            //else
            // launcher.setpower(0);




            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            while (opModeIsActive()) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();


                leftx = gamepad1.left_stick_x;
                lefty = gamepad1.left_stick_y;
                rightx = gamepad1.right_stick_x;
/*
                leftx = Range.clip(leftx, -1, 1);
                lefty = Range.clip(lefty, -1, 1);
                rightx = Range.clip(rightx, -1, 1);

                leftx = (float) scaleInput(leftx);
                lefty = (float) scaleInput(lefty);
                rightx = (float) scaleInput(rightx);
*/

                // Scale the input values by squaring them
                int joysign = 1;
                if (leftx < 0) {
                    joysign = -1;
                }
                leftx = joysign * (pow(abs(leftx), 2.0));

                joysign = 1;
                if (lefty < 0) {
                    joysign = -1;
                }
                lefty = joysign * (pow(abs(lefty), 2.0));

                joysign = 1;
                if (rightx < 0.0) {
                    joysign = -1;
                }
                rightx = joysign * (pow(abs(rightx), 2.0));


//                if (abs(gamepad1.left_stick_x) > deadzone) {
//                    X1 = gamepad1.left_stick_x;
//                } else {
//                    X1 = 0;
//                }
//
//                if (abs(gamepad1.left_stick_y) > deadzone) {
//                    Y1 = gamepad1.left_stick_y;
//                } else {
//                    Y1 = 0;
//                }
//
//                if (abs(gamepad1.right_stick_x) > deadzone) {
//                    X2 = gamepad1.right_stick_x;
//                } else {
//                    X2 = 0;
//                }

                // Original semi working code, which is what we are using for the first competition... yeah
                // the two front motors were swaped in the wiring.
//                RFMotor.setPower(-Y1 - X2 + X1);
//                RBMotor.setPower(Y1 - X2 - X1);
//                LFMotor.setPower(-Y1 + X2 - X1);
//                LBMotor.setPower(Y1 + X2 + X1);

                X1 = leftx;
                Y1 = lefty;
                X2 = rightx;

               // Should be working code
                // If this still isn't working, change the direction of each of the left and
                // right motors up above and retest.
                RFPower = Y1 + X2 + X1;
                RBPower = Y1 + X2 - X1;
                LFPower = Y1 - X2 - X1;
                LBPower = Y1 - X2 + X1;
                RFMotor.setPower(RFPower);
                RBMotor.setPower(RBPower);
                LFMotor.setPower(LFPower);
                LBMotor.setPower(LBPower);
                Launcher.setPower(gamepad2.left_stick_y);
            }
        }
    }
}
/*


            double scaleInput(double dVal)  {
                double[] scaleArray = { 0.0, 0.01, 0.04, 0.06, 0.10, 0.15, 0.18, 0.24,
                        0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

                // get the corresponding index for the scaleInput array.
                int index = (int) (dVal * 16.0);

                // index should be positive.
                if (index < 0) {
                    index = -index;
                }

                // index cannot exceed size of array minus 1.
                if (index > 16) {
                    index = 16;
                }

                // get value from the array.
                double dScale = 0.0;
                if (dVal < 0) {
                    dScale = -scaleArray[index];
                } else {
                    dScale = scaleArray[index];
                }

                // return scaled value.
                return dScale;
            }
}
*/

