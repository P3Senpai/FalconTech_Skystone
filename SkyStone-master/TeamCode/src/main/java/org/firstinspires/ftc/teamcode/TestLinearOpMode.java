/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test: Linear OpMode", group="Linear Opmode")
//@Disabled
public class TestLinearOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareBot bot = new HardwareBot();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        bot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            motorTest(gamepad1);
            servoTest(gamepad2);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
// uses a, and y buttons + buttons used in driveTest
    private void motorTest(Gamepad gp){
        if (gp.a){
            bot.leftLift.setPower(0.4);
            bot.rightLift.setPower(0.4);
        }else if(gp.y){
            bot.leftLift.setPower(-0.4);
            bot.rightLift.setPower(-0.4);
        }else{
            bot.leftLift.setPower(0);
            bot.rightLift.setPower(0);
        }

        driveTest(gp);
    }
//uses left and right stick
    private void driveTest(Gamepad gp){
        double drive = -gp.left_stick_y;
        double turn = gp.left_stick_x;
        double leftPower = Range.clip(drive - turn, -1,1);
        double rightPower = Range.clip(drive + turn, -1,1);
        double strafePower = Range.clip(gp.right_stick_x, -1,1);

        bot.leftDrive.setPower(leftPower);
        bot.rightDrive.setPower(rightPower);
        bot.strafeDrive.setPower(strafePower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

// uses left/right bumpers, triggers and dpad
    private void servoTest(Gamepad gp){
        double leftFoundationPos = bot.leftMoveFoundation.getPosition();
        double rightFoundtaionPos = bot.rightMoveFoundation.getPosition();
        double grabberPos = bot.grabber.getPosition();

    // left foundation servo position checker
        if (gp.left_bumper){
            bot.leftMoveFoundation.setPosition(leftFoundationPos + 0.01);
        }else if(gp.right_bumper){
            bot.leftMoveFoundation.setPosition(leftFoundationPos - 0.01);
        }
    // right foundation servo position checker
        if(gp.left_trigger != 0){
            bot.rightMoveFoundation.setPosition(rightFoundtaionPos + 0.01);
        }else if(gp.right_trigger != 0){
            bot.rightMoveFoundation.setPosition(rightFoundtaionPos - 0.01);
        }
    // grabber servo position checker
        if (gp.dpad_left){
            bot.grabber.setPosition(grabberPos + 0.01);
        }else if (gp.dpad_right){
            bot.grabber.setPosition(grabberPos - 0.01);
        }

    }
    private void driveByVelocity(double inputData, double maxPower, double velocityForward, double velocitySideways){
        // Set up variables
        double power, leftPower, rightPower, forwardV, sidewaysV, threshold, powerPercentWeight, sidewaysPercentWeight;
        maxPower = Math.abs(maxPower);
        threshold = maxPower * 1.5;     // todo TEST if threshold value is good (since it is random)
        powerPercentWeight = 0.95;      // todo TEST if weight strong enough
        sidewaysPercentWeight = 1 - powerPercentWeight;

        // Set range for values
        power = Range.clip(inputData, -maxPower, maxPower);
        forwardV = Range.clip(velocityForward, -maxPower, maxPower);
        sidewaysV = Range.clip(velocitySideways, -maxPower*sidewaysPercentWeight, maxPower*sidewaysPercentWeight);

        // limit power if predicted drastic changes in magnitude
        if (Math.abs(forwardV - power) > threshold){
            power *= 0.3;
        }else{
            power *= powerPercentWeight;
        }
        // positive axis is left and negative is right
        leftPower = power + sidewaysV;  // sideways velocity prevents drifting
        rightPower = power - sidewaysV; // sideways velocity prevents drifting
        // Set motor speeds
        bot.leftDrive.setPower(leftPower);
        bot.rightDrive.setPower(rightPower);
    }
    // uses x and y velocity to adjust turning in order to center it
    private void turnByVelocity(double inputData, double maxPower, int turn, double xAxisV, double yAxisV) {
        // Define key variables
        double power, errorX, errorY, leftPower, rightPower, powerSignificance, errorSignificance, error, errorMin, errorMax;
        maxPower = Math.abs(maxPower);
        powerSignificance = 0.9;    // % weight on the power value
        errorSignificance = (1 - powerSignificance);  // % weight on error value
        errorMin = (-maxPower * errorSignificance) / 2;   // halves weight so that zero is center
        errorMax = (maxPower * errorSignificance) / 2;    // halves weight so that zero is center
        // Set value ranges
        power = Range.clip(inputData, 0, maxPower * powerSignificance);
        errorX = Range.clip(xAxisV, errorMin, errorMax);
        errorY = Range.clip(yAxisV, errorMin, errorMax);
        // main calculation that centers robot while turning
        error = errorX + errorY * turn;
        leftPower = power - error;  // as if error > 0 then leftPowerOutput > rightPowerOutput
        rightPower = power + error; // as if error < 0 then leftPowerOutput < rightPowerOutput
        // set drive power
        bot.leftDrive.setPower(-leftPower * turn);    // turns left on default
        bot.rightDrive.setPower(rightPower * turn);
    }
}
