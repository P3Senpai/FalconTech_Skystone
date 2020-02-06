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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="!Test: Linear OpMode", group="Linear Opmode")
@Disabled
public class TeleOpTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Toggle tgg = new Toggle();
    private HardwareBot bot = new HardwareBot();
    private double leftFrontVelocity, leftBackVelocity, rightFrontVelocity, rightBackVelocity;
    private int timer;

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
            servoTest(gamepad1);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            tgg.reset();
        }
    }

    // uses dpad up and down buttons used + buttons used in driveTest
    private void motorTest(Gamepad gp){
        lift(gp);
        driveTest(gp);
        if(gp.a)
            bot.shooter.setPower(0.5);
        else
            bot.shooter.setPower(0);
    }
    // uses dpad up and down
    private void lift(Gamepad gp){
        if (gp.dpad_up){
            bot.leftLift.setPower(0.75 * 0.7); // at 70% for home testing
            bot.rightLift.setPower(0.75 * 0.7);
        }else if(gp.dpad_down){
            bot.leftLift.setPower(-0.75 * 0.7);
            bot.rightLift.setPower(-0.75 * 0.7);
        }else{
            bot.leftLift.setPower(0);
            bot.rightLift.setPower(0);
        }

        telemetry.addData("Lift ticks", "left: %.0f right: %.0f",(double)bot.leftLift.getCurrentPosition(),(double)bot.rightLift.getCurrentPosition());
    }

    //uses left and right stick and both triggers
    private void driveTest(Gamepad gp){
        double drive = -gp.left_stick_y;
        double turn = gp.right_stick_x;
        double strafe = (gp.left_trigger > gp.right_trigger)? -gp.left_trigger: gp.right_trigger; // left trigger is between -1 and 0 right trigger is between 0 and 1

        double leftFrontPower = Range.clip((drive * 0.8065) + strafe + turn, -1, 1); // average velocity 2202 (1721.0/2202) +0.025
        double leftBackPower = Range.clip((drive * 0.83509) - strafe + turn, -1,1); // average velocity 2114 (1721.0/2114)  +0.025
        double rightFrontPower = Range.clip((drive * 0.96848) - strafe - turn, -1,1); // average velocity 1777 (1721.0/1777)
        double rightBackPower = Range.clip((drive) + strafe - turn, -1,1); // average velocity 1721 (smallest value)

        bot.leftFrontDrive.setPower(leftFrontPower * 0.7); // at 70% for home testing
        bot.leftBackDrive.setPower(leftBackPower * 0.7);
        bot.rightFrontDrive.setPower(rightFrontPower* 0.7);
        bot.rightBackDrive.setPower(rightBackPower* 0.7);
//todo add sprint
        // gets data for velocity average only if driving forward
        if(drive > 0){
            leftFrontVelocity += bot.leftFrontDrive.getVelocity();
            leftBackVelocity += bot.leftBackDrive.getVelocity();
            rightFrontVelocity += bot.rightFrontDrive.getVelocity();
            rightBackVelocity += bot.rightBackDrive.getVelocity();
            timer++;
        }

        telemetry.addData("Drive motors", "Left --- front: %.2f, back: %.2f  ---  Right front: %.2f, back: %.2f", bot.leftFrontDrive.getPower(), bot.leftBackDrive.getPower(),bot.rightFrontDrive.getPower(), bot.rightBackDrive.getPower());
        telemetry.addData("Drive Velocity motors", "Left --- front: %.2f, back: %.2f  ---  Right front: %.2f, back: %.2f", bot.leftFrontDrive.getVelocity(), bot.leftBackDrive.getVelocity(),bot.rightFrontDrive.getVelocity(), bot.rightBackDrive.getVelocity());
        telemetry.addData("Drive Average motors", "Left --- front: %.2f, back: %.2f  ---  Right front: %.2f, back: %.2f", leftFrontVelocity/timer, leftBackVelocity/timer, rightFrontVelocity/timer, rightBackVelocity/timer);
    }

    // uses left/right dpad
    private void servoTest(Gamepad gp){
        double grabberPos = bot.grabber.getPosition();

        // grabber servo position checker
//        if (gp.dpad_left){
//            bot.grabber.setPosition(grabberPos + 0.01);
//        }else if (gp.dpad_right){
//            bot.grabber.setPosition(grabberPos - 0.01);
//        }
        if (tgg.toggle(gp.x)){
            if (bot.grabbedPosition == grabberPos){
                bot.grabber.setPosition(bot.releasedPositionHalf);
            }else if(bot.releasedPositionFull == grabberPos){
                bot.grabber.setPosition(bot.grabbedPosition);
            }else{
                bot.grabber.setPosition(bot.releasedPositionFull);
            }

        }
        telemetry.addData("Servo Position: ", grabberPos);

    }
    private void strafeDrive(Gamepad gp){
        double strafePower = Range.clip(gp.left_stick_x, -1,1);
        bot.leftFrontDrive.setPower(strafePower);
        bot.leftBackDrive.setPower(-strafePower);
        bot.rightFrontDrive.setPower(-strafePower);
        bot.rightBackDrive.setPower(strafePower);
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
        bot.leftFrontDrive.setPower(leftPower);
        bot.rightFrontDrive.setPower(rightPower);
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
        bot.leftFrontDrive.setPower(-leftPower * turn);    // turns left on default
        bot.rightFrontDrive.setPower(rightPower * turn);
    }
}