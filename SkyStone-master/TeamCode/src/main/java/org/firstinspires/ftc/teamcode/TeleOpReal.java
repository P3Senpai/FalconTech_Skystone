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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Official TeleOp", group="Linear Opmode")
@Disabled
public class TeleOpReal extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Toggle tgg = new Toggle();
    private HardwareBot bot = new HardwareBot();


    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            petrControls(gamepad1);
            harryControls(gamepad2);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            tgg.reset();
        }
    }
    private void petrControls(Gamepad gp){
        drive(gp);
    }
    private void harryControls(Gamepad gp){
        lift(gp); // uses left stick and dpad up,down
        grabber(gp); // uses x button
        tapeShooter(gp); // uses a button
    }

    private void tapeShooter(Gamepad gp){
        if(gp.a)
            bot.shooter.setPower(0.5);
        else
            bot.shooter.setPower(0);
    }
    // uses dpad up and down
    private void lift(Gamepad gp){
        boolean withinRange = (bot.leftLift.getCurrentPosition() >=0 && bot.rightLift.getCurrentPosition() >= 0);
        // TODO: see if it works
        if (gp.dpad_up){
            bot.leftLift.setPower(0.75);
            bot.rightLift.setPower(0.75);
        }else if(gp.dpad_down && withinRange){
            bot.leftLift.setPower(-0.6);
            bot.rightLift.setPower(-0.6);
        }else{
            bot.leftLift.setPower(0);
            bot.rightLift.setPower(0);
        }

        //todo add 0 reset for current position

        telemetry.addData("Lift ticks", "left: %.0f right: %.0f",(double)bot.leftLift.getCurrentPosition(),(double)bot.rightLift.getCurrentPosition());
    }

    //uses left and right stick and both triggers
    private void drive(Gamepad gp){
        double percentPower = 0.7; // normal speed
        double drive = -gp.left_stick_y;
        double turn = gp.right_stick_x;
        double strafe = (gp.left_trigger > gp.right_trigger)? -gp.left_trigger: gp.right_trigger; // left trigger is between -1 and 0 right trigger is between 0 and 1

        double leftFrontPower = Range.clip((drive * 0.8065) + strafe + turn, -1, 1); // average velocity 2202 (1721.0/2202) +0.025
        double leftBackPower = Range.clip((drive * 0.83509) - strafe + turn, -1,1); // average velocity 2114 (1721.0/2114)  +0.025
        double rightFrontPower = Range.clip((drive * 0.96848) - strafe - turn, -1,1); // average velocity 1777 (1721.0/1777)
        double rightBackPower = Range.clip((drive) + strafe - turn, -1,1); // average velocity 1721 (smallest value)

        // sprint
        if (gp.left_stick_button || gp.right_stick_button){
            percentPower = 1.;
        }

        bot.leftFrontDrive.setPower(leftFrontPower * percentPower);
        bot.leftBackDrive.setPower(leftBackPower * percentPower);
        bot.rightFrontDrive.setPower(rightFrontPower* percentPower);
        bot.rightBackDrive.setPower(rightBackPower* percentPower);

        telemetry.addData("Drive motors", "Left --- front: %.2f, back: %.2f  ---  Right front: %.2f, back: %.2f", bot.leftFrontDrive.getPower(), bot.leftBackDrive.getPower(),bot.rightFrontDrive.getPower(), bot.rightBackDrive.getPower());
        telemetry.addData("Drive Velocity motors", "Left --- front: %.2f, back: %.2f  ---  Right front: %.2f, back: %.2f", bot.leftFrontDrive.getVelocity(), bot.leftBackDrive.getVelocity(),bot.rightFrontDrive.getVelocity(), bot.rightBackDrive.getVelocity());
    }

    // uses left/right dpad
    private void grabber(Gamepad gp){
        double grabberPos = bot.grabber.getPosition();
        double releasePos = (bot.leftLift.getCurrentPosition() > 100 || bot.rightLift.getCurrentPosition() > 100)?bot.releasedPositionFull:bot.releasedPositionHalf;

        if (tgg.toggle(gp.x)){
            if (bot.grabbedPosition == grabberPos){
                bot.grabber.setPosition(releasePos);
            }else{
                bot.grabber.setPosition(bot.grabbedPosition);
            }
        }
        //todo see if it works
        if (tgg.hold(gp.x,3,runtime.seconds())){ // sets to the capstone open position
            bot.grabber.setPosition(0.45); //todo to find
        }
        telemetry.addData("Servo Position: ", grabberPos);

    }
}