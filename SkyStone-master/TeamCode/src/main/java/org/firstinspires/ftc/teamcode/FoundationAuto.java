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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="Foundation Auto", group="Pushbot")

public class FoundationAuto extends LinearOpMode {


    /* Declare OpMode members. */
    HardwareBot         robot   = new HardwareBot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MALE_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_FEMALE_MOTOR_REV  = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.95 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference // diameter 7.5 cm then converted to 2.952756 inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MALE_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     LIFT_SPEED              = 0.5;
    static final Lock lock = new ReentrantLock();

    static VuforiaTrackables targetsSkyStone;
    static VuforiaTrackable stoneTarget;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.leftLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        releaseFull();

        waitForStart();

        //todo

        encoderLift(20,3,1,0.7);
        drive(29.25);
        encoderLift(20,2,-1,0.7);
        drive(-29.25);
        encoderLift(20,3,1,0.7);



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void strafeRight(double inches){
        encoderDrive(DRIVE_SPEED, strafeDistance(inches), -strafeDistance(inches), -strafeDistance(inches), strafeDistance(inches), 20);
    }

    public void strafeLeft(double inches){
        encoderDrive(DRIVE_SPEED, -strafeDistance(inches), strafeDistance(inches), strafeDistance(inches), -strafeDistance(inches), 20);
    }
    public double strafeDistance(double inches){
        return inches * 1.2;
    }
    public void drive(double inches){
        encoderDrive(DRIVE_SPEED, inches,inches,inches,inches, 20);
    }

    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);


            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            // new motors don't require a sleep in the middle
            robot.leftFrontDrive.setPower(Math.abs(speed));
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.rightBackDrive.isBusy() && robot.leftBackDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                            robot.leftFrontDrive.getCurrentPosition(),
                            robot.rightFrontDrive.getCurrentPosition(),
                            robot.leftBackDrive.getCurrentPosition(),
                            robot.rightBackDrive.getCurrentPosition());
                    telemetry.update();
                }

            // Stop all motion;
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void releaseFull(){
        robot.grabber.setPosition(robot.releasedPositionFull);
        sleep(500);
    }
    public void encoderLift(int timeoutS, double inches, int UP, double liftSpeed){
        // UP is 1 if going up and -1 if going down
        int liftLeftTarget;
        int liftRightTarget;

        if(opModeIsActive()){
            liftLeftTarget = robot.leftLift.getCurrentPosition() + (int)(COUNTS_PER_FEMALE_MOTOR_REV * 0.2 * inches) * UP;
            liftRightTarget = robot.rightLift.getCurrentPosition() + (int)(COUNTS_PER_FEMALE_MOTOR_REV * 0.2 * inches) * UP;

            robot.leftLift.setTargetPosition(liftLeftTarget);
            robot.rightLift.setTargetPosition(liftRightTarget);

            robot.leftLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.rightLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftLift.setPower(Math.abs(liftSpeed) * UP);
            robot.rightLift.setPower(Math.abs(liftSpeed) * UP);

            while(opModeIsActive() && runtime.seconds() < timeoutS && robot.leftLift.isBusy() && robot.rightLift.isBusy()){

            }
            robot.leftLift.setPower(0);
            robot.rightLift.setPower(0);

            robot.leftLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.rightLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        }
    }


}
