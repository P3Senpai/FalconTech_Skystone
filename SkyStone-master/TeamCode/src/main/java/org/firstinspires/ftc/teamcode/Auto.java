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
import com.qualcomm.robotcore.hardware.DcMotor;
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

import java.util.concurrent.locks.*;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="TEST AU", group="Pushbot")

public class Auto extends LinearOpMode {

    // Vuforia Variables Begin

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY =
            "AcU0zmb/////AAABmXdOrBtsW00Zt5mvKhMGGtIBIFineZDj2/dVSeeiQzxa53LgSVImKxPWdDr39Z9IVgQdluFUWAnaDeQ+H117Lqe1RM5+aSzIWsnSN9fNpyc5923VYWVpUIsCAuHvVIktKzOB+3qC92MgEn6/Z0eMkstRDsa6laJ7wSrAxnQIRmdlxYAd7K1FQfWwhiKoae/PNTs9vriRm8t3h0T4DnnvqlX/hc/yMS8g+siEgbZGZMC04UuWe19eb0KMBRAeZowfCMnOhRRyAmz0eJ6qDNlq2VWXjVk3a9GcPNHXnhvIBujlchjtGgQYvG8GHVmjua8YVpjfCT5t1tLBqYVKPLkfQ2sGkto9zXOdeteA25EFglwT";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;     // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private float forward = 0;
    private float sideways = 0;
    private float up = 0;

    // Vuforia Variables End

    //IMU Variables Begin
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;
    //IMU Variables End

    /* Declare OpMode members. */
    HardwareBot         robot   = new HardwareBot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MALE_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_FEMALE_MOTOR_REV  = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.95 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 2.95 ;     // For figuring circumference // diameter 7.5 cm then converted to 2.952756 inches
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

        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU Init Beginning

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        param.loggingEnabled      = true;
        param.loggingTag          = "IMU";
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(param);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        // IMU Init End

        // Vuforia Init Beginning

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 2.0f * mmPerInch;   // eg: Camera is 2.0 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 2.4f * mmPerInch;   // eg: Camera is 2.4 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 1.75f * mmPerInch;     // eg: Camera is left of center line by 1.75 Inches

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);

        // Vuforia Init End

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        releaseFull();

        waitForStart();

        encoderLift(20, 6, 1);
        runVuforia(5);
        if(targetVisible){
            telemetry.addData("forward", forward);
            telemetry.addData("sideways", sideways);
        }
        else{
            //strafe 12.75 inches to the left and pick up the cube
            telemetry.addData("sh", "it");
        }

//        double newForward = Math.abs(forward);
//        if(sideways < 0){
//            encoderDrive(DRIVE_SPEED, strafeDistance(sideways), -strafeDistance(sideways), -strafeDistance(sideways), strafeDistance(sideways), 20);
//        }
//        else{
//            encoderDrive(DRIVE_SPEED, -strafeDistance(sideways), strafeDistance(sideways), strafeDistance(sideways), -strafeDistance(sideways), 20);
//        }
//        encoderLift(20, 6, -1);
//        encoderDrive(DRIVE_SPEED, newForward, newForward, newForward, newForward, 20);
//        grab();

//
//        encoderDrive(DRIVE_SPEED,21,21,21,21,20);
//        runVuforia(5);
//        if(targetVisible){
//            telemetry.addData("suck", "it", "vuforia");
//             if(sideways < 0){
//                 //strafe right
//                encoderDrive(DRIVE_SPEED, strafeDistance(sideways), -strafeDistance(sideways), -strafeDistance(sideways), strafeDistance(sideways), 20);
//             }
//             else{
//                 //strafe left
//                 encoderDrive(DRIVE_SPEED, -strafeDistance(sideways), strafeDistance(sideways), strafeDistance(sideways), -strafeDistance(sideways), 20);
//             }
//            encoderLift(20,6, -1);
//
//            forward = Math.abs(forward);
//            encoderDrive(DRIVE_SPEED, strafeDistance(forward), strafeDistance(forward), strafeDistance(forward), -strafeDistance(forward), 20);
//
//        }
//        else{
//            //strafe 12.75 inches to the left and pick up the cube
//            telemetry.addData("sh", "it");
//            encoderDrive(DRIVE_SPEED, strafeDistance(-12.75),strafeDistance(12.75),strafeDistance(12.75),strafeDistance(-12.75),20);
//            encoderLift(20,6, -1);
//            encoderDrive(DRIVE_SPEED,8,8,8,8,20);
//        }
//
//        grab();



        //encoderDrive(DRIVE_SPEED,21,21,21,21,20);
        //encoderDrive(DRIVE_SPEED, strafeDistance(-10),strafeDistance(10),strafeDistance(10),strafeDistance(-10),20);


//        double newStrafe = strafeDistance(10);
//        encoderDrive(DRIVE_SPEED, -newStrafe,newStrafe,newStrafe,-newStrafe,20);


        //encoderDrive(DRIVE_SPEED,Math.abs(forward),Math.abs(forward),Math.abs(forward),Math.abs(forward),20);

        //moving right on y axis is negative

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        telemetry.addData("Path", "Complete");
        telemetry.update();
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
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void grab(){
        robot.grabber.setPosition(robot.grabbedPosition);
        sleep(500);
    }
    public void releaseHalf(){
        robot.grabber.setPosition(robot.releasedPositionHalf);
        sleep(500);
    }
    public void releaseFull(){
        robot.grabber.setPosition(robot.releasedPositionFull);
        sleep(500);
    }
    public void encoderLift(int timeoutS, int inches, int UP){
        // UP is 1 if going up and -1 if going down
        int liftLeftTarget;
        int liftRightTarget;

        if(opModeIsActive()){
            liftLeftTarget = robot.leftLift.getCurrentPosition() + (int)(COUNTS_PER_FEMALE_MOTOR_REV * 0.25 * inches) * UP;
            liftRightTarget = robot.rightLift.getCurrentPosition() + (int)(COUNTS_PER_FEMALE_MOTOR_REV * 0.25 * inches) * UP;

            robot.leftLift.setTargetPosition(liftLeftTarget);
            robot.rightLift.setTargetPosition(liftRightTarget);

            robot.leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftLift.setPower(Math.abs(LIFT_SPEED) * UP);
            robot.rightLift.setPower(Math.abs(LIFT_SPEED) * UP);

            while(opModeIsActive() && runtime.seconds() < timeoutS && robot.leftLift.isBusy() && robot.rightLift.isBusy()){

            }
            robot.leftLift.setPower(0);
            robot.rightLift.setPower(0);

            robot.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void runVuforia(double timeOut) {
        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.
        // todo synchronized (this) {
            targetsSkyStone.activate();
            runtime.reset();

            while (!isStopRequested() && !targetVisible && runtime.seconds() < timeOut) {
                CameraDevice.getInstance().setFlashTorchMode(false);
                CameraDevice.getInstance().setFocusMode(CameraDevice.FOCUS_MODE.FOCUS_MODE_CONTINUOUSAUTO);

                //encoderDrive(.8,1,1,1,1,5);
                // check all the trackable targets to see which one (if any) is visible.
                targetVisible = false;

                if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", stoneTarget.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }

                // Provide feedback as to where the robot is located (if we know).
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    forward = translation.get(0) / mmPerInch;
                    sideways = translation.get(1) / mmPerInch;
                    up = translation.get(2) / mmPerInch;

                    CameraDevice.getInstance().setFlashTorchMode(false);
                    targetsSkyStone.deactivate();

                } else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();
            }
        // todo }
    }

    public double strafeDistance(double inches){
        return inches * 1.2;
    }

    // turning

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private void rotate(int degrees, double power)
    {
        double  leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

        // restart imu movement tracking.
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftFrontPower = power;
            leftBackPower = power;
            rightFrontPower = -power;
            rightBackPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftFrontPower = -power;
            leftBackPower = -power;
            rightFrontPower = power;
            rightBackPower = power;
        }
        else return;

        // set power to rotate.
        robot.leftFrontDrive.setPower(leftFrontPower);
        robot.leftBackDrive.setPower(leftBackPower);
        robot.rightFrontDrive.setPower(rightFrontPower);
        robot.rightBackDrive.setPower(rightBackPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
}
