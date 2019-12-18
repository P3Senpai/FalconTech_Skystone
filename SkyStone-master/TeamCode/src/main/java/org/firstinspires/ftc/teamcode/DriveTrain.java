package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class DriveTrain {
    private HardwareBot bot;
    private Toggle tgg;
    private LinearOpMode opMode;

/* Encoder variable initialization */
    private ElapsedTime runtime = new ElapsedTime();
    private static final double     COUNTS_PER_MOTOR_REV    = 300 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_CM       = 9.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    private static final double     DRIVE_SPEED             = 0.6;
    private static final double     TURN_SPEED              = 0.5;

/* Constructor */
    public DriveTrain(HardwareBot bot, LinearOpMode opMode, Toggle tgg){
        this.bot = bot;
        this.opMode = opMode;
        this.tgg = tgg;
    /* Init states */
        //todo set all init states
        bot.leftFrontDrive.setPower(0);
        bot.rightFrontDrive.setPower(0);
        bot.leftBackDrive.setPower(0);
        bot.rightBackDrive.setPower(0);
        bot.strafeDrive.setPower(0);
    }

    public void powerToString(){ //todo see if it works
        opMode.telemetry.addData("Drive motors", "Left --- front: %.2f, back: %.2f  ---  Right front: %.2f, back: %.2f  ---  Strafe %.2f", bot.leftFrontDrive.getPower(), bot.leftBackDrive.getPower(),bot.rightFrontDrive.getPower(), bot.rightBackDrive.getPower(), bot.strafeDrive.getPower());
    }
/* Private Methods */

/* Public Methods */
    public void initForController(){
        bot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void initForEncoder(){
        // Sets motor Mode
        //Reset encoder
        bot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.strafeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Run using encoder
        bot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveByController(Gamepad gp){
        double drive = -gp.left_stick_y;
        double turn = gp.right_stick_x;
        double leftPower = Range.clip(drive + turn, -1,1);
        double rightPower = Range.clip(drive - turn, -1,1);
        double strafePower = Range.clip(gp.left_stick_x, -1,1);

        bot.leftFrontDrive.setPower(leftPower);
        bot.leftBackDrive.setPower(leftPower);
        bot.rightFrontDrive.setPower(rightPower);
        bot.rightBackDrive.setPower(rightPower);

        bot.strafeDrive.setPower(strafePower);
        // todo add data for joysticks and motors
    }

    public void driveByEncoder(double speed, int distanceCM, double timeoutS){
        int newTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = bot.leftFrontDrive.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM);
            bot.leftFrontDrive.setTargetPosition(newTarget);
            bot.leftBackDrive.setTargetPosition(newTarget);
            bot.rightFrontDrive.setTargetPosition(newTarget);
            bot.rightBackDrive.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            bot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            bot.leftFrontDrive.setPower(Math.abs(speed));
            bot.leftBackDrive.setPower(Math.abs(speed));
            bot.rightFrontDrive.setPower(Math.abs(speed));
            bot.rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (bot.leftFrontDrive.isBusy() && bot.rightFrontDrive.isBusy()
                            && bot.leftBackDrive.isBusy() && bot.rightBackDrive.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d", newTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        bot.leftFrontDrive.getCurrentPosition(),
                        bot.rightFrontDrive.getCurrentPosition());
                opMode.telemetry.update();
            }
        // todo replace with init method BUT first see if it works
            // Stop all motion;
            bot.leftFrontDrive.setPower(0);
            bot.rightFrontDrive.setPower(0);
            bot.leftBackDrive.setPower(0);
            bot.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            initForEncoder();

//              sleep(250);   // optional pause after each move
        }
    }
    public void strafeByEncoder(double speed, double distanceCM, double timeOutS){
        if (opMode.opModeIsActive()){
        // Set target distance
            int newTarget = bot.strafeDrive.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM);
            bot.strafeDrive.setTargetPosition(newTarget);
        // Run to position
            bot.strafeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // reset runtime and start motion
            runtime.reset();
            bot.strafeDrive.setPower(Math.abs(speed));
        // keep looping while we are still active, and there is time left, and the motor is running.
            while(opMode.opModeIsActive() && runtime.seconds() < timeOutS
            && bot.strafeDrive.isBusy()){
                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d", newTarget);
                opMode.telemetry.addData("Path2", "Running at %7d", bot.strafeDrive.getCurrentPosition());
                opMode.telemetry.update();
            }
        // Stop motor
            bot.strafeDrive.setPower(0);
        // Turn off RUN_TO_POSITION
            initForEncoder();
//              sleep(250);   // optional pause after each move
        }
    }
    public void turnByAngle(double speed, double angle, int turnLeft, double timeOuts){

    }
    public boolean isOnLine(ColorSensor leftSensor, ColorSensor rightSensor){
        boolean isOnBlueLine = leftSensor.blue() >= bot.BLUE_VALUE || rightSensor.blue() >= bot.BLUE_VALUE; //todo see how color sensor works
        boolean isOnRedLine = leftSensor.red() >= bot.RED_VALUE || rightSensor.red() >= bot.RED_VALUE;
        return isOnBlueLine || isOnRedLine;
    }
// region Both alignLine methods will only be useful if they can be done at the same time as driving and without need to stop the robot to align itself
    public void driveAlignLine(ColorSensor leftSensor, ColorSensor rightSensor){
        boolean isLeftSensorOnLine = leftSensor.blue() >= bot.BLUE_VALUE || leftSensor.red() >= bot.RED_VALUE;
        boolean isRightSensorOnLine = rightSensor.blue() >= bot.RED_VALUE || rightSensor.red() >= bot.RED_VALUE;
        while(!isLeftSensorOnLine || !isRightSensorOnLine){
            if (!isLeftSensorOnLine){
                //turn left
                //todo complete
            }else if(!isRightSensorOnLine){
                // turn right
                //todo complete
            }
            // gets the new values
            isLeftSensorOnLine = leftSensor.blue() >= bot.BLUE_VALUE || leftSensor.red() >= bot.RED_VALUE;
            isRightSensorOnLine = rightSensor.blue() >= bot.RED_VALUE || rightSensor.red() >= bot.RED_VALUE;
        }
    }
    public void strafeAlignLine(ColorSensor leftSensor, ColorSensor rightSensor){
        boolean isOnBlueLine = leftSensor.blue() >= bot.BLUE_VALUE || rightSensor.blue() >= bot.BLUE_VALUE;
        boolean isOnRedLine = leftSensor.red() >= bot.RED_VALUE || rightSensor.red() >= bot.RED_VALUE;

    }
// endregion
    
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
