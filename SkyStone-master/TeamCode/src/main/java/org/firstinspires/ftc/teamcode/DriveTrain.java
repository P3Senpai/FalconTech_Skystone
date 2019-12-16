package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class DriveTrain {
    private HardwareBot bot;

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
    public DriveTrain(HardwareBot bot){
        this.bot = bot;
    //todo set all init states
        bot.leftFrontDrive.setPower(0);
        bot.rightFrontDrive.setPower(0);
        bot.leftBackDrive.setPower(0);
        bot.rightBackDrive.setPower(0);
        bot.strafeDrive.setPower(0);
    }

    public String powerToString(){
        return "test";
    }
/* Private Methods */
    private void initForController(){}
    private void initForEncoder(){}

/* Public Methods */
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
    public void driveByEncoder(){}
    public void strafeByEncoder(){}
    public void turnByAngle(){}
    public boolean driveAlignLine(){return false;}
    public boolean strafeAlignLine(){return false;}


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
