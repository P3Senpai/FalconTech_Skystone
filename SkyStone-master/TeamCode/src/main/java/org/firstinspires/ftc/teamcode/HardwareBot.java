package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareBot {
    // strafe drive moves the robot in the horizontal direction
    public DcMotor leftFrontDrive, rightFrontDrive,leftBackDrive, rightBackDrive, leftLift, rightLift, shooter;
    public Servo  grabber;
//    public ColorSensor surfaceScannerLeft, surfaceScannerRight;
    private HardwareMap hwmap = null;
    private DriveTrain driveTrain;
    private CoreMechanism liftPlusGrabber;
// Grabber pos
    public double grabbedPosition = 0.22; // original is 0.2
    // faster when picking up stones off the ground
    public double releasedPositionHalf = 0.4183; // original 0.4183
    // safer when putting stones on tower
    public double releasedPositionFull = 0.7994; // original 0.7994


    public HardwareBot(){}

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public CoreMechanism getLiftPlusGrabber() {
        return liftPlusGrabber;
    }

    /* Init the hardware's starting state*/
    private void initProfiles(String initProfile){
        switch (initProfile) {
            /* Initializes for Autonomous Mode -- BLUE */
            case "AutonomousRed":
                //todo set things to blue
//                driveTrain.initForEncoder();
//                liftPlusGrabber.initForEncoder();
                enableColorSensors(true);
                break;
            /* Initializes for Autonomous Mode -- RED  */
            case "AutonomousBlue":
                //todo set things to red
//                driveTrain.initForEncoder();
//                liftPlusGrabber.initForEncoder();
                enableColorSensors(true);
                break;
            /* Initializes for Teleop Mode */
            case "TeleOp":
//                driveTrain.initForController();
//                liftPlusGrabber.initForController();
                enableColorSensors(false);
                break;
            default:
                break;
        }
    }

    //todo configure color sensors
    private void enableColorSensors(boolean turnOn){
        if (!turnOn){
            // turns on color sensor led if its off

//           if (robot.surfaceScannerLeft instanceof SwitchableLight) {
//                ((SwitchableLight) robot.surfaceScannerLeft).enableLight(false);
//            }
//            if (robot.surfaceScannerRight instanceof SwitchableLight) {
//                ((SwitchableLight) robot.surfaceScannerRight).enableLight(false);
//            }
        }else{
            // turns on color sensor led if its on
//            if (robot.surfaceScannerLeft instanceof SwitchableLight) {
//                ((SwitchableLight) robot.surfaceScannerLeft).enableLight(true);
//            }
//            if (robot.surfaceScannerRight instanceof SwitchableLight) {
//                ((SwitchableLight) robot.surfaceScannerRight).enableLight(true);
//            }
        }
    }
    private void initIMU(){}
    private void initVuforia(boolean turnOn){}

    public void init(HardwareMap hardwareMap){
        hwmap = hardwareMap;

        /* Init motors */
        leftFrontDrive = hwmap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hwmap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hwmap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hwmap.get(DcMotor.class, "right_back_drive");
        leftLift = hwmap.get(DcMotor.class, "left_lift");
        rightLift = hwmap.get(DcMotor.class, "right_lift");
        shooter = hwmap.get(DcMotor.class, "shooter");

        /* Set motor direction */
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* Init servos */
        grabber = hwmap.get(Servo.class, "grabber");

        /* Init sensors */
//        surfaceScannerLeft = hwmap.get(ColorSensor.class, "surface_scanner_left");
//        surfaceScannerRight = hwmap.get(ColorSensor.class, "surface_scanner_right");
    }

}
