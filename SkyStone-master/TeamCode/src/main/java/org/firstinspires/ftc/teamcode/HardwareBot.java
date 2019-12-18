package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareBot {
    private HardwareMap hardwareMap; //todo THIS MIGHT NOT WORK
    // strafe drive moves the robot in the horizontal direction
    public DcMotor leftFrontDrive, rightFrontDrive,leftBackDrive, rightBackDrive, strafeDrive, leftLift, rightLift;
    public Servo  grabber;
    public final int BLUE_VALUE = -1; // todo find
    public final int RED_VALUE = -1; // todo find
    //public ColorSensor surfaceScannerLeft, surfaceScannerRight;

    public HardwareBot(){} //todo should i add an option for different settings?

    /* By separating state from hardware we can by make multiple setting profiles for state init
     * For e.g. different init methods for teleOp and Autonomous
     */
    public void init(){
    /* Init the hardware's starting state*/
        initHardware();

//        if(isTeleop){
//            initState();
//        }else{
//            initAutonomousState();
//        }
    }

    private void initHardware(){
        HardwareMap hwmap = hardwareMap;

    /* Init motors */
        leftFrontDrive = hwmap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hwmap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hwmap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hwmap.get(DcMotor.class, "right_back_drive");
        strafeDrive = hwmap.get(DcMotor.class, "strafe_drive");
        leftLift = hwmap.get(DcMotor.class, "left_lift");
        rightLift = hwmap.get(DcMotor.class, "right_lift");

    /* Set motor direction */
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

    /* Init servos */
        grabber = hwmap.get(Servo.class, "grabber");

    /* Init sensors */
     //   surfaceScannerLeft = hwmap.get(ColorSensor.class, "surface_scanner_left");
     //   surfaceScannerRight = hwmap.get(ColorSensor.class, "surface_scanner_right");
    }

    private void initAutonomousState(){
    // turns on color sensor led if its on
      /*  if (surfaceScannerLeft instanceof SwitchableLight) {
            ((SwitchableLight) surfaceScannerLeft).enableLight(true);
        }
        if (surfaceScannerRight instanceof SwitchableLight) {
            ((SwitchableLight) surfaceScannerRight).enableLight(true);
        }*/
    }
}
