package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareBot {
    // strafe drive moves the robot in the horizontal direction
    public DcMotor leftFrontDrive, rightFrontDrive,leftBackDrive, rightBackDrive, strafeDrive, leftLift, rightLift;
    public Servo  grabber;
    //public ColorSensor surfaceScannerLeft, surfaceScannerRight;

    public HardwareBot(){} //todo should i add an option for different settings?

    /* By separating state from hardware we can by make multiple setting profiles for state init
     * For e.g. different init methods for teleOp and Autonomous
     */
    public void init(HardwareMap hardwareMap){
    /* Init the hardware's starting state*/
        initHardware(hardwareMap);
    /* Init the hardware's starting state*/
        initState();
    }

    private void initHardware(HardwareMap hardwareMap){
        HardwareMap hwmap = hardwareMap;

    /* Init motors */
        leftFrontDrive = hwmap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hwmap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hwmap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hwmap.get(DcMotor.class, "right_back_drive");
        strafeDrive = hwmap.get(DcMotor.class, "strafe_drive");
        leftLift = hwmap.get(DcMotor.class, "left_lift");
        rightLift = hwmap.get(DcMotor.class, "right_lift");

        // Set motor direction
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
    /* Init servos */
        grabber = hwmap.get(Servo.class, "grabber");

    /* Init sensors */
     //   surfaceScannerLeft = hwmap.get(ColorSensor.class, "surface_scanner_left");
     //   surfaceScannerRight = hwmap.get(ColorSensor.class, "surface_scanner_right");
    }
    private void initState(){
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        strafeDrive.setPower(0);
        leftLift.setPower(0);
        rightLift.setPower(0);
        //todo find grabber init position
    // turns on color sensor led if its off
      /*  if (surfaceScannerLeft instanceof SwitchableLight) {
            ((SwitchableLight) surfaceScannerLeft).enableLight(true);
        }
        if (surfaceScannerRight instanceof SwitchableLight) {
            ((SwitchableLight) surfaceScannerRight).enableLight(true);
        }*/
    }
}
