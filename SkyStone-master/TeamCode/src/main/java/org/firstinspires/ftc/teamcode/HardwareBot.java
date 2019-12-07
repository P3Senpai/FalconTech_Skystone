package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareBot {
    // strafe drive moves the robot in the horizontal direction
    public DcMotor leftDrive, rightDrive, strafeDrive, leftLift, rightLift;
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
        leftDrive = hwmap.get(DcMotor.class, "left_drive");
        rightDrive = hwmap.get(DcMotor.class, "right_drive");
        strafeDrive = hwmap.get(DcMotor.class, "strafe_drive");
        leftLift = hwmap.get(DcMotor.class, "left_lift");
        rightLift = hwmap.get(DcMotor.class, "right_lift");

        // Set motor direction
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
    /* Init servos */
        grabber = hwmap.get(Servo.class, "grabber");

    /* Init sensors */
     //   surfaceScannerLeft = hwmap.get(ColorSensor.class, "surface_scanner_left");
     //   surfaceScannerRight = hwmap.get(ColorSensor.class, "surface_scanner_right");
    }
    private void initState(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
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
