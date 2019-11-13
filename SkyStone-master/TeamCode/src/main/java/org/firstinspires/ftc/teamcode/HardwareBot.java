package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class HardwareBot {
    // strafe drive moves the robot in the horizontal direction
    public DcMotor leftDrive, rightDrive, strafeDrive, lift;
    public Servo moveFoundation, grabber;
    public ColorSensor surfaceScanner;

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

    public void initHardware(HardwareMap hardwareMap){
        HardwareMap hwmap = hardwareMap;

    /* Init motors */
        leftDrive = hwmap.get(DcMotor.class, "left_drive");
        rightDrive = hwmap.get(DcMotor.class, "right_drive");
        strafeDrive = hwmap.get(DcMotor.class, "strafe_drive");
        lift = hwmap.get(DcMotor.class, "lift");

        // Set motor direction
    /* Init servos */
        moveFoundation = hwmap.get(Servo.class, "move_foundation");
        grabber = hwmap.get(Servo.class, "grabber");

    /* Init sensors */
        surfaceScanner = hwmap.get(ColorSensor.class, "surfaceScanner");
    }
    public void initState(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        strafeDrive.setPower(0);
        lift.setPower(0);
        //todo find moveFoundation init position
        //todo find grabber init position
    // turns on color sensor led if its of
        if (surfaceScanner instanceof SwitchableLight) {
            ((SwitchableLight)surfaceScanner).enableLight(true);
        }
    }
}
