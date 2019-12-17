package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/* This class initializes all of the classes for the robot's components */
public class Bot {
    private HardwareBot robot;
    private DriveTrain driveTrain;
    private CoreMechanism liftPlusGrabber;


    public Bot(LinearOpMode opMode,Toggle tgg, int initProfile){
        this.robot = new HardwareBot();
        robot.init();
        this.driveTrain = new DriveTrain(this.robot, opMode, tgg);
        this.liftPlusGrabber = new CoreMechanism(this.robot, opMode,tgg);
        initProfiles(initProfile);
    }

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public CoreMechanism getLiftPlusGrabber() {
        return liftPlusGrabber;
    }
    /* Init the hardware's starting state*/
    private void initProfiles(int initProfile){
        switch (initProfile) {
            /* Initializes for Autonomous Mode -- BLUE */
            case 1:
                //todo set things to blue
                driveTrain.initForEncoder();
                liftPlusGrabber.initForEncoder();
                enableColorSensors(true);
                break;
            /* Initializes for Autonomous Mode -- RED  */
            case 2:
                //todo set things to red
                driveTrain.initForEncoder();
                liftPlusGrabber.initForEncoder();
                enableColorSensors(true);
                break;
            /* Default initializes for Teleop Mode */
            default:
                driveTrain.initForController();
                liftPlusGrabber.initForController();
                enableColorSensors(false);
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
}
