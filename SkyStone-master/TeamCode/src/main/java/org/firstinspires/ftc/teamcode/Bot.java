package org.firstinspires.ftc.teamcode;

/* This class initializes all of the classes for the robot's components */
public class Bot {
    private HardwareBot robot;
    private DriveTrain driveTrain;
    private CoreMechanism liftPlusGrabber;

    public Bot(boolean isTeleop, Toggle tgg){
        this.robot = new HardwareBot();
        robot.init(isTeleop);   // maybe add more init states
        this.driveTrain = new DriveTrain(this.robot);
        this.liftPlusGrabber = new CoreMechanism(this.robot, tgg);
    }

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public CoreMechanism getLiftPlusGrabber() {
        return liftPlusGrabber;
    }
}
