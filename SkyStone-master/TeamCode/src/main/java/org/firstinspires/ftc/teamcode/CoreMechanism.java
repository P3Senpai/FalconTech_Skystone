package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class CoreMechanism {
    private HardwareBot bot;
    private Toggle tgg;
    private LinearOpMode opMode;

/* Lift Variables*/
// todo maybe add magnetic limit switch to automatically find it (starting position)
    private final int START_POSITION = 0; // this is the position when A. the phone camera can scan B. the release will change from half to full or vice versa
    private final int MIN_HEIGHT = -1; // todo find (some negative value below the start position)
    private final int MAX_HEIGHT = -1; // todo find (some positive value above the start position)
    private final double LIFT_POWER = 0.5;
/* Grabber variables */
    private final double GRABBED_POSITION = -1; //todo find
    // faster when picking up stones off the ground
    private final double RELEASED_POSITION_HALF = -1; // todo find
    // safer when putting stones on tower
    private final double RELEASED_POSITION_FULL = -1; // todo find
    private boolean isGrabbed;

/* Constructor */
    public CoreMechanism(HardwareBot bot, LinearOpMode opMode, Toggle tgg){
        this.bot = bot;
        this.opMode = opMode;
        this.tgg = tgg;
    /* Init states */
        //todo set all init states
        bot.leftLift.setPower(0);
        bot.rightLift.setPower(0);
        isGrabbed();
    }

/* Private Methods */
    private void isGrabbed(){
    // set grabber position variable
        if (bot.grabber.getPosition() == GRABBED_POSITION)
            isGrabbed = true;
        else
            isGrabbed = false;
    }
/* Public Methods */
    public void initForController(){
        bot.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void initForEncoder(){
        //Reset encoders
        bot.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Run using encoders
        bot.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void liftByController(Gamepad gp){
        if (gp.dpad_up){
            bot.leftLift.setPower(LIFT_POWER);
            bot.rightLift.setPower(LIFT_POWER);
        }else if(gp.dpad_down){
            bot.leftLift.setPower(-LIFT_POWER);
            bot.rightLift.setPower(-LIFT_POWER);
        }else{
            bot.leftLift.setPower(0);
            bot.rightLift.setPower(0);
        }
    }
    public void liftByEncoder(){}
    public void grabByController(Gamepad gp){
        if (tgg.toggle(gp.x)) {
            if (isGrabbed)
                grab(false);
            else
                grab(true);
        }
    }
// true if you want 'toGrab' and false if you want to release (not 'toGrab')
    public void grab(boolean toGrab){
        double position = (toGrab)?GRABBED_POSITION:RELEASED_POSITION_HALF; //todo add check for variable release position (DEPENDS ON MAGNETIC SENSOR)
        isGrabbed = toGrab;
        bot.grabber.setPosition(position);
    }
//temporary method
    public double testGrabFind(Gamepad gp){
        double grabberPos = bot.grabber.getPosition();

        // grabber servo position checker
        if (gp.dpad_left){
            bot.grabber.setPosition(grabberPos + 0.01);
        }else if (gp.dpad_right){
            bot.grabber.setPosition(grabberPos - 0.01);
        }
        return grabberPos;
    }
//todo if a magnetic limit switch is used then create --> public void findStartingPosition
}
