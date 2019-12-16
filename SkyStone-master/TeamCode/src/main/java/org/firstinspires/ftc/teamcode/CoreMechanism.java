package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class CoreMechanism {
    private HardwareBot bot;
    private Toggle tgg;

/* Lift Variables*/
    private final int GROUND_POSITION = -1; // todo find
    // todo maybe add magnetic limit switch to auto find it
    private final int START_POSITION = 0; // todo find
    private final double LIFT_POWER = 0.5;
    // should we code a controller button where it lifts to different block heights (Toggle)
/* Grabber variables */
    private final double GRABBED_POSITION = -1; //todo find
    private final double RELEASED_POSITION = -1; // todo find
    private boolean isGrabbed;

/* Constructor */
    public CoreMechanism(HardwareBot bot, Toggle tgg){
        this.bot = bot;
        this.tgg = tgg;
    // get grabber position
        if (bot.grabber.getPosition() == GRABBED_POSITION)
            isGrabbed = true;
        else
            isGrabbed = false;

    //todo set all init states
        bot.leftLift.setPower(0);
        bot.rightLift.setPower(0);
    }

/* Private Methods */
    private void initForController(){}
    private void initForEncoder(){}

/* Public Methods */
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
        double position = (toGrab)?GRABBED_POSITION:RELEASED_POSITION;
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
