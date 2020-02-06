package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoFindPosition extends LinearOpMode {

    private double finalPosition, otherFinalPosition, currentPosition = -1;
    private Servo servo;
    private Toggle tgg = new Toggle();

    @Override
    public void runOpMode() throws InterruptedException {
        currentPosition = servo.getPosition();

        if (tgg.toggle(gamepad1.dpad_up)){ // increases servo position by 0.005
            servo.setPosition(currentPosition + 0.005);
        }else if(tgg.toggle(gamepad1.dpad_down)) { // decreases servo position by 0.005
            servo.setPosition(currentPosition - 0.005);
        }
        telemetry.addData("Servo's Current Position: ", currentPosition);

        if (tgg.toggle(gamepad1.a)){ // assign positions to values when a button is pressed
            if (finalPosition == -1){
                finalPosition = currentPosition;
                telemetry.addData("First position found: ", finalPosition);
            }else{
                otherFinalPosition = currentPosition;
                telemetry.addData("Second position found", otherFinalPosition);
            }
        }

        tgg.reset();
    }
}