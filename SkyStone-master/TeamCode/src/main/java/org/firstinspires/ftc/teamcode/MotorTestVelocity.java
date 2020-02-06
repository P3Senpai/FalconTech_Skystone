package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorTestVelocity extends LinearOpMode {
    public static DcMotorEx motor1, motor2, motor3, motor4;
    private static HardwareMap hwmap = null;
    private static ElapsedTime runtime = new ElapsedTime();
    private static final double DRIVE_POWER = 0.8;
    private double motor1Vel, motor2Vel, motor3Vel, motor4Vel;
    private int timer;

    @Override
    public void runOpMode() throws InterruptedException {

        // init motors
        motor1 = hwmap.get(DcMotorEx.class, "motor_1");
        motor2 = hwmap.get(DcMotorEx.class, "motor_2");
        motor3 = hwmap.get(DcMotorEx.class, "motor_3");
        motor4 = hwmap.get(DcMotorEx.class, "motor_4");

        waitForStart();

        motor1.setPower(DRIVE_POWER);
        motor2.setPower(DRIVE_POWER);
        motor3.setPower(DRIVE_POWER);
        motor4.setPower(DRIVE_POWER);
        runtime.reset();

        while(opModeIsActive() && runtime.seconds() < 30){
            motor1Vel += motor1.getVelocity();
            motor2Vel += motor2.getVelocity();
            motor3Vel += motor3.getVelocity();
            motor4Vel += motor4.getVelocity();
            timer++;
        }

        telemetry.addData("Motor 1 Average Velocity: ", motor1Vel/timer);
        telemetry.addData("Motor 2 Average Velocity: ", motor2Vel/timer);
        telemetry.addData("Motor 3 Average Velocity: ", motor3Vel/timer);
        telemetry.addData("Motor 4 Average Velocity: ", motor4Vel/timer);



    }
}
