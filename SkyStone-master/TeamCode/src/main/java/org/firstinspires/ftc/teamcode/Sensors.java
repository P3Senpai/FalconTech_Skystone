package org.firstinspires.ftc.teamcode;

public class Sensors {
    private HardwareBot robot;

    public Sensors(HardwareBot bot){
        this.robot = bot;
    }

    private void enableColorLED(boolean turnOn){
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
