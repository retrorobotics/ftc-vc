/* Copyright (c) 2017 FIRST. All rights reserved.

 */

package org.firstinspires.ftc.teamcode;

// do wireless code

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Thread.sleep;

@TeleOp(name="Teleop Test", group="Pushbot")
//@Disabled
public class TeleopTest extends OpMode{



    org.firstinspires.ftc.teamcode.RRHardwarePushbot robot = new RRHardwarePushbot();

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {

        // NEWCODE
        robot.rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override

    public void start() {}


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // NEWCODE
        double wallDist;

        LEDRiver ledRiver = hardwareMap.get(LEDRiver.IMPL, "led");
        ledRiver.setHide(false);
        ledRiver.setMode(LEDRiver.Mode.INDIVIDUAL);
        ledRiver.setLEDMode(LEDRiver.LEDMode.RGB);
        ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_24);
       // ledRiver.setColor(0, new LEDRiver.Color(255, 0, 0, 0));
       // ledRiver.setColor(1, Color.GREEN);
       // ledRiver.setColor(2, Color.BLACK);
        ledRiver.setBrightness(0.01).apply();


        telemetry.addData("**********", "**********");

        wallDist = robot.rightDistanceSensor.getDistance(DistanceUnit.INCH);

        //ROUNDING
        //multiply by ten
        //cast to int
        //cast back to double
        //and divide by ten.
        wallDist = (int) wallDist * 10;
        wallDist = (double) wallDist / 10;
        //telemetry.addData("DISTANCE", wallDist);
        //telemetry.addData("**********", "**********");
        telemetry.update();


        if (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= robot.POWERSHOT_PERFECT - (robot.POWERSHOT_PERFECT * robot.PERFECT_PERCENTAGE) && robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) < robot.POWERSHOT_PERFECT + (robot.POWERSHOT_PERFECT * robot.PERFECT_PERCENTAGE)) {
            ledRiver.setColor(0, Color.BLUE).apply();
            ledRiver.setColor(1, Color.BLUE).apply();
            ledRiver.setColor(2, Color.BLUE).apply();
            ledRiver.setColor(3, Color.BLUE).apply();
            ledRiver.setColor(4, Color.BLUE).apply();
            ledRiver.setColor(5, Color.BLUE).apply();
            ledRiver.setColor(6, Color.GREEN).apply();
            ledRiver.setColor(7, Color.GREEN).apply();
            ledRiver.setColor(8, Color.GREEN).apply();
            ledRiver.setColor(9, Color.GREEN).apply();
            ledRiver.setColor(10, Color.BLUE).apply();
            ledRiver.setColor(11, Color.BLUE).apply();
            ledRiver.setColor(12, Color.BLUE).apply();
            ledRiver.setColor(13, Color.BLUE).apply();
            ledRiver.setColor(14, Color.BLUE).apply();
            ledRiver.setColor(15, Color.BLUE).apply();
            telemetry.addData("DISTANCE", wallDist);
            telemetry.addData("**********", "**********");
            telemetry.update();
        }

        if (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= robot.POWERSHOT_PERFECT + (robot.POWERSHOT_PERFECT * robot.SLIGHTLY_TO_LEFT_LOW_PERCENTAGE) && robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= robot.POWERSHOT_PERFECT + (robot.POWERSHOT_PERFECT * robot.SLIGHTLY_TO_LEFT_HIGH_PERCENTAGE) ) {
            ledRiver.setColor(0, Color.BLUE).apply();
            ledRiver.setColor(1, Color.BLUE).apply();
            ledRiver.setColor(2, Color.BLUE).apply();
            ledRiver.setColor(3, Color.BLUE).apply();
            ledRiver.setColor(4, Color.BLUE).apply();
            ledRiver.setColor(5, Color.BLUE).apply();
            ledRiver.setColor(6, Color.BLUE).apply();
            ledRiver.setColor(7, Color.BLUE).apply();
            ledRiver.setColor(8, Color.BLUE).apply();
            ledRiver.setColor(9, Color.BLUE).apply();
            ledRiver.setColor(10, Color.YELLOW).apply();
            ledRiver.setColor(11, Color.YELLOW).apply();
            ledRiver.setColor(12, Color.BLUE).apply();
            ledRiver.setColor(13, Color.BLUE).apply();
            ledRiver.setColor(14, Color.BLUE).apply();
            ledRiver.setColor(15, Color.BLUE).apply();
        }

        if (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= robot.POWERSHOT_PERFECT - (robot.POWERSHOT_PERFECT * robot.SLIGHTLY_TO_RIGHT_HIGH_PERCENTAGE) && robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= robot.POWERSHOT_PERFECT - (robot.POWERSHOT_PERFECT * robot.SLIGHTLY_TO_RIGHT_LOW_PERCENTAGE) ) {
            ledRiver.setColor(0, Color.BLUE).apply();
            ledRiver.setColor(1, Color.BLUE).apply();
            ledRiver.setColor(2, Color.BLUE).apply();
            ledRiver.setColor(3, Color.BLUE).apply();
            ledRiver.setColor(4, Color.YELLOW).apply();
            ledRiver.setColor(5, Color.YELLOW).apply();
            ledRiver.setColor(6, Color.BLUE).apply();
            ledRiver.setColor(7, Color.BLUE).apply();
            ledRiver.setColor(8, Color.BLUE).apply();
            ledRiver.setColor(9, Color.BLUE).apply();
            ledRiver.setColor(10, Color.BLUE).apply();
            ledRiver.setColor(11, Color.BLUE).apply();
            ledRiver.setColor(12, Color.BLUE).apply();
            ledRiver.setColor(13, Color.BLUE).apply();
            ledRiver.setColor(14, Color.BLUE).apply();
            ledRiver.setColor(15, Color.BLUE).apply();
        }

        if (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= robot.POWERSHOT_PERFECT + (robot.POWERSHOT_PERFECT * robot.TO_THE_LEFT_LOW_PERCENTAGE) && robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= robot.POWERSHOT_PERFECT + (robot.POWERSHOT_PERFECT * robot.TO_THE_LEFT_HIGH_PERCENTAGE) ) {
            ledRiver.setColor(0, Color.BLUE).apply();
            ledRiver.setColor(1, Color.BLUE).apply();
            ledRiver.setColor(2, Color.BLUE).apply();
            ledRiver.setColor(3, Color.BLUE).apply();
            ledRiver.setColor(4, Color.BLUE).apply();
            ledRiver.setColor(5, Color.BLUE).apply();
            ledRiver.setColor(6, Color.BLUE).apply();
            ledRiver.setColor(7, Color.BLUE).apply();
            ledRiver.setColor(8, Color.BLUE).apply();
            ledRiver.setColor(9, Color.BLUE).apply();
            ledRiver.setColor(10, Color.BLUE).apply();
            ledRiver.setColor(11, Color.BLUE).apply();
            ledRiver.setColor(12, Color.YELLOW).apply();
            ledRiver.setColor(13, Color.YELLOW).apply();
            ledRiver.setColor(14, Color.BLUE).apply();
            ledRiver.setColor(15, Color.BLUE).apply();
        }

        if (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= robot.POWERSHOT_PERFECT - (robot.POWERSHOT_PERFECT * robot.TO_THE_RIGHT_HIGH_PERCENTAGE) && robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= robot.POWERSHOT_PERFECT - (robot.POWERSHOT_PERFECT * robot.TO_THE_RIGHT_LOW_PERCENTAGE) ) {
            ledRiver.setColor(0, Color.BLUE).apply();
            ledRiver.setColor(1, Color.BLUE).apply();
            ledRiver.setColor(2, Color.YELLOW).apply();
            ledRiver.setColor(3, Color.YELLOW).apply();
            ledRiver.setColor(4, Color.BLUE).apply();
            ledRiver.setColor(5, Color.BLUE).apply();
            ledRiver.setColor(6, Color.BLUE).apply();
            ledRiver.setColor(7, Color.BLUE).apply();
            ledRiver.setColor(8, Color.BLUE).apply();
            ledRiver.setColor(9, Color.BLUE).apply();
            ledRiver.setColor(10, Color.BLUE).apply();
            ledRiver.setColor(11, Color.BLUE).apply();
            ledRiver.setColor(12, Color.BLUE).apply();
            ledRiver.setColor(13, Color.BLUE).apply();
            ledRiver.setColor(14, Color.BLUE).apply();
            ledRiver.setColor(15, Color.BLUE).apply();
        }
        if (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= robot.POWERSHOT_PERFECT + (robot.POWERSHOT_PERFECT * robot.EXTREME_TO_THE_LEFT_LOW_PERCENTAGE) && robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= robot.POWERSHOT_PERFECT + (robot.POWERSHOT_PERFECT * robot.EXTREME_TO_THE_LEFT_HIGH_PERCENTAGE) ) {
            ledRiver.setColor(0, Color.BLUE).apply();
            ledRiver.setColor(1, Color.BLUE).apply();
            ledRiver.setColor(2, Color.BLUE).apply();
            ledRiver.setColor(3, Color.BLUE).apply();
            ledRiver.setColor(4, Color.BLUE).apply();
            ledRiver.setColor(5, Color.BLUE).apply();
            ledRiver.setColor(6, Color.BLUE).apply();
            ledRiver.setColor(7, Color.BLUE).apply();
            ledRiver.setColor(8, Color.BLUE).apply();
            ledRiver.setColor(9, Color.BLUE).apply();
            ledRiver.setColor(10, Color.BLUE).apply();
            ledRiver.setColor(11, Color.BLUE).apply();
            ledRiver.setColor(12, Color.BLUE).apply();
            ledRiver.setColor(13, Color.BLUE).apply();
            ledRiver.setColor(14, Color.YELLOW).apply();
            ledRiver.setColor(15, Color.YELLOW).apply();
        }

        if (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= robot.POWERSHOT_PERFECT - (robot.POWERSHOT_PERFECT * robot.EXTREME_TO_THE_RIGHT_HIGH_PERCENTAGE) && robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= robot.POWERSHOT_PERFECT - (robot.POWERSHOT_PERFECT * robot.EXTREME_TO_THE_RIGHT_LOW_PERCENTAGE) ) {
            ledRiver.setColor(0, Color.YELLOW).apply();
            ledRiver.setColor(1, Color.YELLOW).apply();
            ledRiver.setColor(2, Color.BLUE).apply();
            ledRiver.setColor(3, Color.BLUE).apply();
            ledRiver.setColor(4, Color.BLUE).apply();
            ledRiver.setColor(5, Color.BLUE).apply();
            ledRiver.setColor(6, Color.BLUE).apply();
            ledRiver.setColor(7, Color.BLUE).apply();
            ledRiver.setColor(8, Color.BLUE).apply();
            ledRiver.setColor(9, Color.BLUE).apply();
            ledRiver.setColor(10, Color.BLUE).apply();
            ledRiver.setColor(11, Color.BLUE).apply();
            ledRiver.setColor(12, Color.BLUE).apply();
            ledRiver.setColor(13, Color.BLUE).apply();
            ledRiver.setColor(14, Color.BLUE).apply();
            ledRiver.setColor(15, Color.BLUE).apply();
        }
    }
    @Override
    public void stop() {
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.03, 0.05, 0.07, 0.10, 0.13, 0.15, 0.21,
                0.26, 0.31, 0.38, 0.45, 0.54, 0.57, 0.63, 0.71, 0.76 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }



    /**
     * Move a servo by the selected increment.
     * @param servo: servo to move
     * @param increment: amount to move it
     */
    private void servoIncrement(Servo servo, double increment){
        servoIncrement( servo, increment, 0, 1);
    }

    /**
     * Move a servo by the selected increment within the provided bounds
     * @param servo: serve to move
     * @param increment: amount to move it
     * @param minServoPosition: minimum position the servo can be moved to
     * @param maxServoPosition: maximum position the servo can be moved to
     */
    private void servoIncrement( Servo servo, double increment, double minServoPosition, double maxServoPosition){
        boolean allowMove = false;
        if ( increment > 0 && servo.getPosition() < maxServoPosition){
            allowMove = true;
        } else if ( increment < 0 && servo.getPosition() > minServoPosition ){
            allowMove = true;
        }

        if ( allowMove ) {
            servo.setPosition(servo.getPosition() + increment);
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
