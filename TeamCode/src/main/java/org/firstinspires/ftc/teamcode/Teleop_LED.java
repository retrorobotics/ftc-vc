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

@TeleOp(name="Teleop LED Test", group="Pushbot")
//@Disabled
public class Teleop_LED extends OpMode{



    // Declare OpMode members
    org.firstinspires.ftc.teamcode.RRHardwarePushbot robot = new RRHardwarePushbot();

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        int loopCount = 0;


        // initialize LED Lights
        //robot.ledRiver = hardwareMap.get(LEDRiver.IMPL, "led");
        //robot.ledRiver.setMode(LEDRiver.Mode.INDIVIDUAL);
        //robot.ledRiver.setLEDMode(LEDRiver.LEDMode.RGB);
        //robot.ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_24);
        //robot.ledRiver.setHide(false);

        // initialize Distance Sensor
        robot.rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        // initial config for
        //robot.ledRiver.setBrightness(0.1);

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

    int MAX_LED = 16;


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        int loopCount;

        telemetry.addData("Distance", robot.rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        // OUTSIDE RANGE
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 8)) || (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 8))) {
            telemetry.addData("Distance", "OUT OF RANGE");
            telemetry.update();

            //robot.ledRiver.setBrightness(0.1);
            //robot.ledRiver.setColor(0, Color.RED);
            //robot.ledRiver.setColor(1, Color.RED);
            //robot.ledRiver.setColor(2, Color.TRANSPARENT);
            //robot.ledRiver.setColor(3, Color.TRANSPARENT);
            //robot.ledRiver.setColor(4, Color.TRANSPARENT);
            //robot.ledRiver.setColor(5, Color.TRANSPARENT);
            //robot.ledRiver.setColor(6, Color.TRANSPARENT);
            //robot.ledRiver.setColor(7, Color.TRANSPARENT);
            //robot.ledRiver.setColor(8, Color.TRANSPARENT);
            //robot.ledRiver.setColor(9, Color.TRANSPARENT);
            //robot.ledRiver.setColor(10, Color.TRANSPARENT);
            //robot.ledRiver.setColor(11, Color.TRANSPARENT);
            //robot.ledRiver.setColor(12, Color.TRANSPARENT);
            //robot.ledRiver.setColor(13, Color.TRANSPARENT);
            //robot.ledRiver.setColor(14, Color.RED);
            //robot.ledRiver.setColor(15, Color.RED);
            //robot.ledRiver.apply();
        }

        // PERFECT
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 1)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 1))) {
            telemetry.addData("Distance", "PERFECT");
            telemetry.update();

            setLEDBackground();

            //robot.ledRiver.setColor(7, Color.MAGENTA);
            //robot.ledRiver.setColor(8, Color.MAGENTA);
            //robot.ledRiver.apply();

            //robot.ledRiver.setColor(7, Color.GREEN);
            //robot.ledRiver.setColor(8, Color.GREEN);
            //robot.ledRiver.apply();

            //robot.ledRiver.setColor(7, Color.MAGENTA);
            //robot.ledRiver.setColor(8, Color.MAGENTA);
            //robot.ledRiver.apply();

            //robot.ledRiver.setColor(7, Color.GREEN);
            //robot.ledRiver.setColor(8, Color.GREEN);
            //robot.ledRiver.apply();

        }
        // Off by 1 -2 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 2)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 1))) {

            telemetry.addData("Distance", "1 - 2 RIGHT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(9, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 2 - 3 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 3)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 2))) {

            telemetry.addData("Distance", "2 - 3 RIGHT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(10, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 3 - 4 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 4)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 3))) {

            telemetry.addData("Distance", "3 - 4 RIGHT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(11, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 4 - 5 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 5)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 4))) {

            telemetry.addData("Distance", "4 - 5 RIGHT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(12, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 5 - 6 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 6)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 5))) {

            telemetry.addData("Distance", "5 - 6 RIGHT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(13, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 6 - 7 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 7)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 6))) {

            telemetry.addData("Distance", "6 - 7 RIGHT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(14, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 7 - 8 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 8)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 7))) {

            telemetry.addData("Distance", "7 - 8 RIGHT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(15, Color.GREEN);
            //robot.ledRiver.apply();
        }

        // Off by 1 -2 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 2)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 1))) {

            telemetry.addData("Distance", "1 - 2 LEFT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(6, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 2 - 3 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 3)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 2))) {

            telemetry.addData("Distance", "2 - 3 LEFT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(5, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 3 - 4 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 4)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 3))) {

            telemetry.addData("Distance", "3 - 4 LEFT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(4, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 4 - 5 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 5)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 4))) {

            telemetry.addData("Distance", "4 - 5 LEFT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(3, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 5 - 6 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 6)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 5))) {

            telemetry.addData("Distance", "5 - 6 LEFT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(2, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 6 - 7 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 7)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 6))) {

            telemetry.addData("Distance", "6 - 7 LEFT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(1, Color.GREEN);
            //robot.ledRiver.apply();
        }
        // Off by 7 - 8 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 8)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 7))) {

            telemetry.addData("Distance", "7 - 8 LEFT");
            telemetry.update();

            setLEDBackground();
            //robot.ledRiver.setColor(0, Color.GREEN);
            //robot.ledRiver.apply();
        }
    }

    void setLEDBackground() {
        // setup LED light strip background color
        int LEDCount = 0;

        while (LEDCount < MAX_LED){
            //robot.ledRiver.setColor(LEDCount, Color.BLUE);
            LEDCount = LEDCount + 1;
        }
        //robot.ledRiver.apply();
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

    public void robotLiftEncoder(DcMotor ringLoader, int tickDistance, double driveSpeed) {
        // this will set the tick count back to 0
        ringLoader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the distance - this is where you set +/- for forward and reverse of motor
        ringLoader.setTargetPosition(-tickDistance);

        // set the speed
        ringLoader.setPower(driveSpeed);

        // set the mode to run until the distance is achieved
        ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void robotLiftEncoderNoReset(DcMotor ringLoader, int tickDistance, double driveSpeed) {
        // set the distance - this is where you set +/- for forward and reverse of motor
        ringLoader.setTargetPosition(-tickDistance);

        // set the speed
        ringLoader.setPower(driveSpeed);

        // set the mode to run until the distance is achieved
        ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
