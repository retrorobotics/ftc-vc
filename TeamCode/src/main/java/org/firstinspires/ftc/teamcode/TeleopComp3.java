/* Copyright (c) 2017 FIRST. All rights reserved.

 */

package org.firstinspires.ftc.teamcode;

// do wireless code

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

import android.graphics.Color;

@TeleOp(name="Teleop Comp 3", group="Pushbot")
//@Disabled
public class TeleopComp3 extends OpMode{



    org.firstinspires.ftc.teamcode.RRHardwarePushbot robot = new RRHardwarePushbot();

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {

        // Define installed motors
        robot.leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        robot.rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        robot.leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        robot.rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        robot.robotIntake = hardwareMap.get(DcMotor.class, "robotIntake");
        robot.ringLoader = hardwareMap.get(DcMotor.class, "ringLoader");
        robot.shooterFront = hardwareMap.get(DcMotor.class, "shooterFront");
        robot.shooterBack = hardwareMap.get(DcMotor.class, "shooterBack");

        // NEWCODE
        robot.rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        // Define installed servos
        robot.leftRightWobble = hardwareMap.get(Servo.class, "leftRightWobble");
        robot.forwardBackWobble = hardwareMap.get(Servo.class, "forwardBackWobble");
        robot.loadRingServo = hardwareMap.get(Servo.class, "loadRingServo");
        robot.tiltElevatorServo = hardwareMap.get(Servo.class, "tiltElevatorServo");
        robot.leftRingBlock = hardwareMap.get(Servo.class, "leftRingBlock");
        robot.rightRingBlock = hardwareMap.get(Servo.class, "rightRingBlock");

        // Make sure the wobble servos stay in place when they aren't moving
        robot.ringLoader.setPower(robot.MOTOR_STOP);

        robot.ringLoader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ringLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.ledRiver = hardwareMap.get(LEDRiver.IMPL, "led");
        robot.ledRiver.setMode(LEDRiver.Mode.INDIVIDUAL);
        robot.ledRiver.setLEDMode(LEDRiver.LEDMode.RGB);
        robot.ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_24);
        robot.ledRiver.setHide(false);

        robot.ledRiver.setBrightness(0.1);
        robot.ledRiver.setColor(0, Color.TRANSPARENT);
        robot.ledRiver.setColor(1, Color.TRANSPARENT);
        robot.ledRiver.setColor(2, Color.TRANSPARENT);
        robot.ledRiver.setColor(3, Color.TRANSPARENT);
        robot.ledRiver.setColor(4, Color.TRANSPARENT);
        robot.ledRiver.setColor(5, Color.TRANSPARENT);
        robot.ledRiver.setColor(6, Color.TRANSPARENT);
        robot.ledRiver.setColor(7, Color.TRANSPARENT);
        robot.ledRiver.setColor(8, Color.TRANSPARENT);
        robot.ledRiver.setColor(9, Color.TRANSPARENT);
        robot.ledRiver.setColor(10, Color.TRANSPARENT);
        robot.ledRiver.setColor(11, Color.TRANSPARENT);
        robot.ledRiver.setColor(12, Color.TRANSPARENT);
        robot.ledRiver.setColor(13, Color.TRANSPARENT);
        robot.ledRiver.setColor(14, Color.TRANSPARENT);
        robot.ledRiver.setColor(15, Color.TRANSPARENT);
        robot.ledRiver.apply();

    }

    float speedFactor = (float)robot.NORMAL_TELEOP_SPEED;
    float orientationFactor = (float)robot.FORWARD_ORIENTATION;

    double TEST_MOTOR_STOP = 0;

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

    float leftFront = 0;
    float rightFront = 0;
    float leftBack = 0;
    float rightBack = 0;

    int MAX_LED = 16;
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // NEWCODE
        double wallDist;

        telemetry.addData("**********", "**********");

        wallDist = robot.rightDistanceSensor.getDistance(DistanceUnit.INCH);

        //ROUNDING
        //multiply by ten
        //cast to int
        //cast back to double
        //and divide by ten.
        wallDist = (int) wallDist * 10;
        wallDist = (double) wallDist / 10;
        telemetry.addData("DISTANCE", wallDist);
        telemetry.addData("**********", "**********");
        telemetry.update();

        int loopCount;

        telemetry.addData("Distance", robot.rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
        if (robot.LED_ON == 1){
        // OUTSIDE RANGE
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 8)) || (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 8))) {
            telemetry.addData("Distance", "OUT OF RANGE");
            telemetry.update();

            robot.ledRiver.setBrightness(0.1);
            robot.ledRiver.setColor(0, Color.RED);
            robot.ledRiver.setColor(1, Color.RED);
            robot.ledRiver.setColor(2, Color.TRANSPARENT);
            robot.ledRiver.setColor(3, Color.TRANSPARENT);
            robot.ledRiver.setColor(4, Color.TRANSPARENT);
            robot.ledRiver.setColor(5, Color.TRANSPARENT);
            robot.ledRiver.setColor(6, Color.TRANSPARENT);
            robot.ledRiver.setColor(7, Color.TRANSPARENT);
            robot.ledRiver.setColor(8, Color.TRANSPARENT);
            robot.ledRiver.setColor(9, Color.TRANSPARENT);
            robot.ledRiver.setColor(10, Color.TRANSPARENT);
            robot.ledRiver.setColor(11, Color.TRANSPARENT);
            robot.ledRiver.setColor(12, Color.TRANSPARENT);
            robot.ledRiver.setColor(13, Color.TRANSPARENT);
            robot.ledRiver.setColor(14, Color.RED);
            robot.ledRiver.setColor(15, Color.RED);
            robot.ledRiver.apply();
        }

        // PERFECT
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 1)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 1))) {
            telemetry.addData("Distance", "PERFECT");
            telemetry.update();

            setLEDBackground();

            robot.ledRiver.setColor(7, Color.MAGENTA);
            robot.ledRiver.setColor(8, Color.MAGENTA);
            robot.ledRiver.apply();

            robot.ledRiver.setColor(7, Color.GREEN);
            robot.ledRiver.setColor(8, Color.GREEN);
            robot.ledRiver.apply();

            robot.ledRiver.setColor(7, Color.MAGENTA);
            robot.ledRiver.setColor(8, Color.MAGENTA);
            robot.ledRiver.apply();

            robot.ledRiver.setColor(7, Color.GREEN);
            robot.ledRiver.setColor(8, Color.GREEN);
            robot.ledRiver.apply();

        }
        // Off by 1 -2 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 2)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 1))) {

            telemetry.addData("Distance", "1 - 2 RIGHT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(9, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 2 - 3 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 3)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 2))) {

            telemetry.addData("Distance", "2 - 3 RIGHT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(10, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 3 - 4 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 4)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 3))) {

            telemetry.addData("Distance", "3 - 4 RIGHT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(11, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 4 - 5 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 5)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 4))) {

            telemetry.addData("Distance", "4 - 5 RIGHT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(12, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 5 - 6 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 6)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 5))) {

            telemetry.addData("Distance", "5 - 6 RIGHT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(13, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 6 - 7 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 7)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 6))) {

            telemetry.addData("Distance", "6 - 7 RIGHT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(14, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 7 - 8 on right side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT - 8)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT - 7))) {

            telemetry.addData("Distance", "7 - 8 RIGHT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(15, Color.GREEN);
            robot.ledRiver.apply();
        }

        // Off by 1 -2 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 2)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 1))) {

            telemetry.addData("Distance", "1 - 2 LEFT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(6, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 2 - 3 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 3)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 2))) {

            telemetry.addData("Distance", "2 - 3 LEFT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(5, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 3 - 4 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 4)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 3))) {

            telemetry.addData("Distance", "3 - 4 LEFT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(4, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 4 - 5 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 5)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 4))) {

            telemetry.addData("Distance", "4 - 5 LEFT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(3, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 5 - 6 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 6)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 5))) {

            telemetry.addData("Distance", "5 - 6 LEFT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(2, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 6 - 7 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 7)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 6))) {

            telemetry.addData("Distance", "6 - 7 LEFT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(1, Color.GREEN);
            robot.ledRiver.apply();
        }
        // Off by 7 - 8 on left side
        if ((robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= (robot.POWERSHOT_PERFECT + 8)) && (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) >= (robot.POWERSHOT_PERFECT + 7))) {

            telemetry.addData("Distance", "7 - 8 LEFT");
            telemetry.update();

            setLEDBackground();
            robot.ledRiver.setColor(0, Color.GREEN);
            robot.ledRiver.apply();
        }
    }


        // END NEWCODE



        if (gamepad1.right_trigger > 0.2 && !robot.rightTrigger1Pressed){


            robot.rightTrigger1Pressed = true;
            telemetry.update();
            robot.LED_ON = robot.LED_ON + 1;
            if (robot.LED_ON > 1){
                robot.LED_ON = 0;
            }
        }
        if (robot.rightTrigger1Pressed && !(gamepad1.right_trigger > 0.2)) {
            robot.rightTrigger1Pressed = false;
        }

        // Slow drive mode is activated if the left bumper is push and held down
        if (gamepad1.left_bumper) {
            speedFactor = (float)robot.SLOW_TELEOP_SPEED;
        }
        else
            // Fast drive mode is activated if the right bumper is push and held down
            if (gamepad1.right_bumper) {
                speedFactor = (float)robot.FAST_TELEOP_SPEED;
            }
                // Normal drive speed mode
                else {
                    speedFactor = (float)robot.NORMAL_TELEOP_SPEED;
                }

        // Reverse the drive orientation so that the back is forward
        if (gamepad1.dpad_down) {
            orientationFactor = (float)robot.BACKWARD_ORIENTATION;
        }

        // Drive orientation changed back to default.....front is forward
        if (gamepad1.dpad_up) {
            orientationFactor = (float)robot.FORWARD_ORIENTATION;
        }

        // Calculating motor values to move the robot in the direction that the joystick is pointed
        float gamepad1LeftY = gamepad1.left_stick_y;
        float gamepad1LeftX = -gamepad1.left_stick_x;
        float gamepad1RightX = gamepad1.right_stick_x;

        gamepad1LeftY = (float) scaleInput(gamepad1LeftY);
        gamepad1LeftX = (float) scaleInput(gamepad1LeftX);
        gamepad1RightX = (float) scaleInput(gamepad1RightX);

        leftFront = (float) (-gamepad1LeftY - gamepad1LeftX + gamepad1RightX);
        rightFront = (float) (gamepad1LeftY - gamepad1LeftX + gamepad1RightX);
        rightBack = (float) (gamepad1LeftY + gamepad1LeftX + gamepad1RightX);
        leftBack = (float) (-gamepad1LeftY + gamepad1LeftX+ gamepad1RightX);

        leftFront = Range.clip(leftFront, -1, 1);
        rightFront = Range.clip(rightFront, -1, 1);
        leftBack = Range.clip(leftBack, -1, 1);
        rightBack = Range.clip(rightBack, -1, 1);

        // Adjust the power based on the speed mode and the orientation
        leftFront = leftFront * speedFactor * orientationFactor;
        rightFront = rightFront * speedFactor * orientationFactor;
        leftBack = leftBack * speedFactor * orientationFactor;
        rightBack = rightBack * speedFactor * orientationFactor;

        robot.leftFrontDrive.setPower(leftFront);
        robot.rightFrontDrive.setPower(rightFront);
        robot.leftBackDrive.setPower(leftBack);
        robot.rightBackDrive.setPower(rightBack);

        // Buttons for the ring blocks

        if (gamepad1.left_trigger > 0.2 && !robot.leftTrigger1Pressed) {
            if (robot.LEFT_BLOCK_POSITION == 0) {
                robot.leftRingBlock.setPosition(robot.RING_BLOCK_OPEN);
            }
            else if (robot.LEFT_BLOCK_POSITION == 1){
                robot.leftRingBlock.setPosition(robot.RING_BLOCK_CLOSE);
            }
                robot.LEFT_BLOCK_POSITION = robot.LEFT_BLOCK_POSITION + 1;

            if (robot.LEFT_BLOCK_POSITION > 1) {
                robot.LEFT_BLOCK_POSITION = 0;
            }
            // Makes sure robot only registers one button press
            robot.leftTrigger1Pressed = true;
            telemetry.update();
        }
        if (robot.leftTrigger1Pressed && !(gamepad1.left_trigger > 0.2)) {
            robot.leftTrigger1Pressed = false;
        }

/*
        if (gamepad1.right_trigger > 0.2 && !robot.rightTriggerPressed) {
            if (robot.RIGHT_BLOCK_POSITION == 0) {
                robot.rightRingBlock.setPosition(robot.RIGHT_RING_BLOCK_OPEN);
            }
            else if (robot.RIGHT_BLOCK_POSITION == 1){
                robot.rightRingBlock.setPosition(robot.RING_BLOCK_CLOSE);
            }
            robot.RIGHT_BLOCK_POSITION = robot.RIGHT_BLOCK_POSITION + 1;

            if (robot.RIGHT_BLOCK_POSITION > 1) {
                robot.RIGHT_BLOCK_POSITION = 0;
            }
            // Makes sure robot only registers one button press
            robot.rightTriggerPressed = true;
            telemetry.update();
        }
        if (robot.rightTriggerPressed && !(gamepad1.right_trigger > 0.2)) {
            robot.rightTriggerPressed = false;
        }
*/



        // Buttons for having the wobble servos go up and down
        if (gamepad1.a) {
            robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_DOWN);
        }

        if (gamepad1.y) {
            robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_OPEN);
        }

        if (gamepad1.x) {
            robot.leftRightWobble.setPosition(-1);
        }

        if (gamepad1.b) {
            robot.leftRightWobble.setPosition(1);
        }


        // Wobble Servo is put all the way down
       if (gamepad1.dpad_left) {
            robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_VERY_DOWN);
        }

        // Reverse the intake motor
        if (gamepad2.dpad_down) {
            robot.robotIntake.setPower(robot.REVERSE_MOTOR);
        }

        // Push the ring loading servo out and then back in
        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            robot.loadRingServo.setPosition(-1);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.loadRingServo.setPosition(1);
        }

        // Robot shoots a ring fast
        if (gamepad2.y) {
            robot.shooterBack.setPower(robot.FAST_SHOOTING);
            robot.shooterFront.setPower(robot.FAST_SHOOTING);
            telemetry.addData("Y - Shooter Back and Shooter Front On, Power", robot.FAST_SHOOTING);
        }

        // Robot shoots a ring slow
        if (gamepad2.a) {
            robot.shooterBack.setPower(robot.SLOW_SHOOTING);
            robot.shooterFront.setPower(robot.SLOW_SHOOTING);
            telemetry.addData("A - Shooter Back and Shooter Front On, Power", robot.SLOW_SHOOTING);
        }

        // Raises the elevator to shoot the rings and then goes back down
        if (gamepad2.x && !robot.xPressed) {
            if (robot.elevatorPosition == 3) {
                telemetry.addData("EP","3");
                robot.ringLoader.setTargetPosition(robot.BOTTOM_ELEVATOR);
                robot.tiltElevatorServo.setPosition(robot.TILT_SERVO_DOWN);
            }
            if (robot.elevatorPosition == 2) {
                telemetry.addData("EP","2");
                robot.ringLoader.setTargetPosition(robot.THIRD_ELEVATOR_RAISE);
            }
            if (robot.elevatorPosition == 1) {
                telemetry.addData("EP","1");
                robot.ringLoader.setTargetPosition(robot.SECOND_ELEVATOR_RAISE);
            }
            if (robot.elevatorPosition == 0) {
                robot.tiltElevatorServo.setPosition(robot.TILT_SERVO_UP);
                // NEWCODE - 1/23/21 - Need to do this twice for the first raise of the elevator
                robot.tiltElevatorServo.setPosition(robot.TILT_SERVO_UP);
                telemetry.addData("EP","0");
                robot.ringLoader.setTargetPosition(robot.FIRST_ELEVATOR_RAISE);
            }
            robot.ringLoader.setPower(1);
            robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (robot.elevatorPosition < 3) {
                robot.elevatorPosition = robot.elevatorPosition + 1;
            }
            else {
                robot.elevatorPosition = 0;
            }

            // Makes sure robot only registers one button press
            robot.xPressed = true;
            telemetry.update();
        }
        if (robot.xPressed && !gamepad2.x) {
            robot.xPressed = false;
        }


        // lower the elevator to start position
        if (gamepad2.b) {
            // robot.ringLoader.setTargetPosition(robot.BOTTOM_ELEVATOR);
            // in case robot stuck in up position at end of run
            robot.tiltElevatorServo.setPosition(robot.TILT_SERVO_DOWN);
            robot.ringLoader.setTargetPosition(-robot.THIRD_ELEVATOR_RAISE);
            robot.ringLoader.setPower(1);
            robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevatorPosition = 0;

        }

        // Reset the elevator
        if ((gamepad2.left_stick_y == 1) && (gamepad2.right_stick_y == 1)) {
            robot.ringLoader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.ringLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("*****************************", "");
            telemetry.addData("ELEVATOR MASTER RESET AT BOTTOM", "");
            telemetry.addData("*****************************", "");
            telemetry.update();
            robot.tiltElevatorServo.setPosition(robot.TILT_SERVO_DOWN);
            robot.ringLoader.setTargetPosition(-robot.THIRD_ELEVATOR_RAISE);
            robot.ringLoader.setPower(1);
            robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevatorPosition = 0;
            robot.ringLoader.setPower(robot.MOTOR_STOP);

        }



        // Robot shoots a ring fast
        if (gamepad2.left_bumper) {
            robot.shooterFront.setPower(robot.FAST_SHOOTING);
            robot.shooterBack.setPower(robot.FAST_SHOOTING);
            telemetry.addData("LEFT BUMPER - Reverse Ring Shooter", robot.REVERSE_MOTOR);
        }

        // Robot shooter is stopped
        if (gamepad2.right_bumper) {
            robot.shooterFront.setPower(TEST_MOTOR_STOP);
            robot.shooterBack.setPower(TEST_MOTOR_STOP);
            telemetry.addData("RIGHT BUMPER - Reverse Ring Shooter", robot.REVERSE_MOTOR);
        }

        // Ring intake is turned off
        if (gamepad2.right_trigger > 0.2) {

            robot.robotIntake.setPower(robot.MOTOR_STOP);
            //telemetry.addData("R TRIGGER - Ring Intake", robot.INTAKE_SPEED);
            // robot.shooterFront.setPower(robot.MOTOR_STOP);
            // robot.shooterBack.setPower(robot.MOTOR_STOP);
        }

        // Ring intake is turned on and lowers elevator if elevator is up
        if (gamepad2.left_trigger > 0.2 && !robot.leftTriggerPressed) {

            if (robot.elevatorPosition == 3 || robot.elevatorPosition == 2 || robot.elevatorPosition == 1){
                robot.ringLoader.setTargetPosition(robot.BOTTOM_ELEVATOR);
                robot.ringLoader.setPower(1);
                robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else {
                robot.robotIntake.setPower(robot.INTAKE_SPEED);
                telemetry.addData("L TRIGGER - Ring Intake", robot.MOTOR_STOP);
                robot.tiltElevatorServo.setPosition(robot.TILT_SERVO_DOWN);
            }
            robot.elevatorPosition = 0;

            robot.leftTriggerPressed = true;
            telemetry.update();
        }

        // Makes sure robot only registers one trigger press
        if (robot.leftTriggerPressed && !(gamepad2.left_trigger > 0.2)) {
            robot.leftTriggerPressed = false;
        }

        if (robot.DEBUG_MODE_ON == true) {
            telemetry.update();
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

    public void setLEDBackground() {
        // setup LED light strip background color
        int LEDCount = 0;

        while (LEDCount < MAX_LED) {
            robot.ledRiver.setColor(LEDCount, Color.BLUE);
            LEDCount = LEDCount + 1;
        }
        robot.ledRiver.apply();
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
