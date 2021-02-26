/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.graphics.DrawFilter;
import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous (name = "Auto Comp 3", group = "Concept")
public class  AutoComp3 extends LinearOpMode {
    // Declare OpMode members
    org.firstinspires.ftc.teamcode.RRHardwarePushbot robot = new RRHardwarePushbot();

    // ??
    private ElapsedTime runtime = new ElapsedTime();

    // Web Cam
    RingDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {
        // Variables
        int ringsDetected = 0;

        // Define installed motors
        robot.leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        robot.rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        robot.leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        robot.rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        robot.robotIntake = hardwareMap.get(DcMotor.class, "robotIntake");
        robot.ringLoader = hardwareMap.get(DcMotor.class, "ringLoader");
        robot.shooterFront = hardwareMap.get(DcMotor.class, "shooterFront");
        robot.shooterBack = hardwareMap.get(DcMotor.class, "shooterBack");

        // Define distance sensors
        robot.frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
        robot.rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
        robot.leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");

        // Define touch sensor
        robot.wobbleTouchSensor = hardwareMap.get(TouchSensor.class, "wobbleTouchSensor");

        // Initialize installed motors ensuring they are not running
        robot.leftFrontDrive.setPower(robot.MOTOR_STOP);
        robot.rightFrontDrive.setPower(robot.MOTOR_STOP);
        robot.leftBackDrive.setPower(robot.MOTOR_STOP);
        robot.rightBackDrive.setPower(robot.MOTOR_STOP);
        robot.robotIntake.setPower(robot.MOTOR_STOP);
        robot.ringLoader.setPower(robot.MOTOR_STOP);
        robot.shooterFront.setPower(robot.MOTOR_STOP);
        robot.shooterBack.setPower(robot.MOTOR_STOP);

        // Initialize motors to start running without encoders
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.robotIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ringLoader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define installed servos
        robot.leftRightWobble = hardwareMap.get(Servo.class, "leftRightWobble");
        robot.forwardBackWobble = hardwareMap.get(Servo.class, "forwardBackWobble");
        robot.loadRingServo = hardwareMap.get(Servo.class, "loadRingServo");
        robot.tiltElevatorServo = hardwareMap.get(Servo.class, "tiltElevatorServo");
        robot.leftRingBlock = hardwareMap.get(Servo.class, "leftRingBlock");
        robot.rightRingBlock = hardwareMap.get(Servo.class, "rightRingBlock");

        // Web Cam Setup
        // robot.currentAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        robot.phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        robot.phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        robot.phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Retro updates
                // Using Sideways Right so that the text info does not lay over the video
                robot.phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
        });

        // Lock servos in place
        robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_UP);
        robot.leftRightWobble.setPosition(robot.WOBBLE_SERVO_CLOSE);
        robot.tiltElevatorServo.setPosition(robot.TILT_SERVO_UP);
        robot.leftRingBlock.setPosition(robot.RING_BLOCK_CLOSE);

        robot.ledRiver = hardwareMap.get(LEDRiver.IMPL, "led");
        robot.ledRiver.setMode(LEDRiver.Mode.INDIVIDUAL);
        robot.ledRiver.setLEDMode(LEDRiver.LEDMode.RGB);
        robot.ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_24);
        robot.ledRiver.setHide(false);

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

        waitForStart();

        // Initialize and Setup Gyro Sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);
        robot.currentAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (opModeIsActive()) {
            // this is added protection in case someone pressed the init and then start without waiting 3-5 seconds
           /*
            while (detectionCheck <= robot.MAX_DETECTION_CHECK) {
                ringsDetected = pipeline.ringCount();
                telemetry.addData("Detection Value", pipeline.avg1);
                // if we find 4 rings, no need to keep checking, end loop
                if (ringsDetected == 4) {
                    detectionCheck = robot.MAX_DETECTION_CHECK;
                }
                ++detectionCheck;
                sleep(500);
            }
        */
            ringsDetected = pipeline.ringCount();

            // Hardcode ring count for testing
            // *********************

             //ringsDetected = 0;

            // *********************

            telemetry.addData("Rings Found", ringsDetected);
            telemetry.update();


            // For Scenario A
            if (ringsDetected == 0) {

                // Drive to drop off point for A
                robot.driveRightEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.A_DELIVER_WOBBLE, robot.DRIVE_SPEED);

                // align the robot to 0 degrees
                robot.angleAlignRobot();

                // Align robot to wall to get to 5 inches away
                if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 5) {
                    robot.driveForwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 5) {
                        telemetry.addData("Toward Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                } else if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 5) {
                    robot.driveBackwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 5) {
                        telemetry.addData("Away Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                }

                // Release wobble goal
                releaseWobbleGoal();

                // Back away from the wobble goal
                robot.driveLeftEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_A_BACKUP_FROM_WOBBLE, robot.DRIVE_SPEED);

                // Align robot to wall to ensure 7.5 inches away
                if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 7.5) {
                    robot.driveForwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 7.5) {
                        telemetry.addData("Toward Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                } else if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 7.5) {
                    robot.driveBackwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 7.5) {
                        telemetry.addData("Away Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                }

                // Rotate robot to the right 90 degrees
                robot.rotateRobot(-90);

                // Correct robot angle if it over/undershoots
                robot.angleAlignRobot();

                // Drive toward the 2nd wobble goal
                robot.driveRightEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_A_SHIFT_TO_SECOND_WOBBLE, robot.DRIVE_SPEED);

                // JAN 10 REVIEW - IS THIS NEEDED?
                // sleep(robot.NORMAL_SLEEP);

                // Pickup second wobble
                pickupWobbleGoal();

                sleep(robot.LONG_SLEEP);

                // Goes to put down the wobble if it is picked up
               //  if (robot.wobbleTouchSensor.isPressed() == Boolean.TRUE) {

                    // Align robot to wall to ensure 9 inches away
                    if (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) > 9) {
                        robot.driveLeftNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                        // align robot from side wall
                        while (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) > 9) {
                            telemetry.addData("Toward Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                        }
                        robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                    } else if (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) < 9) {
                        robot.driveRightNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                        // align robot from side wall
                        while (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) < 9) {
                            telemetry.addData("Away Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                        }
                        robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                    }

                    // Align robot if the angle gets messed up
                    robot.angleAlignRobot();

                    // Rotate the robot to the left 85 degrees
                    robot.rotateRobot(85);

                    // Drive back to A to deliver 2nd wobble goal
                    robot.driveRightEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_A_BACK_TO_WHITE_LINE, robot.DRIVE_SPEED);

                    // Release 2nd wobble goal
                    releaseWobbleGoal();

                    // Back away from the wobble goal
                    robot.driveLeftEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.A_BACKUP_FROM_WOBBLE, robot.DRIVE_SPEED);

                    // Shift to the right
                    robot.driveBackwardEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.A_SHIFT_RIGHT, robot.DRIVE_SPEED);

                    // Drive up to white line
                    robot.driveRightEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_A_GO_TO_WHITE_LINE, robot.DRIVE_SPEED);

                    // lock wobble goal in up position
                    robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_UP);
                    robot.leftRightWobble.setPosition(robot.WOBBLE_SERVO_CLOSE);

                    // Rotate the robot to the left 90 degrees for teleop
                    robot.rotateRobot(-90);


//                 }
                // Shoots rings if robot misses pickup of 2nd wobble goal
   /*             else {

                    // JAN 10 REVIEW - ISN'T THE ROBOT ALREADY FACING THE RIGHT DIRECTION?
                    // Robot rotates so the shooter faces the ring goal
                    // robot.rotateRobot(178);

                    // Robot drives close to the white line
                    robot.driveForwardEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_GO_TO_SHOOTING, robot.DRIVE_SPEED);

                    // Setup the ring loader elevator
                    robot.ringLoader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.ringLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    // JAN 10 REVIEW - DON'T WE NEED TO TURN ROBOT TO THE RIGHT A BIT?
                    robot.rotateRobot(-10);

                    // Start the two motors/wheels that shoot the rings
                    robot.shooterFront.setPower(robot.FAST_SHOOTING);
                    robot.shooterBack.setPower(robot.FAST_SHOOTING);

                    // RING 1 - TOP RING
                    // Raise the elevator so that the top ring of the 3 is in shooting position
                    robot.ringLoader.setTargetPosition(robot.FIRST_ELEVATOR_RAISE);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // Continue raising the elevator until it is at the proper height for the top ring
                    while (robot.ringLoader.isBusy()) {
                        // raise elevator to 1st position
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Engage the servo to push the top ring to the the shooting wheel to shot the ring and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_FORWARD);
                    sleep(robot.SERVO_SLEEP);

                    // Return the servo to the start position and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_BACK);
                    sleep(robot.SERVO_SLEEP);

                    // RING 2 - MIDDLE RING
                    // Raise the elevator so that the middle ring of the 3 is in shooting position
                    robot.ringLoader.setTargetPosition(robot.SECOND_ELEVATOR_RAISE);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    // Continue raising the elevator until it is at the proper height for the middle ring
                    while (robot.ringLoader.isBusy()) {
                        // raise elevator to 2nd position
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Engage the servo to push the middle ring to the the shooting wheel to shot the ring and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_FORWARD);
                    sleep(robot.SERVO_SLEEP);

                    // Return the servo to the start position and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_BACK);
                    sleep(robot.SERVO_SLEEP);

                    // RING 3 - BOTTOM RING
                    // Raise the elevator so that the bottom ring of the 3 is in shooting position
                    robot.ringLoader.setTargetPosition(robot.THIRD_ELEVATOR_RAISE);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    // Continue raising the elevator until it is at the proper height for the bottom ring
                    while (robot.ringLoader.isBusy()) {
                        // raise elevator to 3rd position
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Engage the servo to push the bottom ring to the the shooting wheel to shot the ring and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_FORWARD);
                    sleep(robot.SERVO_SLEEP);

                    // Return the servo to the start position and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_BACK);
                    sleep(robot.SERVO_SLEEP);

                    // Lower the elevator to the original/bottom position
                    robot.ringLoader.setTargetPosition(robot.BOTTOM_ELEVATOR);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.tiltElevatorServo.setPosition(robot.TILT_SERVO_DOWN);

                    // Continue lower the elevator until it has reached the bottom
                    while (robot.ringLoader.isBusy()) {
                        // Drive
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Stop the two motors/wheels that shoot the rings
                    robot.shooterFront.setPower(robot.MOTOR_STOP);
                    robot.shooterBack.setPower(robot.MOTOR_STOP);


                    // park on the white line
                    robot.driveForwardEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_GO_TO_WHITE_LINE, robot.DRIVE_SPEED);

                    // lock wobble goal in up position
                    robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_UP);
                    robot.leftRightWobble.setPosition(robot.WOBBLE_SERVO_CLOSE);
                }
    */
            }




            // For Scenario B
            if (ringsDetected == 1) {

                // Drive to drop off point for B
                robot.driveRightEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.B_DELIVER_WOBBLE, robot.DRIVE_SPEED);

                // Align the robot to 0 degrees
                robot.angleAlignRobot();

                // Align robot to wall to get to 9 inches away
                if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 9) {
                    robot.driveForwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 9) {
                        telemetry.addData("Toward Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                } else if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 9) {
                    robot.driveBackwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 9) {
                        telemetry.addData("Away Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                }

                // Shift to the right
                robot.driveBackwardEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.B_MOVE_TO_WHITE_LINE, robot.DIST_SENSOR_SPEED);

                // Release wobble goal
                releaseWobbleGoal();

                // Raise the forwardBackWobble servo so it doesn't hit the wobble goal
                robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_UP);
                sleep(robot.SHORT_SLEEP);

                // Backup from wobble
                robot.driveForwardEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_B_BACKUP_FROM_WOBBLE, robot.DRIVE_SPEED);

                // Put the forwardBackWobble servo back down
                robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_DOWN);

                // Align robot if the angle changes
                robot.angleAlignRobot();

                // Backup from wobble
                robot.driveLeftEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_B_BACK_TO_SECOND_WOBBLE, robot.DRIVE_SPEED);

                // Align robot to wall to ensure 8.5 inches away
                if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 8.5) {
                    robot.driveForwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 8.5) {
                        telemetry.addData("Toward Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                } else if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 8.5) {
                    robot.driveBackwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 8.5) {
                        telemetry.addData("Away Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                }


                // Rotate the robot 90 degrees to the right
                robot.rotateRobot(-90);

                // Align the robot if the angle over/undershoots
                robot.angleAlignRobot();

                // Go to the second wobble
                robot.driveRightEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_B_SHIFT_TO_SECOND_WOBBLE, robot.DRIVE_SPEED);

                // sleep(robot.NORMAL_SLEEP);

                // Pickup the second wobble
                pickupWobbleGoal();

                //sleep(robot.NORMAL_SLEEP);

               // Goes to put down wobble if it is picked up
               // if (robot.wobbleTouchSensor.isPressed() == Boolean.TRUE) {

                    // Back away from the second wobble
                    robot.driveLeftEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_B_SHIFT_LEFT, robot.DRIVE_SPEED);

                    // Align robot to wall to ensure 9 inches away
                    if (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) > 9) {
                        robot.driveLeftNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                        // align robot from side wall
                        while (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) > 9) {
                            telemetry.addData("Toward Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                        }
                        robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                    } else if (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) < 9) {
                        robot.driveRightNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                        // align robot from side wall
                        while (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) < 9) {
                            telemetry.addData("Away Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                        }
                        robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                    }

                    // Align the robot if the angle gets changed
                    robot.angleAlignRobot();

                    // Rotate the robot 87 degrees to the left
                    robot.rotateRobot(87);

                    // Drive towards B zone
                    robot.driveRightEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_B_BACK_TO_B_ZONE, robot.DRIVE_SPEED);

                    // Go to the B zone to deliver second wobble
                    robot.driveBackwardEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_B_GO_TO_B_ZONE, robot.DRIVE_SPEED);

                    // Release wobble goal
                    finalReleaseWobbleGoal();

                    robot.leftRingBlock.setPosition(robot.RING_BLOCK_OPEN);

                    // Go to white line
                    robot.driveLeftEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_B_GO_TO_WHITE_LINE, robot.DRIVE_SPEED);

                    // lock wobble goal in up position
                    robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_UP);
                    robot.leftRightWobble.setPosition(robot.WOBBLE_SERVO_CLOSE);

                    // Rotate the robot 90 degrees to the right for teleop
                    robot.rotateRobot(-90);
                /*}

                // Robot shoots rings if it misses the wobble
                else{
                    // Robot rotates so the shooter faces the ring goal
                    // robot.rotateRobot(178);

                    // Robot drives close to the white line
                    robot.driveForwardEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_SHORT_GO_TO_SHOOTING, robot.DRIVE_SPEED);

                    robot.rotateRobot(-10);

                    // Setup the ring loader elevator
                    robot.ringLoader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.ringLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    // Start the two motors/wheels that shoot the rings
                    robot.shooterFront.setPower(robot.FAST_SHOOTING);
                    robot.shooterBack.setPower(robot.FAST_SHOOTING);

                    // RING 1 - TOP RING
                    // Raise the elevator so that the top ring of the 3 is in shooting position
                    robot.ringLoader.setTargetPosition(robot.FIRST_ELEVATOR_RAISE);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // Continue raising the elevator until it is at the proper height for the top ring
                    while (robot.ringLoader.isBusy()) {
                        //drive
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Engage the servo to push the top ring to the the shooting wheel to shot the ring and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_FORWARD);
                    sleep(robot.SERVO_SLEEP);

                    // Return the servo to the start position and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_BACK);
                    sleep(robot.SERVO_SLEEP);

                    // RING 2 - MIDDLE RING
                    // Raise the elevator so that the middle ring of the 3 is in shooting position
                    robot.ringLoader.setTargetPosition(robot.SECOND_ELEVATOR_RAISE);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    // Continue raising the elevator until it is at the proper height for the middle ring
                    while (robot.ringLoader.isBusy()) {
                        //drive
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Engage the servo to push the middle ring to the the shooting wheel to shot the ring and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_FORWARD);
                    sleep(robot.SERVO_SLEEP);

                    // Return the servo to the start position and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_BACK);
                    sleep(robot.SERVO_SLEEP);

                    // RING 3 - BOTTOM RING
                    // Raise the elevator so that the bottom ring of the 3 is in shooting position
                    robot.ringLoader.setTargetPosition(robot.THIRD_ELEVATOR_RAISE);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    // Continue raising the elevator until it is at the proper height for the bottom ring
                    while (robot.ringLoader.isBusy()) {
                        //drive
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Engage the servo to push the bottom ring to the the shooting wheel to shot the ring and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_FORWARD);
                    sleep(robot.SERVO_SLEEP);

                    // Return the servo to the start position and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_BACK);
                    sleep(robot.SERVO_SLEEP);

                    // Lower the elevator to the original/bottom position
                    robot.ringLoader.setTargetPosition(robot.BOTTOM_ELEVATOR);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.tiltElevatorServo.setPosition(robot.TILT_SERVO_DOWN);

                    // Continue lower the elevator until it has reached the bottom
                    while (robot.ringLoader.isBusy()) {
                        // Drive
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Stop the two motors/wheels that shoot the rings
                    robot.shooterFront.setPower(robot.MOTOR_STOP);
                    robot.shooterBack.setPower(robot.MOTOR_STOP);

                    robot.driveLeftEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_B_SHIFT_LEFT, robot.DRIVE_SPEED);

                    robot.rotateRobot(-80);

                    robot.leftRingBlock.setPosition(robot.RING_BLOCK_OPEN);

                    robot.driveLeftEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_LONG_GO_TO_WHITE_LINE, robot.DRIVE_SPEED);

                    // lock wobble goal in up position
                    robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_UP);
                    robot.leftRightWobble.setPosition(robot.WOBBLE_SERVO_CLOSE);
                }
                 */
            }

            // For Scenario C
            if (ringsDetected == 4) {

                // Drive to drop off point for C
                robot.driveRightEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.C_DELIVER_WOBBLE, robot.DRIVE_SPEED);

                // align the robot to 0 degrees
                robot.angleAlignRobot();

                // Align robot to wall to get to 5 inches away
                if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 5) {
                    robot.driveForwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 5) {
                        telemetry.addData("Toward Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                } else if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 5) {
                    robot.driveBackwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 5) {
                        telemetry.addData("Away Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                }

                // Release wobble goal
                releaseWobbleGoal();

                // Back away from the wobble goal to return to the side of the field with the second wobble
                robot.driveLeftEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.C_BACKUP_FROM_SECOND_WOBBLE, robot.DRIVE_SPEED);

                // Align robot to wall to ensure 7.5 inches away
                if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 7.5) {
                    robot.driveForwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) > 7.5) {
                        telemetry.addData("Toward Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                } else if (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 7.5) {
                    robot.driveBackwardNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                    // align robot from side wall
                    while (robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < 7.5) {
                        telemetry.addData("Away Front Sensor Distance During: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                    robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                }

                // Rotate the robot 90 degrees to the right to face the wobble grabber towards the second wobble goal
                robot.rotateRobot(-90);

                // Align the robot to -90 degrees
                robot.angleAlignRobot();

                // Go to the second wobble
                robot.driveRightEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.C_SHIFT_TO_SECOND_WOBBLE, robot.DRIVE_SPEED);

                // sleep(robot.NORMAL_SLEEP);

                // Pick up the wobble goal
                pickupWobbleGoal();

                sleep(robot.LONG_SLEEP);

                // Robot goes to put down wobble if it is picked up
                // if (robot.wobbleTouchSensor.isPressed() == Boolean.TRUE) {

                    // Align robot to wall to ensure 9 inches away when backing away from the 2nd wobble position
                    if (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) > 9) {
                        robot.driveLeftNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                        // align robot from side wall
                        while (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) > 9) {
                            telemetry.addData("Toward Left Sensor Distance During: ", robot.leftDistanceSensor.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                        }
                        robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                    } else if (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) < 9) {
                        robot.driveRightNoEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.DIST_SENSOR_SPEED);
                        // align robot from side wall
                        while (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) < 9) {
                            telemetry.addData("Away Left Sensor Distance During: ", robot.leftDistanceSensor.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                        }
                        robot.stopRobot(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
                    }

                    // Align the robot angle to -90 degrees
                    robot.angleAlignRobot();

                    // Angle the robot to the left 85 degrees so that it is slightly angled for delivery of the  second wobble to C
                    robot.rotateRobot(85);

                    // Go back to the C zone
                    robot.driveRightEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.C_BACK_TO_C_ZONE, robot.DRIVE_SPEED);

                    // Release wobble goal
                    releaseWobbleGoal();

                    // Park on the white line
                    robot.driveLeftEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.C_GO_TO_WHITE_LINE, robot.DRIVE_SPEED);

                    // lock wobble goal in up position at the end of autonomous
                    pickupWobbleGoal();

                    // End autonomous by parking such that the robot is angled for shooting rings
                    robot.rotateRobot(-92);
                /*}

                // Robot shoots rings if it misses the wobble
                else {

                    // Robot rotates so the shooter faces the ring goal
                    // robot.rotateRobot(178);

                    // Robot drives close to the white line
                    robot.driveForwardEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_SHORT_GO_TO_SHOOTING, robot.DRIVE_SPEED);

                    robot.rotateRobot(-10);



                    // Setup the ring loader elevator
                    robot.ringLoader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.ringLoader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    // Start the two motors/wheels that shoot the rings
                    robot.shooterFront.setPower(robot.FAST_SHOOTING);
                    robot.shooterBack.setPower(robot.FAST_SHOOTING);

                    // RING 1 - TOP RING
                    // Raise the elevator so that the top ring of the 3 is in shooting position
                    robot.ringLoader.setTargetPosition(robot.FIRST_ELEVATOR_RAISE);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // Continue raising the elevator until it is at the proper height for the top ring
                    while (robot.ringLoader.isBusy()) {
                        //drive
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Engage the servo to push the top ring to the the shooting wheel to shot the ring and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_FORWARD);
                    sleep(robot.SERVO_SLEEP);

                    // Return the servo to the start position and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_BACK);
                    sleep(robot.SERVO_SLEEP);

                    // RING 2 - MIDDLE RING
                    // Raise the elevator so that the middle ring of the 3 is in shooting position
                    robot.ringLoader.setTargetPosition(robot.SECOND_ELEVATOR_RAISE);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    // Continue raising the elevator until it is at the proper height for the middle ring
                    while (robot.ringLoader.isBusy()) {
                        //drive
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Engage the servo to push the middle ring to the the shooting wheel to shot the ring and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_FORWARD);
                    sleep(robot.SERVO_SLEEP);

                    // Return the servo to the start position and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_BACK);
                    sleep(robot.SERVO_SLEEP);

                    // RING 3 - BOTTOM RING
                    // Raise the elevator so that the bottom ring of the 3 is in shooting position
                    robot.ringLoader.setTargetPosition(robot.THIRD_ELEVATOR_RAISE);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    // Continue raising the elevator until it is at the proper height for the bottom ring
                    while (robot.ringLoader.isBusy()) {
                        //drive
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Engage the servo to push the bottom ring to the the shooting wheel to shot the ring and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_FORWARD);
                    sleep(robot.SERVO_SLEEP);

                    // Return the servo to the start position and wait until it is in position
                    robot.loadRingServo.setPosition(robot.LOADER_SERVO_BACK);
                    sleep(robot.SERVO_SLEEP);

                    // Lower the elevator to the original/bottom position
                    robot.ringLoader.setTargetPosition(robot.BOTTOM_ELEVATOR);
                    robot.ringLoader.setPower(robot.ELEVATOR_SPEED);
                    robot.ringLoader.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.tiltElevatorServo.setPosition(robot.TILT_SERVO_DOWN);

                    // Continue lower the elevator until it has reached the bottom
                    while (robot.ringLoader.isBusy()) {
                        // Drive
                    }
                    // Once the elevator is at the target height, stop the lift
                    robot.ringLoader.setPower(robot.MOTOR_STOP);

                    // Stop the two motors/wheels that shoot the rings
                    robot.shooterFront.setPower(robot.MOTOR_STOP);
                    robot.shooterBack.setPower(robot.MOTOR_STOP);

                    robot.driveLeftEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_B_SHIFT_LEFT, robot.DRIVE_SPEED);

                    robot.rotateRobot(-80);

                    robot.leftRingBlock.setPosition(robot.RING_BLOCK_OPEN);

                    robot.driveLeftEncoder(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive, robot.TEST_LONG_GO_TO_WHITE_LINE, robot.DRIVE_SPEED);

                    // lock wobble goal in up position
                    robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_UP);
                    robot.leftRightWobble.setPosition(robot.WOBBLE_SERVO_CLOSE);
                }
                 */
            }

            // sleep at the end to ensure autonomous does not start over
            sleep(30000);
        }

    }

    public static class RingDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the ring position
         */
        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);


        // ******* Retro Updates Start

        // This defines the position and size of the green detection box

        // Using a "Sideways Right" orientation above...
        // Top Left Anchor Point X = Distance from bottom of display area
        // Top Left Anchor Point Y = Distance from Left edge of display area
        // x = 110 and y = 115 seem to work the best
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(110, 20);


        // Using a "Sideways Right" orientation above...
        // REGION_WIDTH is actually the height
        // REGION_HEIGHT is actually the width

        // Height of the green detection area (default was 25)
        // 25 seems to work the best
        static final int REGION_WIDTH = 25;

        // Max value is the 240 with the starting coordinate above added in
        // 35 seems to work the best
        static final int REGION_HEIGHT = 200;

        // changed from 150
        final int FOUR_RING_THRESHOLD = 132;
        // changed from 126
        final int ONE_RING_THRESHOLD = 126;
        // ******** Retro Updates End


        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        // Retro added
        int numberofRings = 0;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);


            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
                numberofRings = 4;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
                numberofRings = 1;
            } else {
                position = RingPosition.NONE;
                numberofRings = 0;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }

        public int ringCount() {
            return numberofRings;
        }

    }

    // Release wobble goal
    void releaseWobbleGoal() {
        robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_DOWN);
        sleep(robot.LONG_SLEEP);
        robot.leftRightWobble.setPosition(robot.WOBBLE_SERVO_OPEN);
        sleep(robot.NORMAL_SLEEP);
    }

    // Release wobble goal fast
    void finalReleaseWobbleGoal() {
        robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_DOWN);
        sleep(robot.SHORT_SLEEP);
        robot.leftRightWobble.setPosition(robot.WOBBLE_SERVO_OPEN);
        sleep(robot.SHORT_SLEEP);
    }

    // Pick up the wobble goal
    void pickupWobbleGoal() {
        robot.leftRightWobble.setPosition(robot.WOBBLE_SERVO_CLOSE);
        sleep(robot.LONG_SLEEP);
        robot.forwardBackWobble.setPosition(robot.WOBBLE_SERVO_UP);
        sleep(robot.NORMAL_SLEEP);
    }

}
