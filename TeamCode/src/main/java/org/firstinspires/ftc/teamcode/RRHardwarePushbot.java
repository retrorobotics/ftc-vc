package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.

 */

public class RRHardwarePushbot
{
    // toggle telemetry data
    public boolean DEBUG_MODE_ON = true;
    public boolean FRONT_DRIVE = true;
    public boolean TOUCH_SENSOR_PRESSED = false;
    public boolean xPressed = false;
    public boolean rightTrigger1Pressed = false;
    public boolean leftTriggerPressed = false;
    public boolean leftTrigger1Pressed = false;
    public boolean rightTriggerPressed = false;

    // number of checks for ring detection
    int MAX_DETECTION_CHECK = 3;

    // servo open and close values for wobble goals
    public double WOBBLE_SERVO_OPEN = -1;
    public double WOBBLE_SERVO_CLOSE = 1;
    public double WOBBLE_SERVO_DOWN = 0.75;
    public double WOBBLE_SERVO_VERY_DOWN = 1;
    public double WOBBLE_SERVO_UP = -1;

    // Servo up and down for elevator tilt servo
    public double TILT_SERVO_UP = 0.5;
    public double TILT_SERVO_DOWN = 0;

    // servo open and close values for ring loader
    public int LOADER_SERVO_FORWARD = -1;
    public int LOADER_SERVO_BACK = 1;

    // Servo open and close values for ring blocks
    public double RING_BLOCK_OPEN = 0.6;
    public double RIGHT_RING_BLOCK_OPEN = 0.5;
    public double RING_BLOCK_CLOSE = 0;
    public double LEFT_BLOCK_POSITION = 0;
    public double RIGHT_BLOCK_POSITION = 0;

    // sleep values
    public long SERVO_SLEEP = 1000;
    public long MOTOR_SLEEP = 250;
    public long SHORT_SLEEP = 250;
    public long NORMAL_SLEEP = 500;
    public long LONG_SLEEP = 750;
    public long VERY_LONG_SLEEP = 2000;

    // target distance to align robot to wall
    public double DISTANCE_TO_WALL = 5.0;

    // angle of the robot at the start
    public double STARTING_ROBOT_ANGLE = 0;

    // angle of the robot to the target, 180 would be perpendicular
    public double SHOOTING_ANGLE = 160;

    // Teleop speed and orientation
    public double NORMAL_TELEOP_SPEED = .85;
    public double SLOW_TELEOP_SPEED = .5;
    public double FAST_TELEOP_SPEED = 1;
    public double FORWARD_ORIENTATION = 1;
    public double BACKWARD_ORIENTATION = -1;


    // Elevator tick value
    public int runningTicks = 0;
    public int FIRST_ELEVATOR_RAISE = -1670;
    public int SECOND_ELEVATOR_RAISE = FIRST_ELEVATOR_RAISE - 350;
    public int THIRD_ELEVATOR_RAISE = SECOND_ELEVATOR_RAISE - 290;
    public int BOTTOM_ELEVATOR = 0;
    public int elevatorPosition = 0;

    // Drive wheel speed
    public double FAST_SHOOTING = 1.0;
    public double ELEVATOR_SPEED = 1.0;
    public double INTAKE_SPEED = 0.9;
    public double REVERSE_MOTOR = -1;
    public double SLOW_SHOOTING = 0.8;
    public double DRIVE_SPEED = 0.9;
    public double B_DRIVE_SPEED = 1.0;
    public double LIFT_UP = -0.8;
    public double LIFT_DOWN = 0.8;
    public double SLOW_DRIVE = 0.5;
    public double TURN_SPEED = 0.3; // was 0.2
    public double MOTOR_STOP = 0.0;
    public double DIST_SENSOR_SPEED = 0.4;
    public double B_DIST_SENSOR_SPEED = 0.6;

    // Autonomous robot rotate speeds
    public double ROTATE_FAST_SPEED = 0.70;
    public double ROTATE_SLOW_SPEED = 0.20;

    // Max RGB values to detect white line
    public double WHITE_LINE_MAX_RED = 600;
    public double WHITE_LINE_MAX_GREEN = 600;
    public double WHITE_LINE_MAX_BLUE = 600;

    // Max RGB values to detect blue line
    public double BLUE_LINE_MAX_RED = 200;
    public double BLUE_LINE_MAX_GREEN = 200;
    public double BLUE_LINE_MAX_BLUE = 600;

    // Motors
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor robotIntake = null;
    public DcMotor shooterFront = null;
    public DcMotor shooterBack = null;
    public DcMotor ringLoader = null;

    // Servos
    public Servo leftRightWobble = null;
    public Servo forwardBackWobble = null;
    public Servo loadRingServo = null;
    public Servo tiltElevatorServo = null;
    public Servo leftRingBlock = null;
    public Servo rightRingBlock = null;

    // Color Sensors
    public ColorSensor backColorSensor = null;

    // Touch Sensors
    public TouchSensor wobbleTouchSensor = null;
    public TouchSensor frontTouchSensor = null;
    public TouchSensor bottomTouchSensor = null;

    // Distance Sensors
    public DistanceSensor backDistanceSensor = null;
    public DistanceSensor rightDistanceSensor = null;
    public DistanceSensor frontDistanceSensor = null;
    public DistanceSensor leftDistanceSensor = null;

    // Control Hub Gyro Sensor
    public BNO055IMU imu;
    public Orientation currentAngles;


    // Web Cam
    public OpenCvInternalCamera phoneCam;

    // LED's
    LEDRiver ledRiver = null;

    // ENCODER DRIVING CONSTANTS
    // length of 1 tile in inches - 22.75 inches according to game manual
    double TILE_SIZE = 22.75;
    // diameter of the wheel in inches
    double WHEEL_DIAMETER = 2.95;
    // circumference of wheel - ie. the distance covered by 1 revolution of the wheels
    double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER *3.14;

    public double POWERSHOT_PERFECT = 19;

    public int LED_ON = 0;

    // Percentage values for LED's
    public double PERFECT_PERCENTAGE = 0.02;

    public double SLIGHTLY_TO_LEFT_LOW_PERCENTAGE = 0.03;
    public double SLIGHTLY_TO_LEFT_HIGH_PERCENTAGE = 0.08;

    public double SLIGHTLY_TO_RIGHT_LOW_PERCENTAGE = 0.03;
    public double SLIGHTLY_TO_RIGHT_HIGH_PERCENTAGE = 0.08;

    public double TO_THE_LEFT_LOW_PERCENTAGE = 0.09;
    public double TO_THE_LEFT_HIGH_PERCENTAGE = 0.14;

    public double TO_THE_RIGHT_LOW_PERCENTAGE = 0.09;
    public double TO_THE_RIGHT_HIGH_PERCENTAGE = 0.14;

    public double EXTREME_TO_THE_LEFT_LOW_PERCENTAGE = 0.15;
    public double EXTREME_TO_THE_LEFT_HIGH_PERCENTAGE = 0.2;

    public double EXTREME_TO_THE_RIGHT_LOW_PERCENTAGE = 0.15;
    public double EXTREME_TO_THE_RIGHT_HIGH_PERCENTAGE = 0.2;

    public double OUT_OF_RANGE_PERCENTAGE = 0.21;







    // number of motor tickets for a single revolution of the motor/wheel driving front and back
    // Testing results with 22.75 target:

    int FR_BK_MOTOR_TICK_COUNTS = 650;

    int RT_LF_MOTOR_TICK_COUNTS = 500;

    // Calculate the amount of turns on the wheels over a distance of 1 tile
    double ROTATIONS_FOR_ONE_TILE = TILE_SIZE / WHEEL_CIRCUMFERENCE;

    // number of motor ticks used for encoder driving to drive 1 tile length
    public int FR_BK_ONE_TILE_MOTOR_TICKS = (int) (ROTATIONS_FOR_ONE_TILE * FR_BK_MOTOR_TICK_COUNTS);
    public int RT_LF_ONE_TILE_MOTOR_TICKS = (int) (ROTATIONS_FOR_ONE_TILE * RT_LF_MOTOR_TICK_COUNTS);

    // A zone autonomous encoder values
    public int A_DELIVER_WOBBLE = 3450;
    public int A_BACKUP_FROM_WOBBLE = 400;
    public int A_SHIFT_RIGHT = 1050;
    public int A_MOVE_TO_WHITE_LINE = 950;

    public int TEST_A_BACKUP_FROM_WOBBLE = 2600;
    public int TEST_A_SHIFT_TO_SECOND_WOBBLE = 875;
    public int TEST_A_SHIFT_LEFT = 800;
    public int TEST_A_BACK_TO_WHITE_LINE = 2500;
    public int TEST_A_GO_TO_WHITE_LINE = 850;
    public int TEST_A_SHIFT_RIGHT = 1100;

    // B zone autonomous encoder values
    public int B_DELIVER_WOBBLE = 4500;

    // used to be 1250
    public int B_MOVE_TO_WHITE_LINE = 1100;
    public int B_BACKUP_FROM_WOBBLE = 1500;
    public int B_MOVE_UP_TO_WHITE_LINE = 950;

    public int RING_B_MOVE_TO_WHITE_LINE = 1400;
    public int RING_B_DELIVER_WOBBLE = 4400;

    public int TEST_B_BACKUP_FROM_WOBBLE = 1250;
    public int TEST_B_BACK_TO_SECOND_WOBBLE = 3825;
    public int TEST_B_SHIFT_TO_SECOND_WOBBLE = 950;
    public int TEST_B_SHIFT_LEFT = 900;
    public int TEST_B_BACK_TO_B_ZONE = 3450;
    public int TEST_B_GO_TO_B_ZONE = 1375;
    public int TEST_B_GO_TO_WHITE_LINE = 550;

    // C zone autonomous encoder values
    public int C_DELIVER_WOBBLE = 5950;
    public int C_BACKUP_FROM_WOBBLE = 3000;
    public int C_MOVE_TO_RIGHT_LINE = 950;
    public int C_SHIFT_RIGHT = 1150;
    public int C_BACKUP_FROM_SECOND_WOBBLE = 5180;
    public int C_SHIFT_TO_SECOND_WOBBLE = 1000;
    public int C_SHIFT_LEFT = 800;
    public int C_BACK_TO_C_ZONE = 4700;
    public int C_GO_TO_WHITE_LINE = 1700;

    public int TEST_GO_TO_SHOOTING = 2500;
    public int TEST_SHORT_GO_TO_SHOOTING = 500;
    public int TEST_LONG_GO_TO_WHITE_LINE = 2500;
    public int TEST_GO_TO_WHITE_LINE = 700;

    // Sets motors to drive the robot backward
    public void driveBackwardNoEncoder(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, double driveSpeed) {
        // make sure we are running without encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set the drive speed and direction of motors
        leftFront.setPower(-driveSpeed);
        rightFront.setPower(driveSpeed);
        leftBack.setPower(-driveSpeed);
        rightBack.setPower(driveSpeed);
    }

    // Sets motors to drive the robot backward
    public void driveBackwardEncoder(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, int tickDistance, double driveSpeed) {
        // this will set the tick count back to 0
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the distance - this is where you set +/- for forward and reverse of motor
        leftFront.setTargetPosition(-tickDistance);
        rightFront.setTargetPosition(tickDistance);
        leftBack.setTargetPosition(-tickDistance);
        rightBack.setTargetPosition(tickDistance);

        // set the speed
        leftFront.setPower(driveSpeed);
        rightFront.setPower(driveSpeed);
        leftBack.setPower(driveSpeed);
        rightBack.setPower(driveSpeed);

        // set the mode to run until the distance is achieved
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // continue driving until the target distance is reached
        while ((leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            //drive
        }
        // Once the robot is at the target distance, stop the robot
        stopRobot(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
    }

    // Sets motors to drive the robot forward
    public void driveForwardNoEncoder (DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, double driveSpeed) {
        // make sure we are running without encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set the drive speed and direction of motorsleftFront.setPower(driveSpeed);
        leftFront.setPower(driveSpeed);
        rightFront.setPower(-driveSpeed);
        leftBack.setPower(driveSpeed);
        rightBack.setPower(-driveSpeed);
    }

    // Sets motors to drive the robot forward
    public void driveForwardEncoder(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, int tickDistance, double driveSpeed) {
        // this will set the tick count back to 0
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the distance - this is where you set +/- for forward and reverse of motor
        leftFront.setTargetPosition(tickDistance);
        rightFront.setTargetPosition(-tickDistance);
        leftBack.setTargetPosition(tickDistance);
        rightBack.setTargetPosition(-tickDistance);

        // set the speed
        leftFront.setPower(driveSpeed);
        rightFront.setPower(driveSpeed);
        leftBack.setPower(driveSpeed);
        rightBack.setPower(driveSpeed);

        // set the mode to run until the distance is achieved
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // continue driving until the target distance is reached
        while ((leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            //drive
        }
        // Once the robot is at the target distance, stop the robot
        stopRobot(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
    }

    // Sets motors to drive the robot left
    public void driveLeftNoEncoder (DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, double driveSpeed) {
        // make sure we are running without encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set the drive speed and direction of motorsleftFront.setPower(-driveSpeed);
        leftFront.setPower(-driveSpeed);
        rightFront.setPower(-driveSpeed);
        leftBack.setPower(driveSpeed);
        rightBack.setPower(driveSpeed);
    }

    // Sets motors to drive the robot left
    public void driveLeftEncoder(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, int tickDistance, double driveSpeed) {
        // this will set the tick count back to 0
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the distance - this is where you set +/- for forward and reverse of motor
        leftFront.setTargetPosition(-tickDistance);
        rightFront.setTargetPosition(-tickDistance);
        leftBack.setTargetPosition(tickDistance);
        rightBack.setTargetPosition(tickDistance);

        // set the speed
        leftFront.setPower(driveSpeed);
        rightFront.setPower(driveSpeed);
        leftBack.setPower(driveSpeed);
        rightBack.setPower(driveSpeed);

        // set the mode to run until the distance is achieved
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // continue driving until the target distance is reached
        while ((leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            //drive
        }
        // Once the robot is at the target distance, stop the robot
        stopRobot(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
    }

    // Sets motors to drive the robot right
    public void driveRightNoEncoder (DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, double driveSpeed) {
        // make sure we are running without encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set the drive speed and direction of motors
        leftFront.setPower(driveSpeed);
        rightFront.setPower(driveSpeed);
        leftBack.setPower(-driveSpeed);
        rightBack.setPower(-driveSpeed);
    }

    // Sets motors to drive the robot right
    public void driveRightEncoder(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, int tickDistance, double driveSpeed) {
        // this will set the tick count back to 0
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the distance - this is where you set +/- for forward and reverse of motor
        leftFront.setTargetPosition(tickDistance);
        rightFront.setTargetPosition(tickDistance);
        leftBack.setTargetPosition(-tickDistance);
        rightBack.setTargetPosition(-tickDistance);

        // set the speed
        leftFront.setPower(driveSpeed);
        rightFront.setPower(driveSpeed);
        leftBack.setPower(driveSpeed);
        rightBack.setPower(driveSpeed);

        // set the mode to run until the distance is achieved
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // continue driving until the target distance is reached
        while ((leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            //drive
        }
        // Once the robot is at the target distance, stop the robot
        stopRobot(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
    }

    // Sets motors to rotate the robot left
    public void moveLeftNoEncoder (DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, double driveSpeed) {
        // make sure we are running without encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set the drive speed and direction of motors
        leftFront.setPower(-driveSpeed);
        rightFront.setPower(-driveSpeed);
        leftBack.setPower(-driveSpeed);
        rightBack.setPower(-driveSpeed);
    }


    // Sets motors to rotate the robot left
    public void moveLeftEncoder(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, int tickDistance, double driveSpeed) {
        // this will set the tick count back to 0
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the distance - this is where you set +/- for forward and reverse of motor
        leftFront.setTargetPosition(-tickDistance);
        rightFront.setTargetPosition(-tickDistance);
        leftBack.setTargetPosition(-tickDistance);
        rightBack.setTargetPosition(-tickDistance);

        // set the speed
        leftFront.setPower(driveSpeed);
        rightFront.setPower(driveSpeed);
        leftBack.setPower(driveSpeed);
        rightBack.setPower(driveSpeed);

        // set the mode to run until the distance is achieved
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Sets motors to rotate the robot right
    public void moveRightNoEncoder (DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, double driveSpeed) {
        // make sure we are running without encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set the drive speed and direction of motors
        leftFront.setPower(driveSpeed);
        rightFront.setPower(driveSpeed);
        leftBack.setPower(driveSpeed);
        rightBack.setPower(driveSpeed);
    }

    // Sets motors to rotate the robot left
    public void moveRightEncoder(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, int tickDistance, double driveSpeed) {
        // this will set the tick count back to 0
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the distance - this is where you set +/- for forward and reverse of motor
        leftFront.setTargetPosition(tickDistance);
        rightFront.setTargetPosition(tickDistance);
        leftBack.setTargetPosition(tickDistance);
        rightBack.setTargetPosition(tickDistance);

        // set the speed
        leftFront.setPower(driveSpeed);
        rightFront.setPower(driveSpeed);
        leftBack.setPower(driveSpeed);
        rightBack.setPower(driveSpeed);

        // set the mode to run until the distance is achieved
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // stops the robot motors
    // use for driving with or without encoders
    public void stopRobot (DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        leftFront.setPower(MOTOR_STOP);
        rightFront.setPower(MOTOR_STOP);
        leftBack.setPower(MOTOR_STOP);
        rightBack.setPower(MOTOR_STOP);
    }

    // move the robot a give number of degrees
    // <0 right
    // >0 left
    public void rotateRobot (float degrees) {
        float movedAmount = 0;
        float absDegrees = 0;
        Orientation currentAngles;
        float prevAng = 0;
        float currentMove = 0;
        boolean firstTimeInLoop = Boolean.TRUE;

        // need degrees as a positive number for the while statement
        if(degrees < 0) {
            absDegrees = -degrees;
        }
        else {
            absDegrees = degrees;
        }

        // move right
        if (degrees < 0 ) {

            // setup the initial angles for compare
            currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            prevAng = currentAngles.firstAngle;
            currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // move the robot to the right
            moveRightNoEncoder(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, TURN_SPEED);

            // continue moving right until the robot turns the number of degrees defined
            while (movedAmount < absDegrees) {
                currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (prevAng >= 0) {
                    movedAmount = movedAmount + (prevAng - currentAngles.firstAngle);
                }
                if ((currentAngles.firstAngle >= 0) && (prevAng < 0)) {
                    movedAmount = movedAmount + ((180 - currentAngles.firstAngle) + (180 + prevAng));
                }
                if ((currentAngles.firstAngle < 0) && (prevAng < 0)) {
                    movedAmount = movedAmount + (prevAng - currentAngles.firstAngle);
                }

                // update the previous angle
                prevAng = currentAngles.firstAngle;
            }
        }
        else {
            // move left
            if (degrees > 0) {

                // setup the initial angles for compare
                currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                prevAng = currentAngles.firstAngle;
                currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                // move the robot to the left
                moveLeftNoEncoder(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, TURN_SPEED);

                // continue moving right until the robot turns the number of degrees defined
                while (movedAmount < absDegrees) {
                    currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    if (currentAngles.firstAngle >= 0) {
                        movedAmount = movedAmount + (currentAngles.firstAngle - prevAng);
                    }
                    if ((currentAngles.firstAngle <0) && (prevAng >= 0)) {
                        movedAmount = movedAmount + ((180 + currentAngles.firstAngle) + (180 - prevAng));
                    }
                    if ((currentAngles.firstAngle <0) && (prevAng < 0)) {
                        movedAmount = movedAmount + (currentAngles.firstAngle - prevAng);
                    }

                    // update the previous angle
                    prevAng = currentAngles.firstAngle;
                }
            }
        }
        // stop the wheels after robot has turned the proper number of degrees
        stopRobot(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
    }

    // aligns the robot according to the 90 degree angle it is closed to
    // range is +/-  3 to 10 degrees (does not align within 3 degrees)
    // -10 to 10, excluding -3 to 3....align to 0 degrees
    // 80 to 100, excluding 87 to 90.....align to 90 degrees
    // 170 to -170 excluding 187 to - 187....align to 180 degrees
    // -100 to -80 excluding -93 to -87....align to - 90 degrees
    void angleAlignRobot() {
        float degreesOff = 0;
        float degreesToAdjust = 0;

        // get the current angle of the robot
        currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // change -0 and -180 to 0 and 180 respectively
        if ((currentAngles.firstAngle == -180) || (currentAngles.firstAngle == -0)) {
            currentAngles.firstAngle = currentAngles.firstAngle * -1;
        }
        // -10 to 10, determine how much to adjust to 0 degrees
        if ((currentAngles.firstAngle >= -10) && (currentAngles.firstAngle <= 10)) {
            // determine degrees from 0 degree target
            degreesOff = currentAngles.firstAngle - 0;
            // 80 to 100, determine how much to adjust to 90 degrees
        } else if ((currentAngles.firstAngle >= 80) && (currentAngles.firstAngle <= 100)) {
            // determine degrees from 90 degree target
            degreesOff = currentAngles.firstAngle - 90;
            // 170 to -170, determine how much to adjust to 180 degrees
        } else if ((currentAngles.firstAngle >= 170) || (currentAngles.firstAngle <= -170)) {
            // determine degrees from 90 degree target
            degreesOff = currentAngles.firstAngle - 180;
            // -100 to -80, determine how much to adjust to -90 degrees
        } else if ((currentAngles.firstAngle >= -100) && (currentAngles.firstAngle <= -80)) {
            // determine degrees from -90 degree target
            degreesOff = currentAngles.firstAngle + 90;
        }

        // determine degrees to adjust in the opposite direction within 5 degrees
        if ((degreesOff > 3 || degreesOff < -3)) {
            if (degreesOff < 0) {
                degreesToAdjust = (degreesOff + 3) * -1;
            } else {
                degreesToAdjust = (degreesOff - 3) * -1;
            }
            rotateRobot(degreesToAdjust);
        }
    }

    // Sets motors to drive the robot right in a graduated fashion
    public void graduatedDriveRightEncoder(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, int tickDistance, double driveSpeed) {

        double percentComplete;

        // this will set the tick count back to 0
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the distance - this is where you set +/- for forward and reverse of motor
        leftFront.setTargetPosition(tickDistance);
        rightFront.setTargetPosition(tickDistance);
        leftBack.setTargetPosition(-tickDistance);
        rightBack.setTargetPosition(-tickDistance);

        // set the initial speed for each wheel - 40% of full drive speed for smooth start
        leftFront.setPower(driveSpeed   * 0.4);
        rightFront.setPower(driveSpeed  * 0.4);
        leftBack.setPower(driveSpeed  * 0.4);
        rightBack.setPower(driveSpeed  * 0.4);

        // set the mode to run until the distance is achieved
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while ((leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy())) {
            // determine which current drive speed
            // only using the left front wheel for calculations
            percentComplete = (double)leftFront.getCurrentPosition() / (double)tickDistance;

            // <= 10% complete = 40% drive speed
            if (percentComplete <= 0.1) {
                leftFront.setPower(driveSpeed * 0.4);
                rightFront.setPower(driveSpeed * 0.4);
                leftBack.setPower(driveSpeed * 0.4);
                rightBack.setPower(driveSpeed * 0.4);
                // <= 20% complete = 60% drive speed
            } else if (percentComplete <= 0.2) {
                leftFront.setPower(driveSpeed * 0.6);
                rightFront.setPower(driveSpeed * 0.6);
                leftBack.setPower(driveSpeed * 0.6);
                rightBack.setPower(driveSpeed * 0.6);
                // <= 30% complete = 80% drive speed
            } else if (percentComplete <= 0.3) {
                leftFront.setPower(driveSpeed * 0.8);
                rightFront.setPower(driveSpeed * 0.8);
                leftBack.setPower(driveSpeed * 0.8);
                rightBack.setPower(driveSpeed * 0.8);
                // <= 70% complete = 100% drive speed
            } else if (percentComplete <= 0.7) {
                leftFront.setPower(driveSpeed);
                rightFront.setPower(driveSpeed);
                leftBack.setPower(driveSpeed);
                rightBack.setPower(driveSpeed);
                // <= 80% complete = 80% drive speed
            } else if (percentComplete <= 0.8) {
                leftFront.setPower(driveSpeed * 0.8);
                rightFront.setPower(driveSpeed * 0.8);
                leftBack.setPower(driveSpeed * 0.8);
                rightBack.setPower(driveSpeed * 0.8);
                // <= 90% complete = 60% drive speed
            } else if (percentComplete <= 0.9) {
                leftFront.setPower(driveSpeed * 0.6);
                rightFront.setPower(driveSpeed * 0.6);
                leftBack.setPower(driveSpeed * 0.6);
                rightBack.setPower(driveSpeed * 0.6);
                // <= 100% complete = 40% drive speed
            } else if (percentComplete <= 1.0) {
                leftFront.setPower(driveSpeed * 0.4);
                rightFront.setPower(driveSpeed * 0.4);
                leftBack.setPower(driveSpeed * 0.4);
                rightBack.setPower(driveSpeed * 0.4);
            }
        }
        // Once the robot is at the target distance, stop the robot
        stopRobot(leftFront, rightFront, leftBack, rightBack);
    }


    /* Constructor */
    public RRHardwarePushbot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {


    }
}