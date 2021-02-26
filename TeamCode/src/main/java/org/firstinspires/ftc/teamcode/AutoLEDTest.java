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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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


import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RRHardwarePushbot;
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

@Autonomous (name = "Test of LEDs", group = "Concept")
public class  AutoLEDTest extends LinearOpMode {
    // Declare OpMode members
    org.firstinspires.ftc.teamcode.RRHardwarePushbot robot = new RRHardwarePushbot();

    DistanceSensor distanceSensor01 = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        LEDRiver ledRiver = hardwareMap.get(LEDRiver.IMPL, "led");
        ledRiver.setMode(LEDRiver.Mode.SOLID);
        ledRiver.setLEDMode(LEDRiver.LEDMode.RGB);
        ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_24);


        waitForStart();

        ledRiver.setBrightness(0.01);

        ledRiver.setHide(false);
        ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_24);
        ledRiver.setMode(LEDRiver.Mode.INDIVIDUAL);

        //START
        /*
        ledRiver.setColor(0, Color.TRANSPARENT).apply();
        ledRiver.setColor(1, Color.TRANSPARENT).apply();
        ledRiver.setColor(2, Color.TRANSPARENT).apply();
        ledRiver.setColor(3, Color.TRANSPARENT).apply();
        ledRiver.setColor(4, Color.TRANSPARENT).apply();
        ledRiver.setColor(5, Color.TRANSPARENT).apply();
        ledRiver.setColor(6, Color.TRANSPARENT).apply();
        ledRiver.setColor(7, Color.TRANSPARENT).apply();
        ledRiver.setColor(8, Color.TRANSPARENT).apply();
        ledRiver.setColor(9, Color.TRANSPARENT).apply();
        ledRiver.setColor(10, Color.TRANSPARENT).apply();
        ledRiver.setColor(11, Color.TRANSPARENT).apply();
        ledRiver.setColor(12, Color.TRANSPARENT).apply();
        ledRiver.setColor(13, Color.TRANSPARENT).apply();
        ledRiver.setColor(14, Color.TRANSPARENT).apply();
        ledRiver.setColor(15, Color.TRANSPARENT).apply();
*/
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
        ledRiver.setColor(14, Color.BLUE).apply();
        ledRiver.setColor(15, Color.BLUE).apply();



        ledRiver.setColor(15, Color.YELLOW).apply();
        sleep(2000);

        ledRiver.setColor(15, Color.BLUE).apply();
        ledRiver.setColor(13, Color.YELLOW).apply();
        sleep(2000);

        ledRiver.setColor(13, Color.BLUE).apply();
        ledRiver.setColor(10, Color.YELLOW).apply();
        sleep(2000);

        ledRiver.setColor(10, Color.BLUE).apply();
        ledRiver.setColor(7, Color.GREEN).apply();
        ledRiver.setColor(8, Color.GREEN).apply();

        sleep(30000);






        ledRiver.setColor(0, Color.HSVToColor(new float[] {40,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "0");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(1, Color.HSVToColor(new float[] {80,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "1");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(2, Color.HSVToColor(new float[] {120,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "2");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(3, Color.HSVToColor(new float[] {160,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "3");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(4, Color.HSVToColor(new float[] {200,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "4");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(5, Color.HSVToColor(new float[] {240,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "5");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(6, Color.HSVToColor(new float[] {280,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "6");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(7, Color.HSVToColor(new float[] {320,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "7");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(8, Color.HSVToColor(new float[] {359,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "8");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(9, Color.HSVToColor(new float[] {80,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "9");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(10, Color.HSVToColor(new float[] {120,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "10");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(11, Color.HSVToColor(new float[] {160,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "11");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(12, Color.HSVToColor(new float[] {200,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "12");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(13, Color.HSVToColor(new float[] {240,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "13");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(14, Color.HSVToColor(new float[] {280,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "14");
        telemetry.update();
        sleep(1000);

        ledRiver.setColor(15, Color.HSVToColor(new float[] {320,1,1}));
        ledRiver.apply();
        telemetry.addData("#", "15");
        telemetry.update();
        sleep(1000);

        ledRiver.reset();

        sleep(20000);



    }
}
