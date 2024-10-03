/*
 * Copyright (c) 2021 OpenFTC Team
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

package org.firstinspires.ftc.teamcode._Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Disabled
@Autonomous
public class LR_Auto extends LinearOpMode
{
    private DcMotor FLW;
    private DcMotor BLW;
    private DcMotor FRW;
    private DcMotor BRW;

    private DcMotor Slide;
    private DcMotor Intake;
    private Servo emo;
    private Servo girl;
    private CRServo bumper;
    private Servo adj;

    private Servo grimace;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // PIXELS AS IN THE HIT MOVIE WITH ADAM SANDLER
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;



    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 383.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.937;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // UNITS ARE METERS
    static final double SPEED = .5;
    OpenCvWebcam webcam;
    RedSkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;
    RedSkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = RedSkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default

    @Override
    public void runOpMode()
    {

        BRW = hardwareMap.dcMotor.get("BRW");
        BLW = hardwareMap.dcMotor.get("BLW");
        FRW = hardwareMap.dcMotor.get("FRW");
        FLW = hardwareMap.dcMotor.get("FLW");

        Intake = hardwareMap.dcMotor.get("Intake");
        Slide = hardwareMap.dcMotor.get("Slide");


        //Servos
        adj = hardwareMap.servo.get("adj");
        bumper = hardwareMap.crservo.get("bumper");
        emo = hardwareMap.servo.get("emo");
        girl = hardwareMap.servo.get("girl");
        grimace = hardwareMap.servo.get("grimace");

        FLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Ares"), cameraMonitorViewId);
        pipeline = new RedSkystoneDeterminationExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);


        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis) {
            case LEFT: {
                /* Your autonomous code */



                break;
            }

            case RIGHT: {
                telemetry.addLine("RIGHT");

                emo.setPosition(.19);
                girl.setPosition(.8);
                grimace.setPosition(0);

                encoderStrafe(.2,3,10);
                sleep(200);
                driveForward(.3,24,10);
                sleep(200);
                turn(10,22,-22);
                sleep(250);
                driveForward(.2,5,10);
                grimace.setPosition(1);
                sleep(250);
                driveForward(.3,-10,10);
                sleep(250);
                encoderStrafe(.2,28.5,10);
                sleep(250);

                turn(10,-38,38);
                sleep(250);
                driveForward(.3,-75,10);
                sleep(250);
                encoderStrafe(.2,19,10);
                sleep(300);

                score();
                sleep(300);
                driveForward(.1,-17,10);
                sleep(200);
                adj.setPosition(.85);
                Slide.setPower(0.05);
                sleep(150);
                driveForward(.1,8,10);
                Slide.setPower(0);





                break;
            }

            case CENTER:
                telemetry.addLine("CENTER");
            {
                emo.setPosition(.19);
                girl.setPosition(.8);
                grimace.setPosition(0);

                encoderStrafe(.2,5,10);
                sleep(200);
                driveForward(.2,28.5,10);
                grimace.setPosition(1);
                sleep(450);
                driveForward(.2, -28 , 10 );
                sleep(500);


                turn(10, -21,21);
                driveForward(.3,-68,10);
                sleep(300);
                encoderStrafe(.3,28,10);
                sleep(300);

                score();
                sleep(300);
                sleep(50);
                driveForward(.1,-17,10);
                sleep(200);
                adj.setPosition(.85);
                Slide.setPower(0.05);
                sleep(150);
                driveForward(.1,8,10);
                Slide.setPower(0);


                break;

            }
        }
    }




    public void driveForward(double speed, double dist, double timeout) {
        encoderDrive(speed, dist, dist, timeout);
    }

    public void turn(double timeout, double angleL, double angleR) {
        encoderDrive(.4, angleL, angleR, timeout);
    }

    public void encoderDrive(double speed, double distL, double distR, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;


        if (opModeIsActive()) {


            newLeftTarget = FLW.getCurrentPosition() + (int) (distL * COUNTS_PER_INCH);
            newRightTarget = FRW.getCurrentPosition() + (int) (distR * COUNTS_PER_INCH);
            FLW.setTargetPosition(-newLeftTarget);
            BLW.setTargetPosition(-newLeftTarget);
            FRW.setTargetPosition(newRightTarget);
            BRW.setTargetPosition(newRightTarget);


            FLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            FLW.setPower(Math.abs(speed));
            BLW.setPower(Math.abs(speed));
            FRW.setPower(Math.abs(speed));
            BRW.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FLW.isBusy() && FRW.isBusy()) &&
                    (FLW.isBusy() && FRW.isBusy())) {


                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        FLW.getCurrentPosition(),
                        BLW.getCurrentPosition(),
                        FRW.getCurrentPosition(),
                        BRW.getCurrentPosition());
                telemetry.update();
            }


            FLW.setPower(0);
            BLW.setPower(0);
            FRW.setPower(0);
            BRW.setPower(0);


            FLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderStrafe(double speed, double dist, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;


        if (opModeIsActive()) {


            newLeftTarget = FLW.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
            newRightTarget = FRW.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
            FLW.setTargetPosition(-newLeftTarget);
            BLW.setTargetPosition(newLeftTarget);
            FRW.setTargetPosition(-newRightTarget);
            BRW.setTargetPosition(newRightTarget);


            FLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            FLW.setPower(Math.abs(speed));
            BLW.setPower(Math.abs(speed));
            FRW.setPower(Math.abs(speed));
            BRW.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FLW.isBusy() && FRW.isBusy()) &&
                    (FLW.isBusy() && FRW.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        FLW.getCurrentPosition(),
                        BLW.getCurrentPosition(),
                        FRW.getCurrentPosition(),
                        BRW.getCurrentPosition());
                telemetry.update();
            }


            FLW.setPower(0);
            BLW.setPower(0);
            FRW.setPower(0);
            BRW.setPower(0);

            FLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void score() {
        Slide.setPower(.7);
        sleep(950);
        adj.setPosition(.68);
        Slide.setPower(.05);
        sleep(1500);
    }

    public void retract() {
        Slide.setPower(-.5);
        sleep(70);
        Slide.setPower(0);
    }
}