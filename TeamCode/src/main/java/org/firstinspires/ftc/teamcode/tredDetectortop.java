/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "tredDetectortop", group = "Concept")

public class tredDetectortop extends LinearOpMode {

    private Servo camera = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor twostage=null;
    private ElapsedTime runtime=new ElapsedTime();
    private Servo claw = null;
    private Servo clarm = null;
    private Servo out=null;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "mods1.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/mods1.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)

    private static final String LABEL_FIRST_ELEMENT = "Rock";

    private static final String[] LABELS = {
            "Rock",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initTfod();

        frontLeftMotor = hardwareMap.dcMotor.get("leftfront");
        backLeftMotor = hardwareMap.dcMotor.get("leftback");
        frontRightMotor = hardwareMap.dcMotor.get("rightfront");
        backRightMotor = hardwareMap.dcMotor.get("rightback");
        twostage = hardwareMap.dcMotor.get("twostage");

        claw=hardwareMap.get(Servo.class, "claw");
        clarm=hardwareMap.get(Servo.class, "clarm");
        out = hardwareMap.get(Servo.class, "out");

        twostage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        twostage.setDirection(DcMotorSimple.Direction.REVERSE);

        twostage.setTargetPosition(0);
        twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        out.setPosition(.7);
        clarm.setPosition(1);
        claw.setPosition(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        twostage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();


                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */





    private void initTfod() {


        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "camra"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.90f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     *
     * @return
     */

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeftTarget=backLeftMotor.getCurrentPosition()+(int)(leftInches*COUNTS_PER_INCH);
            newBackRightTarget=backRightMotor.getCurrentPosition()+(int)(rightInches*COUNTS_PER_INCH);
            frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            frontRightMotor.setTargetPosition(newFrontRightTarget);
            backLeftMotor.setTargetPosition(newBackLeftTarget);
            backRightMotor.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftMotor.setPower(Math.abs(speed));
            frontRightMotor.setPower(Math.abs(speed));
            backLeftMotor.setPower(Math.abs(speed));
            backRightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy()) && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
                telemetry.addData("Running to",
                        " %7d :%7d",
                        newFrontLeftTarget,
                        newFrontRightTarget,
                        newBackLeftTarget,
                        newBackRightTarget);
                telemetry.update();
            }

            // Stop all motion;
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(-1);
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
            sleep(100);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }


    public void telemetryTfod() {

        camera = hardwareMap.get(Servo.class, "camera");

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // auto starts here



        long start = System.currentTimeMillis();

        telemetry.addData("runtime", getRuntime());
        if (getRuntime() <= 4 && currentRecognitions.size() == 0)  {
            camera.setPosition(.4);
            telemetry.update();
        }
        if (getRuntime() >= 4 && getRuntime() <= 8 && currentRecognitions.size() == 0)  {
            camera.setPosition(.5);
            telemetry.update();
        }
        if (getRuntime() <= 12 && getRuntime() >= 8 && currentRecognitions.size() == 0)  {
            camera.setPosition(.62);

            telemetry.update();
        }
        if (getRuntime() >= 9 && currentRecognitions.size() == 0) {
            telemetry.addData("something not goog", "");
            telemetry.update();
        }


        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            long position = Math.round(camera.getPosition() * 100);


            if (Math.round(position) == 40) {

                //net 9.75

                telemetry.addData("pos 2", "");
                encoderDrive(0.6, -2, -2, 69); //2
                encoderDrive(0.6, 17.75, -17.75, 69);
                clarm.setPosition(0.15);
                sleep(100);
                encoderDrive(0.5, 2, 2, 69); //2
                encoderDrive(0.5, -3, 3, 69);
                encoderDrive(0.5, 4, 4, 69);
                sleep(200);
                claw.setPosition(1);

                sleep(500);
                claw.setPosition(0);
                clarm.setPosition(200);
                sleep(100);

                encoderDrive(0.5, -4, -4, 69);
                encoderDrive(0.5, 3, -3, 69);
                encoderDrive(0.5, 5.75, 5.75, 69); //5.75
                encoderDrive(0.5, -8.875, 8.875, 69.420);
                encoderDrive(0.6, -16, -16, 69.420);

                clarm.setPosition(0);
                sleep(750);
                twostage.setTargetPosition(650);
                twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                twostage.setPower(.5);
                sleep(1000);

                out.setPosition(.1);
                sleep(1000);
                out.setPosition(.7);
                sleep(1000);

                twostage.setTargetPosition(0);
                twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                twostage.setPower(.5);
                sleep(30000);

            }
            if (Math.round(position) == 50) {

                //net 9.75

                telemetry.addData("pos 1", "");
                encoderDrive(0.6, -2, -2, 69); //2
                encoderDrive(0.6, 17.75, -17.75, 69);
                clarm.setPosition(0.15);
                sleep(100);
                encoderDrive(0.5, 6, 6, 69); //6
                sleep(200);
                claw.setPosition(1);

                sleep(500);
                claw.setPosition(0);
                clarm.setPosition(1);

                encoderDrive(0.6, 1.75, 1.75, 69.420); //1.75
                encoderDrive(0.5, -8.875, 8.875, 69.420);
                encoderDrive(0.6, -16, -16, 69.420);

                clarm.setPosition(0);
                sleep(750);
                twostage.setTargetPosition(650);
                twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                twostage.setPower(.5);
                sleep(1000);

                out.setPosition(.1);
                sleep(1000);
                out.setPosition(.7);
                sleep(1000);

                twostage.setTargetPosition(0);
                twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                twostage.setPower(.5);
                sleep(30000);

            }
            if (Math.round(position) == 62) {
                telemetry.addData("pos 0", "");

                //needs to travel net 18.75 forwards

                encoderDrive(0.6, -2, -2, 69); //2
                encoderDrive(0.6, -17.75, 17.75, 69);
                clarm.setPosition(0.15);
                sleep(100);
                encoderDrive(0.5, 4, 4, 69); //4
                encoderDrive(0.5, 4, -4, 69);
                //encoderDrive(0.5, 2, 2, 69);
                sleep(200);
                claw.setPosition(1);
                sleep(500);
                claw.setPosition(0);
                clarm.setPosition(200);
                sleep(100);
                //encoderDrive(0.5, -2, -2, 69);
                encoderDrive(0.5, -4, 4, 69); //turn sequence end

                encoderDrive(0.6, -4, -4, 69.420); //12.75
                encoderDrive(0.5, -8.875, 8.875, 69.420);
                encoderDrive(0.6, -11, -11, 69.420);
                encoderDrive(0.5, -8.875, 8.875, 69.420);
                encoderDrive(0.6, -10, -10, 69.420);
                encoderDrive(0.5, 8.875, -8.875, 69.420);
                encoderDrive(0.6, -3, -3, 69.420);

                clarm.setPosition(0);
                sleep(750);
                twostage.setTargetPosition(650);
                twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                twostage.setPower(.5);
                sleep(1000);

                out.setPosition(.1);
                sleep(1000);
                out.setPosition(.7);
                sleep(1000);

                twostage.setTargetPosition(0);
                twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                twostage.setPower(.5);
                sleep(30000);


            }
            telemetry.addData("", position);
            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            break;
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class



























