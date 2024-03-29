package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoTopRight", group="Robot")
//@Disabled
public class AutoTopRight extends LinearOpMode {
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
    @Override
    public void runOpMode() {
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
        waitForStart();

        //1 tile is 9
        encoderDrive(0.6, -9.5, -9.5, 69.420);
        encoderDrive(0.5, 8.75, -8.75, 69.420);
        encoderDrive(0.6, -15, -15, 69.420);

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
        sleep(1000);

        clarm.setPosition(0.6);
        sleep(750);
        claw.setPosition(0.7);
        sleep(750);
        clarm.setPosition(0);
        sleep(750);

        twostage.setTargetPosition(650);
        twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        twostage.setPower(.5);
        sleep(1000);

        out.setPosition(.1);
        sleep(1000);
        out.setPosition(0.7);
        sleep(1000);

        twostage.setTargetPosition(0);
        twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        twostage.setPower(.5);
        sleep(1000);
    }

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
}