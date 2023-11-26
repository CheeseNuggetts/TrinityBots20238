package org.firstinspires.ftc.teamcode;
//bruh
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class urDad extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    @Override

    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftfront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftback");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightfront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightback");
        //DcMotor fourstage = hardwareMap.dcMotor.get("fourstage");
        DcMotor twostage = hardwareMap.dcMotor.get("twostage");
        //DcMotor plane = hardwareMap.dcMotor.get("plane");

        //Servos
        Servo out = hardwareMap.get(Servo.class, "out");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo clarm = hardwareMap.get(Servo.class, "clarm");
        Servo plane = hardwareMap.get(Servo.class, "plane");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Reverse viperslide motors

        //fourstage.setDirection(DcMotorSimple.Direction.REVERSE);
        //twostage.setDirection(DcMotorSimple.Direction.REVERSE);

        //Brake
        //fourstage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        twostage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set position
        //fourstage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fourstage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        twostage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //twostage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        twostage.setTargetPosition(0);
        twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //fourstage.setTargetPosition(0);
        //fourstage.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        out.setPosition(.7);
        clarm.setPosition(1);
        claw.setPosition(0.95);
        plane.setPosition(0);

        //Speed
        Boolean normalmode = true;
        Boolean turbomode = false;
        Boolean slowmode = false;
        String mode = "normalmode";


        waitForStart();

        if (isStopRequested()) {
            out.setPosition(0);
        }

        while (opModeIsActive()) {

            out.getPosition();
            telemetry.addLine("sussy bakea");
            telemetry.addData("out pos", out.getPosition());
            telemetry.addData("clarm pos", clarm.getPosition());
            telemetry.addData("clawpos", claw.getPosition());
            telemetry.addData("planepos", plane.getPosition());

            telemetry.update();

            if (gamepad2.left_bumper) {
                out.setPosition(.7);
            }
            else if (gamepad2.right_bumper) {
                out.setPosition(0.1);
            }

            if (gamepad2.left_stick_button) {
                claw.setPosition(0.95);
            }
            if (gamepad2.right_stick_button) {
                claw.setPosition(0.5);
            }

            if (gamepad2.y) {
                clarm.setPosition(1);
            }
            if (gamepad2.x) {
                clarm.setPosition(0.52);
            }
            if (gamepad2.a) {
                clarm.setPosition(0.462);
            }
            if (gamepad1.dpad_left) {
                plane.setPosition(0);
            }
            if (gamepad1.dpad_right) {
                plane.setPosition(1);
            }


            //More encoder stuff
            double CPR = 537.7;

            // Get the current position of the motor ticks
            /*int fourpos = fourstage.getCurrentPosition();
            double fourrev = fourpos / CPR;
            double fourang = fourrev * 360;
            double fourangnorm = fourang % 360;*/

            int twopos = twostage.getCurrentPosition();
            double tworev = twopos / CPR;
            double twoang = tworev * 360;
            double twoangnorm = twoang % 360;


            if (gamepad1.dpad_up) {
                twostage.setTargetPosition(2200);
                twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                twostage.setPower(.5);
            }
            if (gamepad1.dpad_down) {
                twostage.setTargetPosition(0);
                twostage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                twostage.setPower(.5);
            }

            /*if (gamepad2.dpad_left) {
                fourstage.setTargetPosition(3300);
                fourstage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                fourstage.setPower(1);
            }
            if (gamepad2.dpad_right) {
                fourstage.setTargetPosition(0);
                fourstage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                fourstage.setPower(1);
            }*/

            /*if (gamepad1.right_bumper) {
                plane.setPower(1);
            } else {
                plane.setPower(0);
            }*/


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.left_bumper) {
                slowmode = true;
                turbomode = false;
                normalmode = false;
                mode = "slowmode";
            }

            if (gamepad1.right_stick_button) {
                slowmode = false;
                turbomode = false;
                normalmode = true;
                mode = "normalmode";
            }

            if (gamepad1.right_bumper) {
                slowmode = false;
                turbomode = true;
                normalmode = false;
                mode = "turbomode";
            }

            telemetry.addData("mode", mode);

            if (normalmode == true) {
                frontLeftMotor.setPower(frontLeftPower / 2);
                backLeftMotor.setPower(backLeftPower / 2);
                frontRightMotor.setPower(frontRightPower / 2);
                backRightMotor.setPower(backRightPower / 2);
            }
            if (turbomode == true) {
                frontLeftMotor.setPower(frontLeftPower*.8);
                backLeftMotor.setPower(backLeftPower*.8);
                frontRightMotor.setPower(frontRightPower*.8);
                backRightMotor.setPower(backRightPower*.8);
            }
            if (slowmode == true) {
                frontLeftMotor.setPower(frontLeftPower / 5);
                backLeftMotor.setPower(backLeftPower / 5);
                frontRightMotor.setPower(frontRightPower / 5);
                backRightMotor.setPower(backRightPower / 5);
            }
        }
    }






}