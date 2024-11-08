package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/*
--------------------------------------
---------- CONTROL SCHEME ------------
Buttons:
    A: Hold for Intake
    B: Hold for Reverse Intake
    X: Toggle Hanging Hooks

D-Pad:
    UP: Hold to Extend Arm Slide
    DOWN: Hold to Retract Arm Slide

Triggers:
    RT: Raise Arm
    LT: Lower Arm

Shoulder Buttons:
    RB: Rotate Claw Wrist Strait
    LB: Rotate Claw Wrist Left / Stow

Joysticks:
    Right: Relative Chassis Rotation
    Left: Absolute Chassis Strafe based on orientation when START button is pressed
 */


@TeleOp
public class OmniExample extends LinearOpMode{
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor armPivotMotor;
    DcMotor armSlideMotor;
    Servo rightHang;
    Servo leftHang;
    Servo clawWrist;
    CRServo clawIntake;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Definitions. Must match names setup in robot configuration in the driver hub. config is created and selected selected with driver hub menu
        // Drive Motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Arm Pivot Motor
        // Encoder of ~2500 is vertical
        armPivotMotor = hardwareMap.dcMotor.get("armPivotMotor");
        armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the motor encoder so that it reads zero ticks
        int armPivotDesiredPos = 400; // position the arm pivot will assume when the program is run, until a different position is commanded


        // Arm Slide Motor
        // Encoder of -2130 is fully extended
        armSlideMotor = hardwareMap.dcMotor.get("armSlideMotor");
        armSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the motor encoder so that it reads zero ticks
        int armSlideDesiredPos = 0;
        int armSlideLastMoveDirection = 0; // 0 = startup, 1 = reverse, 2 = forward

        // Hanging Claws
        rightHang = hardwareMap.get(Servo.class, "rightHangServo");
        leftHang = hardwareMap.get(Servo.class, "leftHangServo");

        // Game Element Intake Claw
        clawWrist = hardwareMap.get(Servo.class, "clawWristServo");
        clawIntake = hardwareMap.get(CRServo.class, "clawIntakeServo");
        clawWrist.setPosition(-1); // start within the starting config


        // Reverse some of the drive motors depending on physical setup
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Arm Pivot Motor
            int armPivotPos = armPivotMotor.getCurrentPosition(); // current position of the slide, used to prevent overextension/going past 0
            if(gamepad1.right_trigger > 0.5 && armPivotPos <= 4000) {
                armPivotMotor.setPower(1); // extend continuously while button is held
                armPivotMotor.setDirection(DcMotor.Direction.FORWARD);
                armPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armPivotDesiredPos = armPivotMotor.getCurrentPosition();
            } else if (gamepad1.left_trigger > 0.5 && (armPivotPos >= 150 || armPivotPos <= -150)) { // encoder pos is inverted when in reverse; so it just checks to make sure it isn't within 50 of zero
                armPivotMotor.setPower(1);  // retract continuously while button is held
                armPivotMotor.setDirection(DcMotor.Direction.REVERSE);
                armPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armPivotDesiredPos = armPivotMotor.getCurrentPosition();
            } else {
                armPivotMotor.setTargetPosition(armPivotDesiredPos); // hold the motor in its current position
                armPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Use builtin PID loop to hold position
                armPivotMotor.setPower(1); // Holding power
            }


            // Hanging hooks
            if (gamepad1.x && ((rightHang.getPosition() > 0.55) || (leftHang.getPosition() < 0.45))) { // if A button is pressed AND both of the claws is closed
                rightHang.setPosition(0); // They are facing away from each-other, so they start at opposite ends
                leftHang.setPosition(1);
            } else if (gamepad1.x && ((rightHang.getPosition() < 0.55) || (leftHang.getPosition() > 0.45))) { // if A button is pressed AND both of the claws is open
                rightHang.setPosition(0.6); // +0.6 from starting pos
                leftHang.setPosition(0.4); // -0.6 from starting pos; - is due to facing opposite direction
            }

            // Claw Wrist
            if (gamepad1.right_bumper) {
                clawWrist.setPosition(0.35);
            } else if (gamepad1.left_bumper) {
                clawWrist.setPosition(-1);
            }

            // Claw Intake
            if (gamepad1.a) {
                clawIntake.setPower(-1); // set the power of the continuous servo to full forward
            } else if (gamepad1.b) {
                clawIntake.setPower(1); // full backward
            } else {
                clawIntake.setPower(0); // no power
            }


            // Arm Slide
            //encoder directions become negative depending on motor directions
            int armSlidePos = armSlideMotor.getCurrentPosition(); // current position of the slide, used to prevent overextension/going past 0
            if(gamepad1.dpad_up && (armSlidePos <= 2100 || armSlideLastMoveDirection == 1 || armSlideLastMoveDirection == 0)) { // 2100 is hardcoded end stop
                armSlideMotor.setPower(1); // extend continuously while button is held
                armSlideMotor.setDirection(DcMotor.Direction.REVERSE);
                armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armSlideDesiredPos = armSlideMotor.getCurrentPosition();
                armSlideLastMoveDirection = 2; // forward
            } else if (gamepad1.dpad_down && ((armSlideLastMoveDirection == 2 && armSlidePos >= 35) || (armSlideLastMoveDirection == 1 && armSlidePos <= 35))) { // encoder pos is inverted when in reverse; so it just checks to make sure it isn't within 35 of zero (due to belt slop)
                armSlideMotor.setPower(1);  // retract continuously while button is held
                armSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use builtin PID loop to run to position
                armSlideDesiredPos = armSlideMotor.getCurrentPosition(); // store current position in case the button isn't pressed next loop, so it knows where to hold
                armSlideLastMoveDirection = 1; // backward
            } else {
                armSlideMotor.setTargetPosition(armSlideDesiredPos); // hold the motor at the position it was in last time it was moved
                armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Use builtin PID loop to hold position
                armSlideMotor.setPower(0.25); // Holding power
            }


            // Outputs telemetry data to driver hub screen
            telemetry.addData("Arm Pivot Encoder Position :", armPivotMotor.getCurrentPosition());
            telemetry.addData("Arm Slide Encoder Position :", armSlideMotor.getCurrentPosition());
            telemetry.addData("Right Hang Servo Position :", rightHang.getPosition());
            telemetry.addData("Left Hang Servo Position :", leftHang.getPosition());
            telemetry.addData("Claw Wrist Servo Position :", clawWrist.getPosition());
            telemetry.addData("Claw Intake Servo Power :", clawIntake.getPower());
            telemetry.update();
        }
    }
}

