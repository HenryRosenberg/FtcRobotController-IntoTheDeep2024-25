package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class OmniExample extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Arm Pivot Motor
        // Encoder of ~2500 is vertical
        DcMotor armPivotMotor = hardwareMap.dcMotor.get("armPivotMotor");
        armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the motor encoder so that it reads zero ticks

        // Arm Slide Motor
        // Encoder of -2130 is fully extended
        DcMotor armSlideMotor = hardwareMap.dcMotor.get("armSlideMotor");
        armSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the motor encoder so that it reads zero ticks

        // Hanging Claws
        Servo rightHang = hardwareMap.get(Servo.class, "rightHangServo");
        Servo leftHang = hardwareMap.get(Servo.class, "leftHangServo");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
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




            // Raise the arm
            if (gamepad1.right_trigger > .5) {
                armPivotMotor.setTargetPosition(2000);
                armPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPivotMotor.setPower(1);
            }

            // Lower the arm
            if (gamepad1.left_trigger > .5) {
                armPivotMotor.setTargetPosition(200);
                armPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPivotMotor.setPower(0.8);
            }


            // Hanging claws
            if (gamepad1.a && ((rightHang.getPosition() > 0.55) || (leftHang.getPosition() < 0.45))) { // if A button is pressed AND both of the claws is closed
                rightHang.setPosition(0); // They are facing away from each-other, so they start at opposite ends
                leftHang.setPosition(1);
                telemetry.addLine("Opening Hanging Hooks");
            } else if (gamepad1.a && ((rightHang.getPosition() < 0.55) || (leftHang.getPosition() > 0.45))) { // if A button is pressed AND both of the claws is open
                rightHang.setPosition(0.6); // +0.6 from starting pos
                leftHang.setPosition(0.4); // -0.6 from starting pos; - is due to facing opposite direction
                telemetry.addLine("Closing Hanging Hooks");
            }

            telemetry.addData("Arm Pivot Encoder Position", armPivotMotor.getCurrentPosition());
            telemetry.addData("Arm Slide Encoder Position", armSlideMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
