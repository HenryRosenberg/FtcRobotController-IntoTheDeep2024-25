package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class autoTest extends LinearOpMode {
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


        // Arm Slide Motor
        // Encoder of -2130 is fully extended
        armSlideMotor = hardwareMap.dcMotor.get("armSlideMotor");
        armSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the motor encoder so that it reads zero ticks
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
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        waitForStart();

        if (isStopRequested()) {return;}

    }
    boolean runningAuto = false;
    public void moveToPosition(double inches, double speed) {

        Double width = 16.0; //inches
        Integer cpr = 28; //counts per rotation
        Integer gearratio = 40;
        Double diameter = 4.125;
        Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
        Double bias = 0.8;//default 0.8
        Double conversion = cpi * bias;

        int move = (int) (Math.round(inches * conversion));

        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + move);
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + move);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + move);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + move);
        //
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
        //
        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
            if (isStopRequested()) {
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                return;
            }
        }
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);

        if(!runningAuto) {
            while (opModeIsActive()) {
                runningAuto = true;
                if (isStopRequested()) {
                    stop();
                }

                moveToPosition(12, 1);

                // Outputs telemetry data to driver hub screen
                telemetry.addData("Arm Pivot Encoder Position :", armPivotMotor.getCurrentPosition());
                telemetry.addData("Arm Slide Encoder Position :", armSlideMotor.getCurrentPosition());
                telemetry.addData("Right Hang Servo Position :", rightHang.getPosition());
                telemetry.addData("Left Hang Servo Position :", leftHang.getPosition());
                telemetry.addData("Claw Wrist Servo Position :", clawWrist.getPosition());
                telemetry.addData("Claw Intake Servo Power :", clawIntake.getPower());
                telemetry.update();

                stop();
            }
        }
    }
}