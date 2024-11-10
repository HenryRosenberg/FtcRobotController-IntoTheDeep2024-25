// Written primarily by Henry Rosenberg for AcaBots FTC Team #24689

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
public class omniAutoTest extends LinearOpMode {
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

    private void omniMove(double inches, double power) {
        double cpr = 537.7; //counts per rotation
        double diameter = (96 / 25.4); // 96mm wheels, 25.4 mm per inch
        double cpi = cpr / (Math.PI * diameter); //counts per inch, cpr * gear ratio / (pi * diameter (in inches, in the center))
        double bias = 0.8; //default 0.8
        double conversion = cpi * bias;

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
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        //
        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
            if (isStopRequested()) {
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                return;
            }
            // Outputs telemetry data to driver hub screen
            telemetry.clearAll();
            telemetry.addData("Auto Status :", "Moving chassis \n");
            telemetry.update();
        }

        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }



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

        if (isStopRequested()) {
            return;
        }
        if (opModeIsActive()) { // Used to be a while loop
            telemetry.addData("Auto Status :", "Started \n");
            telemetry.update();


            // Pivot arm upward
            armPivotMotor.setTargetPosition(1580);
            armPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armPivotMotor.setPower(1);

            // Slide the slide out
            armSlideMotor.setTargetPosition(-1320);
            armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlideMotor.setPower(0.5);

            // wait for the arm to finish raising and extending
            while (armPivotMotor.getCurrentPosition() < 1500 || armSlideMotor.getCurrentPosition() > -1280) {
                // Outputs telemetry data to driver hub screen
                telemetry.clearAll();
                telemetry.addData("Auto Status :", "Waiting for arm & alide movement to finish \n");
                telemetry.addData("Arm Pivot Encoder Position :", armPivotMotor.getCurrentPosition());
                telemetry.addData("Arm Slide Encoder Position :", armSlideMotor.getCurrentPosition());
                telemetry.update();
            }

            sleep(1500);

            omniMove(32, 0.3);


            // Lower arm to contact rung
            armPivotMotor.setTargetPosition(1200);
            armPivotMotor.setPower(1);


            // Wait for the arm to finish lowering
            while (armPivotMotor.getCurrentPosition() > 1350) {
                // Outputs telemetry data to driver hub screen
                telemetry.clearAll();
                telemetry.addData("Auto Status :", "Waiting for arm pivot to lower \n");
                telemetry.addData("Arm Pivot Encoder Position :", armPivotMotor.getCurrentPosition());
                telemetry.update();
            }


            // Pull the arm slide back in
            armSlideMotor.setTargetPosition(-1000);
            armSlideMotor.setPower(1);

            // wait for the specimen to be attached, wait until the arm is finish retracting and it has been trying for 5 seconds
            double preEjectRuntime = getRuntime();
            while (armSlideMotor.getCurrentPosition() < - 1050 && ((getRuntime() - preEjectRuntime) < 2.0)) {
                // Outputs telemetry data to driver hub screen
                telemetry.clearAll();
                telemetry.addData("Auto Status :", "Waiting for specimen to eject \n");
                telemetry.addData("Arm Slide Encoder Position :", armSlideMotor.getCurrentPosition());
                telemetry.update();
            }


            // Outputs telemetry data to driver hub screen
            telemetry.clearAll();
            telemetry.addData("Auto Status :", "Finished! \n");
            telemetry.addData("Arm Pivot Encoder Position :", armPivotMotor.getCurrentPosition());
            telemetry.addData("Arm Slide Encoder Position :", armSlideMotor.getCurrentPosition());
            telemetry.addData("Right Hang Servo Position :", rightHang.getPosition());
            telemetry.addData("Left Hang Servo Position :", leftHang.getPosition());
            telemetry.addData("Claw Wrist Servo Position :", clawWrist.getPosition());
            telemetry.addData("Claw Intake Servo Power :", clawIntake.getPower());
            telemetry.update();


            omniMove(-12, 0.3);

            sleep(1500);

            armSlideMotor.setTargetPosition(0);
            armSlideMotor.setPower(0.5);

            armPivotMotor.setTargetPosition(50);
            armPivotMotor.setPower(1);

            sleep(1500);

            armPivotMotor.setPower(0);
            armSlideMotor.setPower(0);
        }
    }
}