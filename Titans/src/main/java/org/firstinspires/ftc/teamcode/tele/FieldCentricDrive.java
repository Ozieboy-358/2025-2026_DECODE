package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Field Drive")
public class FieldCentricDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor Motor_Front_Left = hardwareMap.dcMotor.get( "Motor Front Left"  );
        DcMotor Motor_Front_Right = hardwareMap.dcMotor.get("Motor Front Right" );
        DcMotor Motor_Back_Left = hardwareMap.dcMotor.get(  "Motor Back Left"   );
        DcMotor Motor_Back_Right = hardwareMap.dcMotor.get( "Motor Back Right"  );
        CRServo Cylinder  = hardwareMap.crservo.get("Cylinder");
        CRServo Launcher  = hardwareMap.crservo.get("launch");








        Motor_Back_Left.setDirection(DcMotor.Direction.REVERSE);
        Motor_Front_Left.setDirection(DcMotor.Direction.REVERSE);

        Motor_Front_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_Back_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_Front_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_Back_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

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

            rotX = -rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            Motor_Front_Left.setPower(frontLeftPower);
            Motor_Front_Right.setPower(frontRightPower);
            Motor_Back_Left.setPower(backLeftPower);
            Motor_Back_Right.setPower(backRightPower);

            if (gamepad2.a){
                Cylinder.setDirection(DcMotorSimple.Direction.FORWARD);
                Cylinder.setPower(1);
            }
            if (gamepad2.x){
                Cylinder.setPower(0);
            }

            //what button for launcher
            //this is the ooutline gamepad 2 is gunner gamepad 1 is driver gamepad
            //gamepad2.(buttom they are wanting to use)
            if (gamepad2.right_trigger > 0.1){
                //negative if runs wrong way
                //tuen as needed
                Launcher.setPower(1);
            }

            //ill let you handle this for now while that is importing
            //no, there havent said anything yet


            //idk how its set up or how the reset is on the launcher but 0 power for now if it t reset keep going forward or go backwards
            if (gamepad2.right_trigger > 0.1){
                Launcher.setPower(0);
            }

        }
    }

}
