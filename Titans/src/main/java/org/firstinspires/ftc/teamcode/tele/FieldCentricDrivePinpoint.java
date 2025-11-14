package org.firstinspires.ftc.teamcode.tele;

// Imports for Road Runner
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

// Imports for FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Import your own MecanumDrive class
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Disabled
@TeleOp(name = "new odo comp Field Drive ")
public class FieldCentricDrivePinpoint extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the MecanumDrive class, which handles all hardware setup
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        CRServo Cylinder = hardwareMap.crservo.get("Cylinder");


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Pose2d currentPose = drive.localizer.getPose();
            double botHeading = currentPose.heading.toDouble();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Reset heading on a button press
            if (gamepad1.options) {
                drive.localizer.setPose(new Pose2d(currentPose.position, 0));
            }


            double inputX = y;
            double inputY = x;

            double rotationAngle = -botHeading;

            double rotatedX = inputX * Math.cos(rotationAngle) - inputY * Math.sin(rotationAngle);
            double rotatedY = inputX * Math.sin(rotationAngle) + inputY * Math.cos(rotationAngle);

            Vector2d driveVector = new Vector2d(rotatedX, rotatedY);

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            driveVector,
                            rx
                    )
            );
            if (gamepad2.a){
                Cylinder.setDirection(DcMotorSimple.Direction.FORWARD);
                Cylinder.setPower(1);
            }
            if (gamepad2.b){
                Cylinder.setPower(0);
            }


            // CRITICAL: Update the localizer's position estimate in every loop
            drive.updatePoseEstimate();

            // Optional: Telemetry
            telemetry.addData("X Position", currentPose.position.x);
            telemetry.addData("Y Position", currentPose.position.y);
            telemetry.addData("Heading (degrees)", Math.toDegrees(botHeading));
            telemetry.update();
        }
    }
}