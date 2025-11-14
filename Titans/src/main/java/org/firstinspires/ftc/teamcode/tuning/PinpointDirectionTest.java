package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pinpoint Encoder Direction Test", group = "Tuning")
public class PinpointDirectionTest extends LinearOpMode {

    // Renamed this variable from "pinpoint" to "driver" to match your PinpointLocalizer.java
    private GoBildaPinpointDriver driver;

    @Override
    public void runOpMode() throws InterruptedException {
        // The hardware map name "pinpoint" matches your configuration. This is correct.
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        telemetry.addLine("Ready to start.");
        telemetry.addLine("On start, push the robot to test encoder directions.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driver.update();


            telemetry.addLine("--- INSTRUCTIONS ---");
            telemetry.addLine("Push robot FORWARD: Parallel ticks should INCREASE.");
            telemetry.addLine("Push robot LEFT: Perpendicular ticks should INCREASE.");
            telemetry.addLine();
            telemetry.addLine("If a value decreases, its direction needs to be reversed.");
            telemetry.addLine("Change directions in PinpointLocalizer.java");
            telemetry.addLine("--------------------");
            telemetry.addData("Parallel Encoder Ticks", driver.getPosX(DistanceUnit.INCH));
            telemetry.addData("Perpendicular Encoder Ticks", driver.getPosY(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}