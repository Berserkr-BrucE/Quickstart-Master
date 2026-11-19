package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

    @TeleOp(name = "Pinpoint Readout")
    public class MeasureTeleOp extends LinearOpMode {
        @Override
        public void runOpMode() {
            PinpointConfig config = Constants.localizerConfig;
            GoBildaPinpointDriver pinpoint =
                    hardwareMap.get(GoBildaPinpointDriver.class, config.name.get());

            pinpoint.setOffsets(
                    config.xPodOffset.get(), config.yPodOffset.get(), config.offsetUnits.get());
            pinpoint.setEncoderResolution(config.podType.get());
            pinpoint.setEncoderDirections(config.xPodDirection.get(), config.yPodDirection.get());
            pinpoint.resetPosAndIMU();

            telemetry.addLine("Keep the robot still during INIT. Press START to read X and Y.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                pinpoint.update();
                Pose2D pose = pinpoint.getPosition();

                telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
                telemetry.addData("X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
                telemetry.addData("Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
                telemetry.update();
            }
        }
    }
