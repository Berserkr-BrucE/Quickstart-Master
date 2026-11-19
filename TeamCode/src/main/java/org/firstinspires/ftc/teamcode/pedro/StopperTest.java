package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Stopper Test")
public class StopperTest extends OpMode {
    private static final String STOPPER_NAME = "leftFeeder";

    private Servo stopper;
    private double selectedPosition = 0.50;

    @Override
    public void init() {
        stopper = hardwareMap.get(Servo.class, STOPPER_NAME);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadLeftWasPressed()) {
            selectedPosition = Math.max(0.0, selectedPosition - 0.01);
        }
        if (gamepad1.dpadRightWasPressed()) {
            selectedPosition = Math.min(1.0, selectedPosition + 0.01);
        }
        if (gamepad1.aWasPressed()) {
            stopper.setPosition(selectedPosition);
            telemetry.log().add(String.format("Applied: %.2f", selectedPosition));
        }

        telemetry.addData("Servo name", STOPPER_NAME);
        telemetry.addData("Servo port", stopper.getPortNumber());
        telemetry.addData("Selected position", "%.2f", selectedPosition);
        telemetry.addLine("D-pad left/right to select. Cross to apply.");
        telemetry.update();
    }
}