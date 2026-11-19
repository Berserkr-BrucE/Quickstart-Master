package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FlyWheel.FlyWheel;
import org.firstinspires.ftc.teamcode.FlyWheel.Turret;

/**
 * A ............ toggle flywheel on/off
 * D-pad up/down . flywheel target RPM +/- 100
 * Right bumper .. turret target +0.2 rad
 * Left bumper ... turret target -0.2 rad
 * X ............. turret target back to 0
 */
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends OpMode {

    private FlyWheel flywheel = new FlyWheel();
    private Turret turret = new Turret();

    private boolean flywheelOn = false;
    private double targetRPM = 1000;      // start LOW and work up
    private double turretTarget = 0;

    @Override
    public void init() {
        flywheel.init(hardwareMap, "pollenMotor");
        turret.init(hardwareMap, "turretMotor");
        telemetry.addLine("Make sure the turret is centered before INIT");
    }

    @Override
    public void loop() {
        // ---- flywheel controls ---- \\
        if (gamepad1.aWasPressed()) flywheelOn = !flywheelOn;
        if (gamepad1.dpadUpWasPressed()) targetRPM += 100;
        if (gamepad1.dpadDownWasPressed()) targetRPM = Math.max(0, targetRPM - 100);

        flywheel.setTargetRPM(flywheelOn ? targetRPM : 0);
        flywheel.update();

        // ---- turret controls ---- \\
        if (gamepad1.rightBumperWasPressed()) turretTarget += 0.2;
        if (gamepad1.leftBumperWasPressed()) turretTarget -= 0.2;
        if (gamepad1.xWasPressed()) turretTarget = 0;

        turret.update(turretTarget, 0);   // 0 = no heading feedforward yet

        // ---- telemetry ---- \\
        telemetry.addData("Flywheel on", flywheelOn);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", "%.0f", flywheel.getRPM());
        telemetry.addData("At speed", flywheel.atSpeed(50));
        telemetry.addLine("-----");
        telemetry.addData("Turret target (rad)", "%.2f", turret.getTargetAngle());
        telemetry.addData("Turret angle (rad)", "%.2f", turret.getAngle());
        telemetry.addData("Turret error (rad)", "%.3f", turret.getError());
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.setTargetRPM(0);
        flywheel.update();
        turret.stop();
    }
}