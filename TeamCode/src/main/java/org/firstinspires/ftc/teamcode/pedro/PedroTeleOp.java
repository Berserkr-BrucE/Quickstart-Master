package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FlyWheel.FlyWheel;
import org.firstinspires.ftc.teamcode.FlyWheel.Turret;
import org.firstinspires.ftc.teamcode.Intake;

@TeleOp
public class PedroTeleOp extends OpMode {
    private static final boolean ENABLE_STOPPERS = false;

    private static final double GOAL_X = 57.7;
    private static final double GOAL_Y = 55.5;
    private static final double POLLEN_FORWARD = -2.3;
    private static final double POLLEN_LEFT = -4.7;

    // Confirm this name moves the physical pollen stopper before enabling it.
    private static final String STOPPER_NAME = "rightFeeder";
    private static final double STOPPER_OPEN = 0.00;
    private static final double STOPPER_CLOSED = 0.10;

    private static final String NECTAR_STOPPER_NAME = "";
    private static final double NECTAR_STOPPER_OPEN = Double.NaN;
    private static final double NECTAR_STOPPER_CLOSED = Double.NaN;

    private final Pose startPose = PoseFactory.degrees().of(0, 0, 90);
    private final Pose aimingReference =
            PoseFactory.degrees().of(36.0488, 11.1178, 90);

    private final Intake intake = new Intake();
    private final Turret turret = new Turret();
    private final FlyWheel pollen = new FlyWheel();
    private final FlyWheel nectar = new FlyWheel();

    private Follower follower;
    private Servo pollenStopper;
    private Servo nectarStopper;
    private double driverHeadingOffset;
    private boolean fieldPoseSet;
    private boolean pollenAutoAim;
    private boolean pollenAimed;
    private boolean pollenOn;
    private boolean nectarOn;

    private double pollenPower = 0.48; // 2,800 RPM in your test
    private double nectarPower = 0.50; // 3,000 RPM in your test
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
        driverHeadingOffset = follower.pose().heading();

        intake.init(hardwareMap);
        turret.init(hardwareMap, "turretMotor", false);
        pollen.init(hardwareMap, "pollenMotor", true);
        nectar.init(hardwareMap, "nectarMotor", true);

        if (ENABLE_STOPPERS) {
            pollenStopper = findStopper(STOPPER_NAME, STOPPER_OPEN, STOPPER_CLOSED);
            nectarStopper = findStopper(
                    NECTAR_STOPPER_NAME, NECTAR_STOPPER_OPEN, NECTAR_STOPPER_CLOSED);
        }

        stopMechanisms();
        telemetry.addLine("Turrets should be centered before INIT.");
        telemetry.update();
    }

    private Servo findStopper(String name, double open, double closed) {
        if (name.isEmpty() || !(open >= 0 && open <= 1)
                || !(closed >= 0 && closed <= 1) || open == closed) {
            return null;
        }
        return hardwareMap.get(Servo.class, name);
    }

    @Override
    public void loop() {
        updateDrive();
        updateMechanisms();

        Pose pose = follower.pose();
        telemetry.addData("Drive mode", "Field-centric");
        telemetry.addData("X (in)", "%.1f", pose.x());
        telemetry.addData("Y (in)", "%.1f", pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Turret (deg)", "%.1f", Math.toDegrees(turret.getAngle()));
        telemetry.addData("Pollen power", "%.0f%% %s",
                100 * pollenPower, pollenOn ? "ON" : "OFF");
        telemetry.addData("Nectar power", "%.0f%% %s",
                100 * nectarPower, nectarOn ? "ON" : "OFF");
        telemetry.addData("Pollen stopper configured", pollenStopper != null);
        telemetry.addData("Nectar stopper configured", nectarStopper != null);
        telemetry.addData("Field pose set", fieldPoseSet);
        telemetry.addData("Pollen auto-aim", pollenAutoAim);
        telemetry.addData("Pollen aimed", pollenAimed);
        telemetry.update();
    }

    private void updateDrive() {
        double heading = follower.pose().heading();

        if (gamepad1.xWasPressed()) {
            if (gamepad1.back) { // SHARE + Square at the known field pose
                follower.setPose(aimingReference);
                heading = aimingReference.heading();
                fieldPoseSet = true;
                pollenAutoAim = false;
                pollenAimed = false;
            }
            driverHeadingOffset = heading;
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                heading - driverHeadingOffset);
        ManualDrive.driveOrHold(follower, powers);
        follower.update();
    }

    private void updateMechanisms() {
        if (gamepad1.right_trigger > 0.1) {
            intake.intake();
        } else if (gamepad1.left_trigger > 0.1) {
            intake.outtake();
        } else {
            intake.stop();
        }

        updateTurret();

        if (gamepad1.aWasPressed()) { // Cross
            pollenOn = !pollenOn;
        }
        if (gamepad1.bWasPressed()) { // Circle
            nectarOn = !nectarOn;
        }

        // In your fixed-power FlyWheel.java, a positive target means motor on.
        if (gamepad1.dpadUpWasPressed()) {
            if (gamepad1.back) {
                nectarPower = Math.min(1.0, nectarPower + 0.01);
            } else {
                pollenPower = Math.min(1.0, pollenPower + 0.01);
            }
        }

        if (gamepad1.dpadDownWasPressed()) {
            if (gamepad1.back) {
                nectarPower = Math.max(0.0, nectarPower - 0.01);
            } else {
                pollenPower = Math.max(0.0, pollenPower - 0.01);
            }
        }

        pollen.setFlywheelPower(pollenPower);
        nectar.setFlywheelPower(nectarPower);
        pollen.setTargetRPM(pollenOn ? 1 : 0);
        nectar.setTargetRPM(nectarOn ? 1 : 0);
        pollen.update();
        nectar.update();

        if (pollenStopper != null) {
            boolean shootPollen = gamepad1.left_bumper && pollenOn
                    && (!pollenAutoAim || pollenAimed);
            pollenStopper.setPosition(shootPollen ? STOPPER_OPEN : STOPPER_CLOSED);
        }

        if (nectarStopper != null) {
            boolean shootNectar = gamepad1.right_bumper && nectarOn;
            nectarStopper.setPosition(shootNectar
                    ? NECTAR_STOPPER_OPEN : NECTAR_STOPPER_CLOSED);
        }
    }

    private void updateTurret() {
        if (gamepad1.yWasPressed() && fieldPoseSet
                && Double.isFinite(POLLEN_FORWARD) && Double.isFinite(POLLEN_LEFT)) {
            pollenAutoAim = !pollenAutoAim;
        }

        double manualPower = (gamepad1.dpad_left ? 0.25 : 0.0)
                - (gamepad1.dpad_right ? 0.25 : 0.0);

        if (manualPower != 0) {
            pollenAutoAim = false;
        }

        pollenAimed = false;
        if (!pollenAutoAim) {
            turret.setManualPower(manualPower);
            return;
        }

        Pose pose = follower.pose();
        double heading = pose.heading();
        double pivotX = pose.x() + POLLEN_FORWARD * Math.cos(heading)
                - POLLEN_LEFT * Math.sin(heading);
        double pivotY = pose.y() + POLLEN_FORWARD * Math.sin(heading)
                + POLLEN_LEFT * Math.cos(heading);

        double angle = Math.atan2(GOAL_Y - pivotY, GOAL_X - pivotX) - heading;
        double target = Math.atan2(Math.sin(angle), Math.cos(angle));

        if (target < Turret.MIN_ANGLE || target > Turret.MAX_ANGLE) {
            turret.setManualPower(0);
            return;
        }

        turret.update(target, 0);
        pollenAimed = turret.atTarget(0.05);
    }

    private void stopMechanisms() {
        intake.stop();
        turret.stop();

        if (pollenStopper != null) {
            pollenStopper.setPosition(STOPPER_CLOSED);
        }
        if (nectarStopper != null) {
            nectarStopper.setPosition(NECTAR_STOPPER_CLOSED);
        }

        pollenOn = false;
        nectarOn = false;
        pollen.setTargetRPM(0);
        nectar.setTargetRPM(0);
        pollen.update();
        nectar.update();
    }

    @Override
    public void stop() {
        stopMechanisms();
    }
    }