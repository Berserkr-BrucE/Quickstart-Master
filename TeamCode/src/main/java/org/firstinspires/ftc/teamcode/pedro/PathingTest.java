
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FlyWheel.FlyWheel;
import org.firstinspires.ftc.teamcode.FlyWheel.Turret;
import org.firstinspires.ftc.teamcode.Intake;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "PathingTest", group = "Autonomous")
public class PathingTest extends LinearOpMode {
    private static final boolean ENABLE_POLLEN = true;
    private static final double GOAL_X = 57.7;
    private static final double GOAL_Y = 55.5;
    private static final double POLLEN_FORWARD = -2.3;
    private static final double POLLEN_LEFT = -4.7;
    // Use the power that lands the pollen shot in TeleOp, written as a decimal.
    private static final double POLLEN_POWER = 0.50;
    private static final int POLLEN_SPINUP_MS = 2000;
    private static final int SHOT_READY_TIMEOUT_MS = 3500;

    // Match the confirmed pollen stopper name in PedroTeleOp.
    private static final String STOPPER_NAME = "rightFeeder";
    private static final double STOPPER_OPEN = 0.00;
    private static final double STOPPER_CLOSED = 0.10;
    private static final int STOPPER_HOLD_OPEN_MS = 600;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(9.6685, 13.7113, 90);
    private final Pose path1 = poseFactory.of(36.0488, 11.1178, 90);
    private final Pose point2Start = poseFactory.of(36.0488, 11.1178, 90);
    private final Pose point2 = poseFactory.of(9.03, 117.0163, 90);

    private final Turret turret = new Turret();
    private final FlyWheel pollen = new FlyWheel();
    private final Intake intake = new Intake();
    private final ElapsedTime shotTimer = new ElapsedTime();

    private Follower follower;
    private Servo pollenStopper;
    private boolean aiming;
    private boolean readyToShoot;
    private String shootingStatus = "Disabled by ENABLE_POLLEN";
    private String autoStage = "Ready";

    public Command autoRoutine() {
        if (pollenStopper == null) {
            return sequential(
                    instant(() -> autoStage = "Driving Path 1"),
                    follow(follower, path1()),
                    collectOnPath2()
            );
        }

        return sequential(
                instant(() -> autoStage = "Driving Path 1"),
                follow(follower, path1()),
                instant(() -> {
                    pollen.setFlywheelPower(POLLEN_POWER);
                    pollen.setTargetRPM(1); // Positive target switches on fixed-power mode.
                    shotTimer.reset();
                    aiming = true;
                    autoStage = "Aiming and spinning up";
                }),
                waitMs(POLLEN_SPINUP_MS),
                waitUntil(() -> readyToShoot
                        || shotTimer.milliseconds() >= SHOT_READY_TIMEOUT_MS),
                conditional(
                        () -> readyToShoot,
                        sequential(
                                instant(() -> {
                                    autoStage = "Releasing pollen";
                                    pollenStopper.setPosition(STOPPER_OPEN);
                                }),
                                waitMs(STOPPER_HOLD_OPEN_MS)
                        ),
                        instant(() -> telemetry.log().add("Pollen skipped: turret aim timed out."))
                ),
                instant(this::stopPollen),
                collectOnPath2()
        );
    }

    private Command collectOnPath2() {
        return sequential(
                instant(() -> {
                    autoStage = "Driving Path 2 with intake";
                    intake.intake();
                }),
                follow(follower, path2()),
                instant(() -> {
                    intake.stop();
                    autoStage = "Done";
                })
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        intake.init(hardwareMap);
        intake.stop();

        if (ENABLE_POLLEN) {
            if (!(POLLEN_POWER > 0 && POLLEN_POWER <= 1)
                    || !(STOPPER_OPEN >= 0 && STOPPER_OPEN <= 1)
                    || !(STOPPER_CLOSED >= 0 && STOPPER_CLOSED <= 1)
                    || STOPPER_OPEN == STOPPER_CLOSED) {
                shootingStatus = "Invalid pollen power or stopper positions";
            } else {
                turret.init(hardwareMap, "turretMotor", false);
                pollen.init(hardwareMap, "pollenMotor", true);
                pollenStopper = hardwareMap.get(Servo.class, STOPPER_NAME);
                pollenStopper.setPosition(STOPPER_CLOSED);
                shootingStatus = "Enabled";
            }
        }

        telemetry.addData("Pollen shot", shootingStatus);
        telemetry.update();

        try {
            waitForStart();
            if (isStopRequested()) {
                return;
            }
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                if (aiming) {
                    updatePollenAim();
                }
                Scheduler.execute();

                Pose pose = follower.pose();
                telemetry.addData("X (in)", pose.x());
                telemetry.addData("Y (in)", pose.y());
                telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
                telemetry.addData("Auto stage", autoStage);
                telemetry.addData("Pollen shot", shootingStatus);
                telemetry.addData("Pollen ready", readyToShoot);
                telemetry.update();
            }
        } finally {
            intake.stop();
            if (pollenStopper != null) {
                stopPollen();
            }
        }
    }

    private void updatePollenAim() {
        Pose pose = follower.pose();
        double heading = pose.heading();
        double pivotX = pose.x() + POLLEN_FORWARD * Math.cos(heading)
                - POLLEN_LEFT * Math.sin(heading);
        double pivotY = pose.y() + POLLEN_FORWARD * Math.sin(heading)
                + POLLEN_LEFT * Math.cos(heading);

        double angle = Math.atan2(GOAL_Y - pivotY, GOAL_X - pivotX) - heading;
        double target = Math.atan2(Math.sin(angle), Math.cos(angle));
        boolean inRange = target >= Turret.MIN_ANGLE && target <= Turret.MAX_ANGLE;
        if (inRange) {
            turret.update(target, 0);
        } else {
            turret.setManualPower(0);
        }
        pollen.update();
        readyToShoot = inRange && turret.atTarget(0.05);
    }

    private void stopPollen() {
        aiming = false;
        readyToShoot = false;
        pollenStopper.setPosition(STOPPER_CLOSED);
        pollen.setTargetRPM(0);
        pollen.update();
        turret.stop();
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }
}