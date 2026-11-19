package org.firstinspires.ftc.teamcode.FlyWheel;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The whole shooting system: ONE turret motor that rotates BOTH turrets together,
 * plus TWO flywheels (left and right).
 *
 * Because both turrets share one angle, the turret aims from a reference point between the two
 * pivots. Each flywheel still picks its own RPM from its own distance to the goal.
 */
public class Shooter {
    private final Turret turret = new Turret();
    private final FlyWheel leftFlywheel = new FlyWheel();
    private final FlyWheel rightFlywheel = new FlyWheel();

    // Pivot positions relative to the robot's center, in INCHES.
    // X = forward (+) / backward (-), Y = left (+) / right (-)
    private final double leftOffsetX, leftOffsetY, rightOffsetX, rightOffsetY;

    // Which point the shared turret angle is aimed from:
    // 0.0 = aim from the LEFT pivot, 1.0 = aim from the RIGHT pivot, 0.5 = midpoint.
    // Use this to favor the shooter you care about more.
    public static double AIM_BLEND = 0.5;

    // ---- Distance (inches) -> flywheel RPM table ----
    // PLACEHOLDER VALUES. Replace with real numbers from test shots.
    // Distances must be in increasing order.
    private static final double[] DISTANCES = {24, 48, 72, 96, 120};
    private static final double[] RPMS      = {2500, 3000, 3500, 4000, 4500};

    // ---- State ----
    private boolean spinning = false;     // are the flywheels supposed to be running?
    private boolean autoAim = true;       // false = driver controls the turret
    private double manualTurretPower = 0;
    private double manualRPM = 0;         // if > 0, overrides the distance table (both flywheels)

    private double lastLeftDistance = 0, lastRightDistance = 0;
    private double lastTurretTarget = 0;

    public Shooter(double leftOffsetX, double leftOffsetY, double rightOffsetX, double rightOffsetY) {
        this.leftOffsetX = leftOffsetX;
        this.leftOffsetY = leftOffsetY;
        this.rightOffsetX = rightOffsetX;
        this.rightOffsetY = rightOffsetY;
    }

    public void init(HardwareMap hw,
                     String turretName, boolean turretReversed,
                     String leftFlywheelName, boolean leftFlywheelReversed,
                     String rightFlywheelName, boolean rightFlywheelReversed) {
        turret.init(hw, turretName, turretReversed);
        leftFlywheel.init(hw, leftFlywheelName, leftFlywheelReversed);
        rightFlywheel.init(hw, rightFlywheelName, rightFlywheelReversed);
    }

    // ---------- controls from teleop ----------
    public void setSpinning(boolean on)        { spinning = on; }
    public boolean isSpinning()                { return spinning; }
    public void setAutoAim(boolean on)         { autoAim = on; }
    public boolean isAutoAim()                 { return autoAim; }
    public void setManualTurretPower(double p) { manualTurretPower = p; }
    public void setManualRPM(double rpm)       { manualRPM = rpm; }   // 0 = use the table

    /**
     * True when the turret is pointed where it WANTS to point. We compare against the real desired
     * angle (not the clamped one), so if the goal is outside the turret's range this is false.
     */
    public boolean isTurretAimed() {
        return !autoAim || Math.abs(angleWrap(lastTurretTarget - turret.getAngle())) < 0.05;
    }

    public boolean isLeftReady()  { return spinning && isTurretAimed() && leftFlywheel.atSpeed(50); }
    public boolean isRightReady() { return spinning && isTurretAimed() && rightFlywheel.atSpeed(50); }
    public boolean isReady()      { return isLeftReady() && isRightReady(); }

    // ---------- the main loop call ----------
    public void update(double x, double y, double heading, double headingVelocity,
                       double goalX, double goalY) {

        double cos = Math.cos(heading), sin = Math.sin(heading);

        // Each pivot's position on the FIELD (rotate its offset by the robot heading)
        double lx = x + leftOffsetX * cos - leftOffsetY * sin;
        double ly = y + leftOffsetX * sin + leftOffsetY * cos;
        double rx = x + rightOffsetX * cos - rightOffsetY * sin;
        double ry = y + rightOffsetX * sin + rightOffsetY * cos;

        // Turret angle: aimed from a point between the two pivots
        double ax = lx + AIM_BLEND * (rx - lx);
        double ay = ly + AIM_BLEND * (ry - ly);
        double fieldAngleToGoal = Math.atan2(goalY - ay, goalX - ax);
        double turretAngle = angleWrap(fieldAngleToGoal - heading);   // relative to the robot

        // Each flywheel's own distance to the goal
        lastLeftDistance = Math.hypot(goalX - lx, goalY - ly);
        lastRightDistance = Math.hypot(goalX - rx, goalY - ry);
        lastTurretTarget = turretAngle;

        // ---- turret (one motor, both turrets) ----
        if (autoAim) {
            turret.update(turretAngle, headingVelocity);
        } else {
            turret.setManualPower(manualTurretPower);
        }

        // ---- flywheels ----
        double leftRpm = manualRPM > 0 ? manualRPM : rpmFromDistance(lastLeftDistance);
        double rightRpm = manualRPM > 0 ? manualRPM : rpmFromDistance(lastRightDistance);
        leftFlywheel.setTargetRPM(spinning ? leftRpm : 0);
        rightFlywheel.setTargetRPM(spinning ? rightRpm : 0);
        leftFlywheel.update();
        rightFlywheel.update();
    }

    // ---------- helpers ----------
    // Linear interpolation between the two nearest table entries
    private double rpmFromDistance(double d) {
        if (d <= DISTANCES[0]) return RPMS[0];
        for (int i = 1; i < DISTANCES.length; i++) {
            if (d <= DISTANCES[i]) {
                double t = (d - DISTANCES[i - 1]) / (DISTANCES[i] - DISTANCES[i - 1]);
                return RPMS[i - 1] + t * (RPMS[i] - RPMS[i - 1]);
            }
        }
        return RPMS[RPMS.length - 1];
    }

    public static double angleWrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // ---------- telemetry getters ----------
    public double getTurretAngle()      { return turret.getAngle(); }
    public double getTurretTarget()     { return lastTurretTarget; }
    public double getLeftDistance()     { return lastLeftDistance; }
    public double getRightDistance()    { return lastRightDistance; }
    public double getLeftRPM()          { return leftFlywheel.getRPM(); }
    public double getRightRPM()         { return rightFlywheel.getRPM(); }
    public double getLeftTargetRPM()    { return leftFlywheel.getTargetRPM(); }
    public double getRightTargetRPM()   { return rightFlywheel.getTargetRPM(); }
}