package org.firstinspires.ftc.teamcode.FlyWheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    private DcMotorEx motor;

    // ---- UPDATE THESE FOR YOUR ROBOT ----
    // Encoder ticks per MOTOR revolution (537.7 for a goBILDA 312 RPM motor, 28 for a 6000 RPM motor)
    public static double MOTOR_TICKS_PER_REV = 537.7;
    // Motor revolutions per ONE full turret revolution (e.g. a 100-tooth turret gear driven by a
    // 20-tooth gear on the motor = 5)
    public static double GEAR_RATIO = 5;

    // ---- TUNING (start with kP only) ----
    public static double kP = 0.5;   // power per radian of error
    public static double kD = 0;     // brakes overshoot; add a little if the turret wobbles
    public static double kF = 0;     // heading-rate feedforward; add last
    public static double kS = 0;     // friction push; add only if it stops short of the target

    // Cable/hard-stop limits in radians (0 = pointing straight forward, + = counterclockwise).
    // Starts SMALL on purpose. Widen it only after testing the real limits by hand.
    public static double MIN_ANGLE = -Math.PI / 2;
    public static double MAX_ANGLE =  Math.PI / 2;

    private double targetAngle = 0;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hw, String name) {
        init(hw, name, false);
    }

    /**
     * reversed = true flips the motor. Positive angle must be COUNTERCLOCKWISE (seen from above).
     * If a positive target turns the turret clockwise, set this to true.
     */
    public void init(HardwareMap hw, String name, boolean reversed) {
        motor = hw.get(DcMotorEx.class, name);
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        // Wherever the turret is sitting RIGHT NOW becomes angle 0.
        // Always start the turret centered (pointing forward) before pressing INIT.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();
    }

    private double ticksPerRad() {
        return MOTOR_TICKS_PER_REV * GEAR_RATIO / (2 * Math.PI);
    }

    public double getAngle() {
        return motor.getCurrentPosition() / ticksPerRad();
    }

    public double getTargetAngle() { return targetAngle; }

    public double getError() { return targetAngle - getAngle(); }

    /** True when the turret is within tolRad radians of its target (0.05 is about 3 degrees). */
    public boolean atTarget(double tolRad) {
        return Math.abs(getError()) < tolRad;
    }

    /**
     * Auto mode. Call every loop.
     * targetAngle: relative to the robot, in radians.
     * headingVelocity: robot's turning speed in rad/s (for feedforward).
     */
    public void update(double target, double headingVelocity) {
        targetAngle = chooseReachable(target);

        double error = targetAngle - getAngle();
        double dt = timer.seconds();
        timer.reset();
        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        lastError = error;

        double power = kP * error + kD * derivative - kF * headingVelocity
                + kS * Math.signum(error);
        applyPower(power);
    }

    /** Manual mode: use a joystick value from -1 to 1. Still respects the limits. */
    public void setManualPower(double power) {
        targetAngle = getAngle();   // so auto mode doesn't snap back when you let go
        lastError = 0;
        applyPower(power);
    }

    public void stop() { motor.setPower(0); }

    // Never push further into a limit, no matter what the controller asks for
    private void applyPower(double power) {
        double angle = getAngle();
        if (angle >= MAX_ANGLE && power > 0) power = 0;
        if (angle <= MIN_ANGLE && power < 0) power = 0;
        motor.setPower(Range.clip(power, -1, 1));
    }

    // Picks the equivalent angle (+/- 2pi) that lies inside the allowed range
    public double chooseReachable(double a) {
        while (a > MAX_ANGLE && a - 2 * Math.PI >= MIN_ANGLE) a -= 2 * Math.PI;
        while (a < MIN_ANGLE && a + 2 * Math.PI <= MAX_ANGLE) a += 2 * Math.PI;
        return Range.clip(a, MIN_ANGLE, MAX_ANGLE);
    }
}