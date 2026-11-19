package org.firstinspires.ftc.teamcode.FlyWheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class FlyWheel {

    private static final double POLLEN_POWER = 0.20;
    private static final double NECTAR_POWER = 0.20;

    private double flywheelPower = 0.20;

    public void setFlywheelPower(double power) {
        flywheelPower = Math.max(0.0, Math.min(1.0, power));
    }
    private DcMotorEx motor;

    // Encoder ticks per MOTOR revolution (28 for goBILDA 6000 RPM motors)
    public static double TICKS_PER_REV = 28;

    // Motor revolutions per flywheel revolution.
    // 1 = flywheel on the motor shaft directly. UPDATE THIS FOR YOUR ROBOT.
    public static double GEAR_RATIO = 5;

    // Copy these from your FlywheelTuner once you've tuned them
    private static final double POLLEN_KP = 0.0;
    private static final double POLLEN_KF = 0.0;
    private static final double NECTAR_KP = 0.0;
    private static final double NECTAR_KF = 0.0;

    private double targetRPM = 0;

    public void init(HardwareMap hw, String name) {
        init(hw, name, true);   // true = the REVERSE direction you were already using
    }

    /** reversed = true flips the motor. The flywheel should spin so pieces launch forward. */

    public void init(HardwareMap hw, String name, boolean reversed) {
        motor = hw.get(DcMotorEx.class, name);
        flywheelPower = "nectarMotor".equals(name) ? NECTAR_POWER : POLLEN_POWER;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(reversed
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        applyPIDF(name);
    }

    private void applyPIDF(String motorName) {
        boolean isNectar = "nectarMotor".equals(motorName);
        double p = isNectar ? NECTAR_KP : POLLEN_KP;
        double f = isNectar ? NECTAR_KF : POLLEN_KF;

        if (p == 0.0 && f == 0.0) {
            return;
        }


        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(p, 0, 0, f));
    }



    public void setTargetRPM(double rpm) { targetRPM = rpm; }

    public double getTargetRPM() { return targetRPM; }

    public double getTicksPerSec() { return motor.getVelocity(); }

    /** Flywheel RPM (after the gear ratio). */
    public double getRPM() {
        return (getTicksPerSec() / TICKS_PER_REV) * 60.0 / GEAR_RATIO;
    }

    public boolean atSpeed(double tolRPM) {
        return targetRPM > 0 && Math.abs(targetRPM - getRPM()) < tolRPM;
    }

    public void update() {
        motor.setPower(targetRPM > 0 ? flywheelPower : 0.0);
    }

}