package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {
    private CRServo servo;

    public static double FEED_POWER = 1.0;
    public static double REVERSE_POWER = -0.6;   // for clearing jams

    /** reversed = true if feed() pushes pieces the wrong way for this servo. */
    public void init(HardwareMap hw, String name, boolean reversed) {
        servo = hw.get(CRServo.class, name);
        servo.setDirection(reversed ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
    }

    public void feed()    { servo.setPower(FEED_POWER); }
    public void reverse() { servo.setPower(REVERSE_POWER); }
    public void stop()    { servo.setPower(0); }
}