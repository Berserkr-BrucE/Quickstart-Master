package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake{
    //picking up pollen/nectar with intake(), outtake(), and stop()
    private DcMotor intakeMotor;
    public void init(HardwareMap hw){
        intakeMotor = hw.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void intake(){intakeMotor.setPower(1);}
    public void outtake(){intakeMotor.setPower(-1);}
    public void stop(){intakeMotor.setPower(0);}
}
