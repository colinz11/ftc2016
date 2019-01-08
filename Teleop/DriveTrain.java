package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by SaajanPatel on 12/3/16.
 */

class Drivetrain {
    public DcMotor Left1 = null;
    public DcMotor Left2 = null;
    public DcMotor Right1 = null;
    public DcMotor Right2 = null;
    HardwareMap hwMap = null;

    public Drivetrain() {
    }

    public void init(HardwareMap ahwMap) {
        //Save reference to Hardware map
        hwMap = ahwMap;
        Left1 = hwMap.dcMotor.get("Left1");
        Left2 = hwMap.dcMotor.get("Left2");
        Right1 = hwMap.dcMotor.get("Right1");
        Right2 = hwMap.dcMotor.get("Right2");

        Left1.setDirection(DcMotor.Direction.REVERSE);
        Left2.setDirection(DcMotor.Direction.REVERSE);
        Right1.setDirection(DcMotor.Direction.FORWARD);
        Right2.setDirection(DcMotor.Direction.FORWARD);

        Left1.setPower(0);
        Left2.setPower(0);
        Right1.setPower(0);
        Right2.setPower(0);

        Left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
