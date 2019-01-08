package org.firstinspires.ftc.teamcode.Teleop;
import android.os.SystemClock;
import android.provider.Settings;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by SaajanPatel on 12/3/16.
 */

class ParticleAccelerator {
    private enum acceleratorEnum {Run, Stop}

    private acceleratorEnum AcceleratorState = acceleratorEnum.Stop;

    public DcMotor ShooterRight = null;
    public DcMotor ShooterLeft = null;
    HardwareMap hwMap = null;


    //Max encoder clicks per second
    public static final int SHOOTER_MAX_SPEED = 4000;


    //Percentage of speed
    public static final double SHOOTER_LEFT_POWER = 0.475;
    public static final double SHOOTER_RIGHT_POWER = -0.475;


    public void init(HardwareMap ahwMap) {
        //Save reference to Hardware map
        hwMap = ahwMap;
        ShooterRight = hwMap.dcMotor.get("ShooterRight");
        ShooterLeft = hwMap.dcMotor.get("ShooterLeft");

        ShooterLeft.setDirection(DcMotor.Direction.FORWARD);
        ShooterRight.setDirection(DcMotor.Direction.REVERSE);

        ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterLeft.setMaxSpeed(SHOOTER_MAX_SPEED);
        ShooterRight.setMaxSpeed(SHOOTER_MAX_SPEED);

    }

    public ParticleAccelerator(String ShooterRight, String ShooterLeft, HardwareMap hardwareMap) {
        this.ShooterRight = hardwareMap.dcMotor.get(ShooterRight);
        this.ShooterLeft = hardwareMap.dcMotor.get(ShooterLeft);
    }

    public void run() {
        ShooterLeft.setPower(SHOOTER_LEFT_POWER);
        ShooterRight.setPower(SHOOTER_RIGHT_POWER);

        AcceleratorState = acceleratorEnum.Run;
    }


    public void stop() {
        ShooterLeft.setPower(0);
        ShooterRight.setPower(0);
        AcceleratorState = acceleratorEnum.Stop;
    }

    @Override
    public String toString() {
        switch (AcceleratorState){
            case Run:
                return "Running";
            case Stop:
                return "Stopped";
            default:
                return "Unknown";



        }
    }
}


