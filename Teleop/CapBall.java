package org.firstinspires.ftc.teamcode.Teleop;
import android.os.SystemClock;
import android.provider.Settings;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Colin Zhu on 12/3/16.
 */

class CapBall {
    private enum acceleratorEnum {Run, Stop}

    private acceleratorEnum AcceleratorState = acceleratorEnum.Stop;

    public DcMotor capBall = null;
    HardwareMap hwMap = null;

    //Max encoder clicks per second




    public void init(HardwareMap ahwMap) {
        //Save reference to Hardware map
        hwMap = ahwMap;
        capBall = hwMap.dcMotor.get("Cap Ball");

        capBall.setDirection(DcMotorSimple.Direction.FORWARD);

        capBall.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public CapBall(String capBall, HardwareMap hardwareMap) {
        this.capBall = hardwareMap.dcMotor.get(capBall);
    }

    public void run() {
        capBall.setPower(1);

        AcceleratorState = acceleratorEnum.Run;
    }


    public void stop() {
        capBall.setPower(0);

        AcceleratorState = acceleratorEnum.Stop;
    }

    public void reverse(){
        capBall.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void forward(){
        capBall.setDirection(DcMotorSimple.Direction.FORWARD);
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


