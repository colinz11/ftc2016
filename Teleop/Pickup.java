package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Pickup {



    private enum pickupEnum {Run, Stop}
    private pickupEnum PickupState = pickupEnum.Stop;


    public DcMotor Pickup = null;
    HardwareMap hwMap = null;


    public void init(HardwareMap ahwMap) {
        //Save reference to Hardware map
        hwMap = ahwMap;
        Pickup = hwMap.dcMotor.get("Pickup");

        Pickup.setDirection(DcMotor.Direction.FORWARD);


    }

    public Pickup(String Pickup, HardwareMap hardwareMap) {
        this.Pickup = hardwareMap.dcMotor.get(Pickup);
    }

    public void Reverse()
    {
        Pickup.setDirection(DcMotor.Direction.REVERSE);
    }

    public void Forward()
    {
        Pickup.setDirection(DcMotor.Direction.FORWARD);
    }

    public void run() {
        Pickup.setPower(1.0); PickupState = pickupEnum.Run;
    }

    public void stop() {
        Pickup.setPower(0); PickupState = pickupEnum.Stop;
    }
    @Override
    public String toString() {
        switch (PickupState){
            case Run:
                return "Running";
            case Stop:
                return "Stopped";
            default:
                return "Unknown";
        }
    }
}


