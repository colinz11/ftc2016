package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by SaajanPatel on 12/3/16.
 */

@TeleOp(name="Teleop version 1.0", group="Teleop2016")
 class Teleop_Iteration_1 extends OpMode{
    GamepadWrapper joy1;
    GamepadWrapper joy2;
    Drivetrain drivetrain = new Drivetrain();
    ParticleAccelerator Shooter;
    Pickup Pickup;
    //Column column;

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        Pickup = new Pickup("Pickup", hardwareMap);
        Shooter = new ParticleAccelerator("ShooterLeft", "ShooterRight", hardwareMap);
        // column = new Column("Column", hardwareMap);
        joy1 = new GamepadWrapper();
        joy2 = new GamepadWrapper();
    }

    @Override
    public void init_loop(){
    }

    @Override
    public void start(){
    }

    @Override
    public void loop() {
        joy1.update(gamepad1);
        joy2.update(gamepad2);
        double left;
        double right;
        left = -gamepad1.right_stick_y;
        right = -gamepad1.left_stick_y;
        //drivetrain.Left1.setPower(left);
        //drivetrain.Left2.setPower(left);
        drivetrain.Right1.setPower(right);
        drivetrain.Right2.setPower(right);

        if (joy1.toggle.right_bumper) {
            Pickup.run();
        }
        else {
            Pickup.stop();
        }

        if (joy1.toggle.left_bumper) {

            Shooter.run();
            //ShooterRight.run();
        }
        else {
            Shooter.stop();
        //    ShooterRight.stop();
        }

        if (joy1.toggle.left_stick_button)
        {
            Shooter.run();
            Pickup.stop();
        }
        else{
            Shooter.stop();
            Pickup.stop();
        }


        telemetry.addData("left", "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("Shooter", Shooter);
      //telemetry.addData("ShooterRight", ShooterRight);
        telemetry.addData("Pickup", Pickup);

       // telemetry.addData("Column", column);
    }
    }

________________________________________


