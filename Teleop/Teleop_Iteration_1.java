package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Teleop version 1.0", group="Teleop2016")
    class Teleop_Iteration_1 extends OpMode{
    GamepadWrapper joy1;
    GamepadWrapper joy2;
    Drivetrain drivetrain = new Drivetrain();
    CapBall capBall;
    ParticleAccelerator Shooter;
    org.firstinspires.ftc.teamcode.Teleop.Pickup Pickup;

    Servo rightServo;
    Servo leftServo;

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        Pickup = new Pickup("Pickup", hardwareMap);
        Shooter = new ParticleAccelerator("ShooterLeft", "ShooterRight", hardwareMap);
        capBall = new CapBall("Cap Ball", hardwareMap);

        rightServo = hardwareMap.servo.get("right servo");

        leftServo = hardwareMap.servo.get("left servo");

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
        double posLeft = leftServo.getPosition();
        double posRight = rightServo.getPosition();

        joy1.update(gamepad1);
        joy2.update(gamepad2);
        double left;
        double right;
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        drivetrain.Left1.setPower(left);
        drivetrain.Left2.setPower(left);
        drivetrain.Right1.setPower(right);
        drivetrain.Right2.setPower(right);

        if (joy2.toggle.a)
        {
            leftServo.setPosition(0.625);
            rightServo.setPosition(0.575);
        }
        else {
            leftServo.setPosition(0.0);
            rightServo.setPosition(1.0);
        }

        if (joy2.toggle.dpad_up){
            capBall.forward();
            capBall.run();
        }
        else if (!joy2.toggle.dpad_up && !joy2.toggle.dpad_down){
            capBall.forward();
            capBall.stop();
        }

        if (joy2.toggle.dpad_down){
            capBall.reverse();
            capBall.run();
        }
        else if (!joy2.toggle.dpad_up && !joy2.toggle.dpad_down){
            capBall.reverse();
            capBall.stop();
        }

        if (joy2.toggle.left_bumper) {
            Pickup.Reverse();
            Pickup.run();
        }
        else if (!joy2.toggle.b &&!joy2.toggle.left_bumper)   {
            Pickup.Reverse();
            Pickup.stop();
        }

        if (joy2.toggle.right_bumper){
        Shooter.run();
        }
        else {
            Shooter.stop();
        }

        if (joy2.toggle.b  ) {
            Pickup.Forward();
            Pickup.run();
        }
        else if (!joy2.toggle.left_bumper && !joy2.toggle.b  ){
            Pickup.Reverse();
            Pickup.stop();
        }

        telemetry.addData("left", "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("Shooter", Shooter);
        telemetry.addData("Pickup", Pickup);
        telemetry.addData("Left Servo", posLeft);
        telemetry.addData("Right Servo", posRight);
        telemetry.update();

    }
}
