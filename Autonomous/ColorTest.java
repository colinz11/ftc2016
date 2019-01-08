package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Colin_Zhu on 2/18/2017.
 */
@Autonomous(name="Test", group="Hardware Problem")
public class ColorTest extends LinearOpMode{
    public ColorSensor ColorSensor = null;
    Servo rightServo;
    Servo leftServo;

    @Override
    public void runOpMode() throws InterruptedException {

        ColorSensor = hardwareMap.colorSensor.get("Color Sensor");

        rightServo = hardwareMap.servo.get("right servo");
        leftServo = hardwareMap.servo.get("left servo");
        double red = ColorSensor.red();
        double blue = ColorSensor.blue();

        ColorSensor.enableLed(true);
        waitForStart();

        leftServo.setPosition(0.0);
        rightServo.setPosition(1.0);

        while(opModeIsActive()) {
            double leftPosition = leftServo.getPosition();
            double rightPosition = rightServo.getPosition();

            if (ColorSensor.red() > ColorSensor.blue() && ColorSensor.red() > ColorSensor.green()) {

                rightServo.setPosition(0.575);
                leftServo.setPosition(0.0);
                telemetry.addData("left", rightPosition);
                telemetry.addData("right", rightPosition);
                telemetry.addData("red", red);
                telemetry.update();
            }
            else if (ColorSensor.blue() > ColorSensor.red() && ColorSensor.blue() > ColorSensor.green()) {
                leftServo.setPosition(0.625);
                rightServo.setPosition(1.0);

                telemetry.addData("left", leftPosition);
                telemetry.addData("right", rightPosition);
                telemetry.addData("blue", blue);
                telemetry.update();

            }
            else {
            }
        }
    }
}
