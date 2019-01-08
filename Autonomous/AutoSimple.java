package org.firstinspires.ftc.teamcode.Autonomous;
import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Colin Zhu on 1/16/17.
 */

@Autonomous(name="Auto Simple", group="Hardware Problem")
public class AutoSimple extends LinearOpMode {
    public ColorSensor ColorSensor = null;

    public DcMotor ShooterRight = null;
    public DcMotor ShooterLeft = null;

    public DcMotor Pickup = null;

    public DcMotor Left1 = null;
    public DcMotor Left2 = null;
    public DcMotor Right1 = null;
    public DcMotor Right2 = null;

    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    Servo leftServo;
    Servo rightServo;
    OpticalDistanceSensor opticalDistanceSensor;


    double POWER = 1;

    //Max encoder clicks per second
    public static final int SHOOTER_MAX_SPEED = 4000;

    //Percentage of speed
    public static final double SHOOTER_POWER = 0.475;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("opticalDistanceSensor");

        ColorSensor = hardwareMap.colorSensor.get("Color Sensor");

        leftServo = hardwareMap.servo.get("left servo");
        rightServo = hardwareMap.servo.get("right servo");

        //init Shooter
        ShooterRight = hardwareMap.dcMotor.get("ShooterRight");
        ShooterLeft = hardwareMap.dcMotor.get("ShooterLeft");

        ShooterLeft.setDirection(DcMotor.Direction.FORWARD);
        ShooterRight.setDirection(DcMotor.Direction.REVERSE);

        ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterLeft.setMaxSpeed(SHOOTER_MAX_SPEED);
        ShooterRight.setMaxSpeed(SHOOTER_MAX_SPEED);

        //init Pickup
        Pickup = hardwareMap.dcMotor.get("Pickup");

        Pickup.setDirection(DcMotor.Direction.REVERSE);

        //init Drivetrain
        Left1 = hardwareMap.dcMotor.get("Left1");
        Left2 = hardwareMap.dcMotor.get("Left2");
        Right1 = hardwareMap.dcMotor.get("Right1");
        Right2 = hardwareMap.dcMotor.get("Right2");

        Left1.setDirection(DcMotor.Direction.REVERSE);
        Left2.setDirection(DcMotor.Direction.REVERSE);
        Right1.setDirection(DcMotor.Direction.FORWARD);
        Right2.setDirection(DcMotor.Direction.FORWARD);

        Left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        Left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        //Start Autonomous
        while(opModeIsActive() && (runtime.seconds() < 15)){
        }

        encoderDrive(1.0, -17.5, -17.5, 10000);

        runtime.reset();

        ShooterLeft.setPower(-SHOOTER_POWER);
        ShooterRight.setPower(-SHOOTER_POWER);
        //Start Shooter
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("shooter", "starting");
            telemetry.update();

            idle();
        }

        runtime.reset();

        ShooterLeft.setPower(-SHOOTER_POWER);
        ShooterRight.setPower(-SHOOTER_POWER);

        Pickup.setPower(POWER);
        //Start Pickup
        while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("shooter", "pickup");
            telemetry.update();

            idle();
        }

        ShooterLeft.setPower(0);
        ShooterRight.setPower(0);
        Pickup.setPower(0);

        encoderDrive(1.0, -10, -10, 10000);


        runtime.reset();




    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeft1Target;
        int newLeft2Target;
        int newRight1Target;
        int newRight2Target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //Determine new target position, and pass to motor controller
            newLeft1Target = Left1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeft2Target = Left2.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRight1Target = Right1.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRight2Target = Right2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            Left1.setTargetPosition(newLeft1Target);
            Left2.setTargetPosition(newLeft2Target);
            Right1.setTargetPosition(newRight1Target);
            Right2.setTargetPosition(newRight2Target);

            // Turn On RUN_TO_POSITION
            Left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            Left1.setPower(Math.abs(speed));
            Left2.setPower(Math.abs(speed));
            Right1.setPower(Math.abs(speed));
            Right2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Left1.isBusy() && Right1.isBusy())) {//MAYBE ADD RIGHT2 AND LEFT2

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeft1Target,  newRight1Target);
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeft2Target,  newRight2Target);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        Left1.getCurrentPosition(),
                        Right1.getCurrentPosition());
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        Left1.getCurrentPosition(),
                        Right1.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            Left1.setPower(0);
            Left2.setPower(0);
            Right1.setPower(0);
            Right2.setPower(0);

            // Turn off RUN_TO_POSITION
            Left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(1000);   // optional pause after each move
        }
    }


}
