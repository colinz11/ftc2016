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

@Autonomous(name="Red", group="Hardware Problem")
public class RedAutonomous extends LinearOpMode {

    public ColorSensor ColorSensor = null;

    public DcMotor ShooterRight = null;
    public DcMotor ShooterLeft = null;

    public DcMotor Pickup = null;

    public DcMotor Left1 = null;
    public DcMotor Left2 = null;
    public DcMotor Right1 = null;
    public DcMotor Right2 = null;

    Servo rightServo;
    Servo leftServo;

    int zAccumulated;  //Total rotation left/right
    int target = 0;  //Desired angle to turn to

    OpticalDistanceSensor odsSensor;

    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    double left1Power, left2Power, right1Power, right2Power, correction;

    //Max encoder clicks per second
    public static final int SHOOTER_MAX_SPEED = 4000;

    //Percentage of speed
    public static final double SHOOTER_POWER = 0.49;



    double POWER = 1;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {



        odsSensor = hardwareMap.opticalDistanceSensor.get("opticalDistanceSensor");

        ColorSensor = hardwareMap.colorSensor.get("Color Sensor");

        rightServo = hardwareMap.servo.get("right servo");
        leftServo = hardwareMap.servo.get("left servo");


        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID
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

        Left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        Left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        mrGyro.calibrate();

        while (!isStopRequested() && mrGyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        Left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", mrGyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }
        mrGyro.resetZAxisIntegrator();

        ColorSensor.enableLed(false);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();//Start

        ColorSensor.enableLed(false);


        rightServo.setPosition(1.0);
        leftServo.setPosition(0.0);

        ShooterLeft.setPower(-SHOOTER_POWER);
        ShooterRight.setPower(-SHOOTER_POWER);


        encoderDrive(1.0, -16, -16, 10000);

        runtime.reset();

        Pickup.setPower(POWER);
        //Start Pickup
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("shooter", "pickup");
            telemetry.update();

            idle();
        }

        ShooterLeft.setPower(0);
        ShooterRight.setPower(0);
        Pickup.setPower(0);
        runtime.reset();

        turn(-120);
        encoderDrive(1.0, 32, 32, 10000);
        turn(40);
        encoderDrive(1.0, 6, 6, 10000);

        double red = ColorSensor.red();
        double blue = ColorSensor.blue();

        if (ColorSensor.red() > ColorSensor.blue() && ColorSensor.red() > ColorSensor.green()) {
            leftServo.setPosition(0.625);
            rightServo.setPosition(1.0);
            while(opModeIsActive() && runtime.seconds() < 0.5){
            }
            encoderDrive(1.0, 10, 10, 10000);
            while(opModeIsActive() && runtime.seconds() < 0.5){
            }
            encoderDrive(1.0, -10, -10, 10000);
            leftServo.setPosition(0.0);
            rightServo.setPosition(1.0);
            telemetry.addData("blue", blue);
            telemetry.update();

        }
        else if (ColorSensor.blue() > ColorSensor.red() && ColorSensor.blue() > ColorSensor.green()) {

            rightServo.setPosition(0.575);
            leftServo.setPosition(0.0);
            while(opModeIsActive() && runtime.seconds() < 0.5){
            }
            encoderDrive(1.0, 10, 10, 10000);
            while(opModeIsActive() && runtime.seconds() < 0.5){
            }
            encoderDrive(1.0, -10, -10, 10000);
            rightServo.setPosition(1.0);
            leftServo.setPosition(0.0);
            telemetry.addData("red", red);
            telemetry.update();
        }

        turn(-90);
        encoderDrive(1.0, 34, 34, 10000);
        turn(90);


        if (ColorSensor.red() > ColorSensor.blue() && ColorSensor.red() > ColorSensor.green()) {
            leftServo.setPosition(0.625);
            rightServo.setPosition(1.0);
            while(opModeIsActive() && runtime.seconds() < 0.5){
            }
            encoderDrive(1.0, 10, 10, 10000);
            leftServo.setPosition(0.0);
            rightServo.setPosition(1.0);
            telemetry.addData("blue", blue);
            telemetry.update();

        }
        else if (ColorSensor.blue() > ColorSensor.red() && ColorSensor.blue() > ColorSensor.green()) {

            rightServo.setPosition(0.575);
            leftServo.setPosition(0.0);
            while(opModeIsActive() && runtime.seconds() < 0.5){
            }
            encoderDrive(1.0, 10, 10, 10000);
            rightServo.setPosition(1.0);
            leftServo.setPosition(0.0);
            telemetry.addData("red", red);
            telemetry.update();
        }
        else {
        }


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

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeft1Target;
        int     newLeft2Target;
        int     newRight1Target;
        int     newRight2Target;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeft1Target = Left1.getCurrentPosition() + moveCounts;
            newLeft2Target = Left2.getCurrentPosition() + moveCounts;
            newRight1Target = Right1.getCurrentPosition() + moveCounts;
            newRight2Target = Right2.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            Left1.setTargetPosition(newLeft1Target);
            Left2.setTargetPosition(newLeft2Target);
            Right1.setTargetPosition(newRight1Target);
            Right2.setTargetPosition(newRight2Target);

            Left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            Left1.setPower(speed);
            Left2.setPower(speed);
            Right1.setPower(speed);
            Right2.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (Left1.isBusy() && Right1.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                Left1.setPower(leftSpeed);
                Left2.setPower(leftSpeed);
                Right1.setPower(rightSpeed);
                Right2.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeft1Target,  newRight2Target, newLeft2Target, newRight1Target);
                telemetry.addData("Actual",  "%7d:%7d",      Left1.getCurrentPosition(),
                        Right1.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
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
        }
    }
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        Left1.setPower(0);
        Left2.setPower(0);
        Right1.setPower(0);
        Right2.setPower(0);
    }
    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        Left1.setPower(leftSpeed);
        Left2.setPower(leftSpeed);
        Right1.setPower(rightSpeed);
        Right2.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - mrGyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers trn left.
    public void turnAbsolute(int target) {
        zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.15;

        while (Math.abs(zAccumulated - target) > 4 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                Left1.setPower(turnSpeed);
                Left2.setPower(turnSpeed);
                Right1.setPower(-turnSpeed);
                Right2.setPower(-turnSpeed);
                telemetry.addData("Turning left",2);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                Left1.setPower(-turnSpeed);
                Left2.setPower(-turnSpeed);
                Right1.setPower(turnSpeed);
                Right2.setPower(turnSpeed);
                telemetry.addData("Turning right",2);
            }

            zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }

        Left1.setPower(0);  //Stop the motors
        Right1.setPower(0);
        Left2.setPower(0);  //Stop the motors
        Right2.setPower(0);

    }
    //This function turns a number of degrees compared to where the robot is. Positive numbers trn left.
    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + mrGyro.getIntegratedZValue());
    }

}
