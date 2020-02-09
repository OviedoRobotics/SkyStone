package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.RobotUtilities.MovementVars;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

/**
 *Created by 12090 STEM Punk
 */
public class HardwareOmnibotDrive
{
    /* Public OpMode members. */
    public final static double STRAFE_MULTIPLIER = 1.5;
    public final static double SLOW_STRAFE_MULTIPLIER = 1.0;
    public final static double MIN_FOUNDATION_SPIN_RATE = 0.19;
    public final static double MIN_FOUNDATION_DRIVE_RATE = 0.18;
    public final static double MIN_FOUNDATION_STRAFE_RATE = 0.19;
    public final static double MIN_SPIN_RATE = 0.12;
    public final static double MIN_DRIVE_RATE = 0.10;
    public final static double MIN_STRAFE_RATE = 0.19;
    public final static double MIN_DRIVE_MAGNITUDE = Math.sqrt(MIN_DRIVE_RATE*MIN_DRIVE_RATE+MIN_DRIVE_RATE*MIN_DRIVE_RATE);
    public final static double MIN_FOUNDATION_DRIVE_MAGNITUDE = Math.sqrt(MIN_FOUNDATION_DRIVE_RATE*MIN_FOUNDATION_DRIVE_RATE+MIN_FOUNDATION_DRIVE_RATE*MIN_FOUNDATION_DRIVE_RATE);

    // Robot Controller Config Strings
    public final static String IMU = "imu";
    public final static String FRONT_LEFT_MOTOR = "FrontLeft";
    public final static String FRONT_RIGHT_MOTOR = "FrontRight";
    public final static String REAR_LEFT_MOTOR = "RearLeft";
    public final static String REAR_RIGHT_MOTOR = "RearRight";
    public final static String LEFT_INTAKE = "LeftIntake";
    public final static String RIGHT_INTAKE = "RightIntake";
    public final static String EXTENDER = "Extender";
    // We need both hubs here because one has the motors, and the other has the
    // odometry encoders.
    public final static String HUB1 = "Expansion Hub 2";
    public final static String HUB2 = "Expansion Hub 3";

    List<LynxModule> allHubs;

    // These motors have the odometry encoders attached
    protected DcMotorEx leftIntake = null;
    protected DcMotorEx rightIntake = null;
    protected DcMotorEx extender = null;

    protected DcMotorEx frontLeft = null;
    protected DcMotorEx frontRight = null;
    protected DcMotorEx rearLeft = null;
    protected DcMotorEx rearRight = null;
    protected BNO055IMU imu = null;

    // Tracking variables
    private static final int encoderClicksPerSecond = 2800;
    protected double frontLeftMotorPower = 0.0;
    protected double rearLeftMotorPower = 0.0;
    protected double frontRightMotorPower = 0.0;
    protected double rearRightMotorPower = 0.0;
    public boolean defaultInputShaping = true;
    protected boolean imuRead = false;
    protected double imuValue = 0.0;
    protected double strafeMultiplier = STRAFE_MULTIPLIER;

    public static boolean encodersReset = false;
    public boolean forceReset = false;

    public double xAngle, yAngle, zAngle;
    /* local OpMode members. */
    protected HardwareMap hwMap  =  null;

    /* Constructor */
    public HardwareOmnibotDrive(){
    }

    /**
     * If the motion value is less than the threshold, the controller will be
     * considered at rest
     */
    protected static float joystickDeadzone = 0.07f;
    private static final float MAX_MOTION_RANGE = 1.0f;
    private static final float MIN_MOTION_RANGE = (float)MIN_DRIVE_RATE;

    // Used to clean up the slop in the joysticks.
    protected static float cleanMotionValues(float number) {
        // apply deadzone
        if (number < joystickDeadzone && number > -joystickDeadzone) return 0.0f;
        // apply trim
        if (number >  MAX_MOTION_RANGE) return  MAX_MOTION_RANGE;
        if (number < -MAX_MOTION_RANGE) return -MAX_MOTION_RANGE;
        // scale values "between deadzone and trim" to be "between Min range and Max range"
        if (number > 0)
            number = (float) Range.scale(number, joystickDeadzone, MAX_MOTION_RANGE, MIN_MOTION_RANGE, MAX_MOTION_RANGE);
        else
            number = (float)Range.scale(number, -joystickDeadzone, -MAX_MOTION_RANGE, -MIN_MOTION_RANGE, -MAX_MOTION_RANGE);

        return number;
    }

    public int getLeftEncoderWheelPosition() {
        // This is to compensate for GF having a negative left.
        return -leftIntake.getCurrentPosition();
    }

    public int getRightEncoderWheelPosition() {
        return rightIntake.getCurrentPosition();
    }

    public int getStrafeEncoderWheelPosition() {
        return extender.getCurrentPosition();
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Motor: Lifter, RightIntake, Extender, LeftIntake
        // Encoder: Lifter, LeftEncoder, CenterEncoder, RightEncoder
//        expansionHub1 = hwMap.get(ExpansionHubEx.class, HUB1);
        // RearRight, RearLeft, FrontLeft, FrontRight
//        expansionHub2 = hwMap.get(ExpansionHubEx.class, HUB2);

        // Define and Initialize Motors
        frontLeft = hwMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR);
        frontRight = hwMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR);
        rearLeft = hwMap.get(DcMotorEx.class, REAR_LEFT_MOTOR);
        rearRight = hwMap.get(DcMotorEx.class, REAR_RIGHT_MOTOR);
        leftIntake = hwMap.get(DcMotorEx.class, LEFT_INTAKE);
        rightIntake = hwMap.get(DcMotorEx.class, RIGHT_INTAKE);
        extender = hwMap.get(DcMotorEx.class, EXTENDER);


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        setAllDriveZero();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the stop mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Let's try to tweak the PIDs
//		frontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,
//                3, 0, 12, MotorControlAlgorithm.PIDF));
//        frontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,
//                3, 0, 12, MotorControlAlgorithm.PIDF));
//        rearLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,
//                3, 0, 12, MotorControlAlgorithm.PIDF));
//        rearRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,
//                3, 0, 12, MotorControlAlgorithm.PIDF));

        initIMU();
    }

    public void setInputShaping(boolean inputShapingEnabled) {
        defaultInputShaping = inputShapingEnabled;
    }

    public void initIMU()
    {
        // Init IMU code
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, IMU);
        imu.initialize(parameters);
    }

    public void resetReads() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        imuRead = false;
    }

    public double readIMU()
    {
        if(!imuRead) {
            // Read IMU Code
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            imuValue = (double)angles.firstAngle;
            imuRead = true;
        }

        return imuValue;
    }

    public void setFrontLeftMotorPower(double power)
    {
        if(Math.abs(power - frontLeftMotorPower) > 0.005)
        {
            frontLeftMotorPower = power;
            frontLeft.setPower(power);
        }
    }

    public void setRearLeftMotorPower(double power)
    {
        if(Math.abs(power - rearLeftMotorPower) > 0.005)
        {
            rearLeftMotorPower = power;
            rearLeft.setPower(power);
        }
    }

    public void setFrontRightMotorPower(double power)
    {
        if(Math.abs(power - frontRightMotorPower) > 0.005)
        {
            frontRightMotorPower = power;
            frontRight.setPower(power);
        }
    }

    public void setRearRightMotorPower(double power)
    {
        if(Math.abs(power - rearRightMotorPower) > 0.005)
        {
            rearRightMotorPower = power;
            rearRight.setPower(power);
        }
    }

    public void setAllDrive(double power) {
        setFrontLeftMotorPower(power);
        setFrontRightMotorPower(power);
        setRearRightMotorPower(power);
        setRearLeftMotorPower(power);
    }

    public void setAllDriveZero()
    {
        setAllDrive(0.0);
    }

    /**
     *
     * @param xPower - -1.0 to 1.0 power in the X axis
     * @param yPower - -1.0 to 1.0 power in the Y axis
     * @param spin - -1.0 to 1.0 power to rotate the robot, reduced to MAX_SPIN_RATE
     * @param angleOffset - The offset from the gyro to run at, such as drive compensation
     */
    public void drive(double xPower, double yPower, double spin, double angleOffset, boolean inputShaping) {
        double gyroAngle = readIMU() + angleOffset;
        double joystickMagnitude = sqrt(xPower*xPower + yPower*yPower);
        double driveAngle = atan2(yPower, xPower);
        double robotDriveAngle = driveAngle - Math.toRadians(gyroAngle) + Math.toRadians(90);
        double newPower = driverInputShaping(joystickMagnitude, inputShaping);

        MovementVars.movement_turn = driverInputSpinShaping(spin, inputShaping);
        MovementVars.movement_x = newPower * cos(robotDriveAngle);
        MovementVars.movement_y = newPower * sin(robotDriveAngle);

		ApplyMovement();
    }

    protected double driverInputShaping( double valueIn, boolean inputShaping) {
        double aValue = 0.77;
        double valueOut = 0.0;

        if(valueIn == 0.0) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                valueOut = aValue * Math.pow(valueIn, 3) + (1 - aValue) * valueIn;
                valueOut = Math.copySign(Math.max(MIN_DRIVE_RATE, Math.abs(valueOut)), valueOut);
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    protected double driverInputSpinShaping( double valueIn, boolean inputShaping) {
        double aValue = 0.77;
        double valueOut;

        if(valueIn == 0.0) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                valueOut = aValue * Math.pow(valueIn, 3) + (1 - aValue) * valueIn;
                valueOut = Math.copySign(Math.max(MIN_SPIN_RATE, Math.abs(valueOut)), valueOut);
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    public void disableDriveEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoders()
    {
        int sleepTime = 0;
        int encoderCount = frontLeft.getCurrentPosition();

        // The Odometry Encoders
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) { break; }
            sleepTime += 10;
            resetReads();
            encoderCount = frontLeft.getCurrentPosition();
        }

        // The Odometry Encoders
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @param targetAngle  - The angle the robot should try to face when reaching destination.
     * @param pullingFoundation - If we are pulling the foundation.
     * @param resetDriveAngle - When we start a new drive, need to reset the starting drive angle.
     * @return - Boolean true we have reached destination, false we have not
     */
    public double lastDriveAngle;
    public boolean rotateToAngle(double targetAngle, boolean pullingFoundation, boolean resetDriveAngle) {
        boolean reachedDestination = false;
        double errorMultiplier = pullingFoundation ? 0.04 : 0.016;
        double minSpinRate = pullingFoundation ? MIN_FOUNDATION_SPIN_RATE : MIN_SPIN_RATE;
        double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;

        // This should be set on the first call to start us on a new path.
        if(resetDriveAngle) {
            lastDriveAngle = deltaAngle;
        }

        // We are done if we are within 2 degrees
        if(Math.abs(Math.toDegrees(deltaAngle)) < 2) {
            // We have reached our destination if the angle is close enough
            setAllDriveZero();
            reachedDestination = true;
            // We are done when we flip signs.
        } else if(lastDriveAngle < 0) {
            // We have reached our destination if the delta angle sign flips from last reading
            if(deltaAngle >= 0) {
                setAllDriveZero();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                MovementVars.movement_x = 0;
                MovementVars.movement_y = 0;
                if(turnSpeed > -minSpinRate) {
                    turnSpeed = -minSpinRate;
                }
                MovementVars.movement_turn = turnSpeed;
                ApplyMovement();
            }
        } else {
            // We have reached our destination if the delta angle sign flips
            if(deltaAngle <= 0) {
                setAllDriveZero();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                MovementVars.movement_x = 0;
                MovementVars.movement_y = 0;
                if(turnSpeed < minSpinRate) {
                    turnSpeed = minSpinRate;
                }
                MovementVars.movement_turn = turnSpeed;
                ApplyMovement();
            }
        }
        lastDriveAngle = deltaAngle;

        return reachedDestination;
    }

    /**
     * @param x           - The X field coordinate to go to.
     * @param y           - The Y field coordinate to go to.
     * @param targetAngle - The angle the robot should try to face when reaching destination in radians.
     * @param minSpeed    - The minimum speed that allows movement.
     * @param maxSpeed    - Sets the maximum speed to drive.
     * @param errorMultiplier - Sets the proportional speed to slow down.
     * @param allowedError - Sets the allowable error to claim target reached.
     * @param passThrough - Allows waypoint to be a drive through where the robot won't slow down.
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean driveToXY(double x, double y, double targetAngle, double minSpeed,
                             double maxSpeed, double errorMultiplier, double allowedError,
                             boolean passThrough) {
        boolean reachedDestination = false;
        double deltaX = x - MyPosition.worldXPosition;
        double deltaY = y - MyPosition.worldYPosition;
        double driveAngle = Math.atan2(deltaY, deltaX);
        double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
        double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double driveSpeed;
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;
        // Have to convert from world angles to robot centric angles.
        double robotDriveAngle = driveAngle - MyPosition.worldAngle_rad + Math.toRadians(90);

        // This will allow us to do multi-point routes without huge slowdowns.
        // Such use cases will be changing angles, or triggering activities at
        // certain points.
        if(!passThrough) {
            driveSpeed = magnitude * errorMultiplier;
        } else {
            driveSpeed = maxSpeed;
        }

        if(driveSpeed < minSpeed) {
            driveSpeed = minSpeed;
        } else if (driveSpeed > maxSpeed) {
            driveSpeed = maxSpeed;
        }

        // Check if we passed through our point
        if(magnitude <= allowedError) {
            reachedDestination = true;
            if(!passThrough) {
                setAllDriveZero();
            } else {
                // This can happen if the robot is already at error distance for drive through
                MovementVars.movement_x = driveSpeed * Math.cos(robotDriveAngle);
                MovementVars.movement_y = driveSpeed * Math.sin(robotDriveAngle);
                MovementVars.movement_turn = turnSpeed;
                ApplyMovement();
            }
        } else {
            MovementVars.movement_x = driveSpeed * Math.cos(robotDriveAngle);
            MovementVars.movement_y = driveSpeed * Math.sin(robotDriveAngle);
            MovementVars.movement_turn = turnSpeed;
            ApplyMovement();
        }

        return reachedDestination;
    }
    /**
     * @param x           - The X field coordinate to go to.
     * @param y           - The Y field coordinate to go to.
     * @param targetAngle  - The angle the robot should try to face when reaching destination in radians.
     * @param maxSpeed    - Sets the speed when we are driving through the point.
     * @param passThrough - Slows the robot down to stop at destination coordinate.
     * @param pullingFoundation - If we are pulling the foundation.
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean driveToXY(double x, double y, double targetAngle, double maxSpeed,
                             boolean passThrough, boolean pullingFoundation) {
        double errorMultiplier = pullingFoundation ? 0.020 : 0.014;
        double minDriveMagnitude = pullingFoundation ? MIN_FOUNDATION_DRIVE_MAGNITUDE : MIN_DRIVE_MAGNITUDE;
        double allowedError = 2;

        if(passThrough) {
            allowedError = 7;
        }
        return (driveToXY(x, y, targetAngle, minDriveMagnitude, maxSpeed, errorMultiplier,
                allowedError, passThrough));
    }

    // Odometry updates
    private long lastUpdateTime = 0;

    /**converts movement_y, movement_x, movement_turn into motor powers */
    public void ApplyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        // 2.1 is the ratio between the minimum power to strafe, 0.19, and driving, 0.09.
        double tl_power_raw = MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*strafeMultiplier;
        double bl_power_raw = MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*strafeMultiplier;
        double br_power_raw = -MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*strafeMultiplier;
        double tr_power_raw = -MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*strafeMultiplier;

        //find the maximum of the powers
        double maxRawPower = Math.abs(tl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(tr_power_raw) > maxRawPower){ maxRawPower = Math.abs(tr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        tr_power_raw *= scaleDownAmount;

        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        setFrontLeftMotorPower(tl_power_raw);
        setFrontRightMotorPower(tr_power_raw);
        setRearRightMotorPower(br_power_raw);
        setRearLeftMotorPower(bl_power_raw);
    }
}

