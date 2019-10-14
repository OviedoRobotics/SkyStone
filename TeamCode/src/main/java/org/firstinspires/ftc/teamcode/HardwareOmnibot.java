package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 *Created by Ethan
 */
public class HardwareOmnibot extends HardwareOmnibotDrive
{
    public enum LiftActivity {
        IDLE,
        GRABBING_STONE,
        LIFTING_TO_ROTATE,
        ROTATING,
        LIFTING_TO_STONE,
        STOPPING
    }

    public enum ReleaseActivity {
        IDLE,
        ALIGN_TO_FOUNDATION,
        RELEASE_STONE,
        STOPPING
    }

    public enum StowActivity {
        IDLE,
        LOWERING_TO_ROTATE,
        ROTATING,
        LOWERING_TO_STOW,
        STOPPING
    }

    public enum ExtendPosition {
        RETRACTED(10),
		CAPSTONE(1036),
		SPINMIN(1425),
		EJECT(1618),
		EXTENDED(1877);

        private final int encoderCount;

        ExtendPosition(int encoderCount)
		{
			this.encoderCount = encoderCount;
		}

        public int getEncoderCount()
		{
			return encoderCount;
		}
	}

    public enum LiftPosition {
        STOWED(10),
        STONE1(149),
        ROTATE(372),
        STONE2(412),
        STONE3(660),
        STONE4(908),
        STONE5(1156),
        STONE6(1403),
        STONE7(1651),
        STONE8(1899),
        STONE9(2147),
        STONE10(2395),
        STONE11(2643),
        STONE12(2890);

        private final int encoderCount;

        LiftPosition(int encoderCount)
		{
			this.encoderCount = encoderCount;
		}

        public int getEncoderCount()
		{
			return encoderCount;
		}

		public static LiftPosition addStone(LiftPosition currentStone)
		{
			switch(currentStone)
			{
				case STOWED:
					return STONE1;
				case STONE1:
					return STONE2;
				case ROTATE:
					return STONE2;
				case STONE2:
					return STONE3;
				case STONE3:
					return STONE4;
				case STONE4:
					return STONE5;
				case STONE5:
					return STONE6;
				case STONE6:
					return STONE7;
				case STONE7:
					return STONE8;
				case STONE8:
					return STONE9;
				case STONE9:
					return STONE10;
				case STONE10:
					return STONE11;
				case STONE11:
					return STONE12;
				case STONE12:
					return STONE12;
				default:
					return currentStone;
			}
		}

		public static LiftPosition removeStone(LiftPosition currentStone)
		{
			switch(currentStone)
			{
				case STOWED:
					return STONE1;
				case STONE1:
					return STONE1;
				case ROTATE:
					return STONE1;
				case STONE2:
					return STONE1;
				case STONE3:
					return STONE2;
				case STONE4:
					return STONE3;
				case STONE5:
					return STONE4;
				case STONE6:
					return STONE5;
				case STONE7:
					return STONE6;
				case STONE8:
					return STONE7;
				case STONE9:
					return STONE8;
				case STONE10:
					return STONE9;
				case STONE11:
					return STONE10;
				case STONE12:
					return STONE11;
				default:
					return currentStone;
			}
		}
    }

    /* Public OpMode members. */
    public static double RIGHT_FINGER_DOWN = 0.32;
    public static double LEFT_FINGER_DOWN = 0.82;
    public static double RIGHT_FINGER_UP = 0.89;
    public static double LEFT_FINGER_UP = 0.25;
    public static double CLAW_OPEN = 0.0;
    public static double CLAW_PINCHED = 1.0;
    public static double CLAWDRICOPTER_FRONT = 0.0;
    public static double CLAWDRICOPTER_BACK = 1.0;
    public static int CLAW_OPEN_TIME = 500;
    public static int CLAW_CLOSE_TIME = 500;
    public static int ROTATE_BACK_TIME = 500;
    public static int ROTATE_FRONT_TIME = 500;

    public LiftPosition liftTargetHeight = LiftPosition.STOWED;


    // Robot Controller Config Strings
    public final static String RIGHT_FINGER = "RightFinger";
    public final static String LEFT_FINGER = "LeftFinger";
    public final static String CLAW = "Claw";
    public final static String CLAWDRICTOPTER = "Clawdricopter";
    public final static String LEFT_INTAKE = "LeftIntake";
    public final static String RIGHT_INTAKE = "RightIntake";
    public final static String LIFTER = "Lifter";
    public final static String EXTENDER = "Extender";

    // Hardware objects
    protected Servo rightFinger = null;
    protected Servo leftFinger = null;
    protected Servo claw = null;
    protected Servo clawdricopter = null;
    protected DcMotor leftIntake = null;
    protected DcMotor rightIntake = null;
    protected DcMotor lifter = null;
    protected DcMotor extender = null;

    /* LEDs: Use this line if you drive the LEDs using an I2C/SPI bridge. */
    private DotStarBridgedLED leds;
    private IDotStarPattern robotDisplay;
    private IDotStarPattern ftcTimer;
    private IDotStarPattern halfAndHalf;
    private List<Integer> colors;

    // Tracking variables
    private ElapsedTime stateTimer;
    private LiftActivity liftState = LiftActivity.IDLE;
    private ReleaseActivity releaseState = ReleaseActivity.IDLE;
    private StowActivity stowState = StowActivity.IDLE;
    private boolean fingersUp = true;
    private boolean clawPinched = false;
    private boolean clawdricopterBack = false;
    private boolean intakeForward = false;
    private boolean intakeReverse = false;
    // These are the heights of the stone levels to auto set the lift to
    protected LiftPosition lastLiftHeight = LiftPosition.STOWED;
    protected int liftZero = 0;

    /* Constructor */
    public HardwareOmnibot(){
        super();
    }

    public void initGroundEffects()
    {
        // Use ModernRoboticsDIM if using Modern Robotics hardware.
        leds.setController(DotStarBridgedLED.Controller.RevExpansionHub);
        leds.setLength(60);
        colors = new ArrayList<>();
        colors.add(0, 0x0);
        colors.add(1, 0x0);
        halfAndHalf = new DSPatternHalfAndHalf(leds);
        halfAndHalf.setPatternColors(colors);
        ftcTimer = new DSPatternFtcTimer(leds);
        robotDisplay = halfAndHalf;
    }

    public void updateTimerGroundEffects() {
        robotDisplay.update();
    }

    public void startTimerGroundEffects() {
        robotDisplay = ftcTimer;
        robotDisplay.update();
    }

    public void stopGroundEffects() {
    }

    public void toggleFingers() {
        if(fingersUp) {
            rightFinger.setPosition(RIGHT_FINGER_DOWN);
            leftFinger.setPosition(LEFT_FINGER_DOWN);
            fingersUp = false;
        } else {
            rightFinger.setPosition(RIGHT_FINGER_UP);
            leftFinger.setPosition(LEFT_FINGER_UP);
            fingersUp = true;
        }
    }

    public void startLifting() {
        if(liftState == LiftActivity.IDLE) {
            if(releaseState != ReleaseActivity.IDLE) {
                releaseState = ReleaseActivity.STOPPING;
            }
            if(stowState != StowActivity.IDLE) {
                stowState = StowActivity.STOPPING;
            }
        }
    }

    public void performLifting() {
        if(liftState != LiftActivity.IDLE) {

        }
    }

    public void startReleasing() {
        if(releaseState == ReleaseActivity.IDLE) {
            if(liftState != LiftActivity.IDLE) {
                liftState = LiftActivity.STOPPING;
            }
            if(stowState != StowActivity.IDLE) {
                stowState = StowActivity.STOPPING;
            }
            releaseState = ReleaseActivity.RELEASE_STONE;
            claw.setPosition(CLAW_OPEN);
            stateTimer.reset();

        }
    }

    public void performReleasing() {
        switch(releaseState)
        {
            case IDLE:
                break;
            case STOPPING:
                // I don't think we can do anything here. The servo going
                // either way is about the same.  Maybe when we implement
                // the ALIGN_TO_FOUNDATION
                releaseState = ReleaseActivity.IDLE;
                break;
            case ALIGN_TO_FOUNDATION:
                // In the future this should use distance sensors to line
                // up to the foundation.
                releaseState = ReleaseActivity.RELEASE_STONE;
                break;
            case RELEASE_STONE:
                if(stateTimer.milliseconds() >= CLAW_OPEN_TIME)
                {
                    releaseState = ReleaseActivity.IDLE;
                }
                break;
        }
    }

    public void startStowing() {
        if(stowState == StowActivity.IDLE) {
            if(liftState != LiftActivity.IDLE) {
                liftState = LiftActivity.STOPPING;
            }
            if(releaseState != ReleaseActivity.IDLE) {
                releaseState = ReleaseActivity.STOPPING;
            }
        }
    }

    public void performStowing() {
        if(stowState != StowActivity.IDLE) {

        }
    }

    public void addStone() {
        liftTargetHeight = LiftPosition.addStone(liftTargetHeight);
    }

    public void removeStone() {
        liftTargetHeight = LiftPosition.removeStone(liftTargetHeight);
    }

    public void toggleClaw() {
        if(clawPinched) {
            claw.setPosition(CLAW_OPEN);
            clawPinched = false;
        } else {
            claw.setPosition(CLAW_PINCHED);
            clawPinched = true;
        }
    }

    public void rotateClawdricopter() {
        if(clawdricopterBack) {
            clawdricopter.setPosition(CLAWDRICOPTER_FRONT);
            clawdricopterBack = false;
        } else {
            clawdricopter.setPosition(CLAWDRICOPTER_BACK);
            clawdricopterBack = true;
        }
    }

    public void manualExtendIntake(double power) {
        extender.setPower(power);
    }

    public void manualLift(double power) {
        lifter.setPower(power);
    }

    public void extendIntake() {
        extender.setTargetPosition(ExtendPosition.SPINMIN.getEncoderCount());
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setPower(1.0);
    }

    public void retractIntake() {
        extender.setTargetPosition(ExtendPosition.RETRACTED.getEncoderCount());
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setPower(1.0);
    }

    public void runLift() {
        lifter.setTargetPosition(liftTargetHeight.getEncoderCount());
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.5);
    }

    public void toggleIntake(boolean reverse) {
        if(reverse) {
            if(intakeReverse) {
                stopIntake();
            } else {
                if (intakeForward) {
                    stopIntake();
                }
                leftIntake.setPower(-1.0);
                rightIntake.setPower(-1.0);
                intakeReverse = true;
            }
        } else {
            if(intakeForward) {
                stopIntake();
            } else {
                if (intakeReverse) {
                    stopIntake();
                }
                leftIntake.setPower(1.0);
                rightIntake.setPower(1.0);
                intakeForward = true;
            }
        }
    }

    public void stopIntake() {
        if(intakeForward || intakeReverse) {
            intakeForward = false;
            intakeReverse = false;
            leftIntake.setPower(0.0);
            rightIntake.setPower(0.0);
        }
    }

    public void setLiftZero(int value) {
        liftZero = value;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        super.init(ahwMap);

        stateTimer = new ElapsedTime();
        rightFinger = hwMap.get(Servo.class, RIGHT_FINGER);
        leftFinger = hwMap.get(Servo.class, LEFT_FINGER);
        claw = hwMap.get(Servo.class, CLAW);
        clawdricopter = hwMap.get(Servo.class, CLAWDRICTOPTER);
        leftIntake = hwMap.get(DcMotor.class, LEFT_INTAKE);
        rightIntake = hwMap.get(DcMotor.class, RIGHT_INTAKE);
        lifter = hwMap.get(DcMotor.class, LIFTER);
        extender = hwMap.get(DcMotor.class, EXTENDER);

        // Set motor rotation
        // This makes lift go up with positive encoder values and power
        lifter.setDirection(DcMotor.Direction.REVERSE);
        // This makes extender go out with positive encoder values and power
        extender.setDirection(DcMotor.Direction.FORWARD);
        // This makes intake pull in with positive encoder values and power
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        // Set motor encoder usage
        // Lifter encoder allows us to go to specific heights.
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Extender encoder allows us to extend to specific positions.
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // The intake should just run as fast as it can.
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset the encoders that are used.
        int sleepTime = 0;
        int encoderCount = lifter.getCurrentPosition();

        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) { break; }
            sleepTime += 10;
            encoderCount = lifter.getCurrentPosition();
        }

        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set the stop mode
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the LEDs. Change this to your configured name.
        leds = hwMap.get(DotStarBridgedLED.class, "leds");

        initGroundEffects();
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    /*
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    */
}

