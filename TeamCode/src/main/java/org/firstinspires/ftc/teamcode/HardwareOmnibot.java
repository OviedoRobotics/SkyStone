package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HelperClasses.WayPoint;

import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by 12090 STEM Punk
 */
public class HardwareOmnibot extends HardwareOmnibotDrive
{
    public enum StackAlignActivity {
        IDLE,
        GOTO_POSITION,
        GOTO_ANGLE
    }
    public enum ExtendIntakeActivities {
        IDLE,
        GOTO_LIMIT_SWITCH,
        MAX_EXTEND
    }

    public enum StackActivities {
        IDLE,
        LIFT,
        RELEASE,
        STOW
    }

    public enum LiftActivity {
        IDLE,
		LOWERING_TO_GRAB,
        GRABBING_STONE,
        CLEARING_LIFT,
        LIFTING_TO_ROTATE,
        LIFTING_TO_STONE,
        ROTATING,
        LOWERING_TO_STONE
    }

    public enum ReleaseActivity {
        IDLE,
        LOWER_TO_RELEASE
    }

    public enum StowActivity {
        IDLE,
        RAISING_TO_ROTATE,
        ROTATING,
        CLEARING_LIFT,
        LOWERING_TO_STOW,
		OPENING_CLAW
    }

    public enum EjectActivity {
        IDLE,
        EJECT
    }

    public enum CapstoneActivity {
        IDLE,
        CLEARING_LIFT,
        LIFTING_TO_GRAB,
        ROTATING_TO_GRAB,
        LOWERING_TO_GRAB,
        GRABBING_CAPSTONE,
        LIFTING_CAPSTONE,
        ROTATING_TO_STONE,
        LOWERING_TO_RELEASE,
        RELEASING_CAPSTONE,
        LOWERING_TO_STONE
    }

    public static int MAX_LIFT = 2800;
    public enum LiftPosition {
		GRABBING(5),
        STOWED(25),
        AUTO_OVERCOMP(50),
        STONE1_RELEASE(108),
        STONE1(176),
        STONE1_ROTATE(408),
        // Try and make stone auto maneuvers faster.
//        STONE_AUTO_RELEASE(226),
//        STONE_AUTO(226),
//        STONE_AUTO_ROTATE(226),
        STONE_AUTO_RELEASE(300),
        STONE_AUTO(300),
        STONE_AUTO_ROTATE(300),
        CAPSTONE_GRAB(360),
        STONE2_RELEASE(343),
        STONE2(474),
        STONE2_ROTATE(643),
        CAPSTONE_ROTATE(650),
        STONE3_RELEASE(579),
        STONE3(672),
        STONE3_ROTATE(879),
        ROTATE(790),
        STONE4_RELEASE(811),
        STONE4(907),
        STONE4_ROTATE(1111),
        STONE5_RELEASE(1081),
        STONE5(1171),
        STONE5_ROTATE(1381),
        STONE6_RELEASE(1313),
        STONE6(1407),
        STONE6_ROTATE(1613),
        STONE7_RELEASE(1545),
        STONE7(1653),
        STONE7_ROTATE(1845),
        STONE8_RELEASE(1794),
        STONE8(1903),
        STONE8_ROTATE(2104),
        STONE9_RELEASE(2031),
        STONE9(2140),
        STONE9_ROTATE(2331),
        STONE10_RELEASE(2247),
        STONE10(2351),
        STONE10_ROTATE(2547),
        STONE11_RELEASE(2472),
        STONE11(2575),
        STONE11_ROTATE(2772),
        STONE12_RELEASE(2690),
        STONE12(2780),
        STONE12_ROTATE(2870),
        LIFTMAX(MAX_LIFT);

        private final int encoderCount;

        LiftPosition(int encoderCount)
		{
			this.encoderCount = encoderCount;
		}

        public int getEncoderCount()
		{
			return encoderCount;
		}

		public static LiftPosition releasePosition(LiftPosition currentStone)
        {
            switch(currentStone)
            {
                case STONE1:
                    return STONE1_RELEASE;
                case STONE_AUTO:
                    return STONE_AUTO_RELEASE;
                case STONE2:
                    return STONE2_RELEASE;
                case STONE3:
                    return STONE3_RELEASE;
                case STONE4:
                    return STONE4_RELEASE;
                case STONE5:
                    return STONE5_RELEASE;
                case STONE6:
                    return STONE6_RELEASE;
                case STONE7:
                    return STONE7_RELEASE;
                case STONE8:
                    return STONE8_RELEASE;
                case STONE9:
                    return STONE9_RELEASE;
                case STONE10:
                    return STONE10_RELEASE;
                case STONE11:
                    return STONE11_RELEASE;
                case STONE12:
                    return STONE12_RELEASE;
                default:
                    return currentStone;
            }
        }

        public static LiftPosition rotatePosition(LiftPosition currentStone)
        {
            switch(currentStone)
            {
                case ROTATE:
                    return ROTATE;
                case STONE1:
                    return STONE1_ROTATE;
                case STONE_AUTO:
                    return STONE_AUTO_ROTATE;
                case STONE2:
                    return STONE2_ROTATE;
                case STONE3:
                    return STONE3_ROTATE;
                case STONE4:
                    return STONE4_ROTATE;
                case STONE5:
                    return STONE5_ROTATE;
                case STONE6:
                    return STONE6_ROTATE;
                case STONE7:
                    return STONE7_ROTATE;
                case STONE8:
                    return STONE8_ROTATE;
                case STONE9:
                    return STONE9_ROTATE;
                case STONE10:
                    return STONE10_ROTATE;
                case STONE11:
                    return STONE11_ROTATE;
                case STONE12:
                    return STONE12_ROTATE;
                default:
                    return currentStone;
            }
        }

		public static LiftPosition addStone(LiftPosition currentStone)
		{
			switch(currentStone)
			{
				case STONE1:
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
				default:
					return currentStone;
			}
		}

		public static LiftPosition removeStone(LiftPosition currentStone)
		{
			switch(currentStone)
			{
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
    public final static double ACCEL_STEP = 0.007;
    public static double INTAKE_SPEED = 1.0;
    public static double OUTAKE_SPEED = -0.50;
	public static double LIFT_MAX_SPEED = 1.0;
	public static double LIFT_MID_SPEED = 0.4;
	public static double LIFT_MIN_SPEED = 0.2;
    public static double RIGHT_FINGER_UP = 0.20;
    public static double LEFT_FINGER_UP = 0.90;
    public static double RIGHT_FINGER_DOWN = 0.70;
    public static double LEFT_FINGER_DOWN = 0.40;
//    public static double RIGHT_FINGER_DOWN = 0.31;
//    public static double LEFT_FINGER_DOWN = 0.83;
//    public static double RIGHT_FINGER_UP = 0.65;
//    public static double LEFT_FINGER_UP = 0.44;
    public static double CLAW_OPEN = 0.22;
    public static double CLAW_PINCHED = 0.95;
    public static double CLAW_CAPSTONE = 1.0;
//    public static double CLAW_OPEN = 1.0;
//    public static double CLAW_PINCHED = 0.21;
//    public static double CLAW_CAPSTONE = 0.20;
    public static double CLAWDRICOPTER_FRONT = 0.865;
    public static double CLAWDRICOPTER_CAPSTONE = 0.707;
    public static double CLAWDRICOPTER_BACK = 0.12;
    public static int CLAW_OPEN_TIME = 500;
    public static int CLAW_CLOSE_TIME = 800;
    public static int CLAW_ROTATE_BACK_TIME = 1000;
    public static int CLAW_ROTATE_CAPSTONE_TIME = 500;
    public static int CLAW_ROTATE_FRONT_TIME = 1000;
    public static int MAX_EXTEND_TIME = 300;
    public static int EJECT_EXTEND_TIME = 700;
    public static int FINGER_ROTATE_TIME = 300;
	private static int ENCODER_ERROR = 15;

	// The OpMode set target height for the lift to go.
    public LiftPosition liftTargetHeight = LiftPosition.STONE1;
    // The height the activity was activated to achieve
	public LiftPosition liftActivityTargetHeight = LiftPosition.STONE1;

    // Robot Controller Config Strings
    public final static String RIGHT_FINGER = "RightFinger";
    public final static String LEFT_FINGER = "LeftFinger";
    public final static String CLAW = "Claw";
    public final static String CLAWDRICTOPTER = "Clawdricopter";
    public final static String LIFTER = "Lifter";
    public final static String INTAKE_LIMIT = "IntakeLimit";
    public final static String STONE_DETECTOR = "StoneDetector";

    // Hardware objects
    protected Servo rightFinger = null;
    protected Servo leftFinger = null;
    protected Servo claw = null;
    protected Servo clawdricopter = null;
    protected DcMotorEx lifter = null;
    protected DigitalChannel intakeLimit = null;
    protected AnalogInput stoneDetector = null;

    /* LEDs: Use this line if you drive the LEDs using an I2C/SPI bridge. */
    private DotStarBridgedLED leds;
    private IDotStarPattern robotDisplay;
    private IDotStarPattern ftcTimer;
    private IDotStarPattern halfAndHalf;
    private List<Integer> colors;

    // Tracking variables
    private ElapsedTime clawTimer;
    private ElapsedTime fingerTimer;
    private ElapsedTime settleTimer;
    private ElapsedTime clawdricopterTimer;
    private ElapsedTime intakeExtendTimer;
    public StackAlignActivity stackAlignmentState = StackAlignActivity.IDLE;
    public LiftActivity liftState = LiftActivity.IDLE;
    public ReleaseActivity releaseState = ReleaseActivity.IDLE;
    public StowActivity stowState = StowActivity.IDLE;
    public EjectActivity ejectState = EjectActivity.IDLE;
    public CapstoneActivity capstoneState = CapstoneActivity.IDLE;
    public StackActivities stackStone = StackActivities.IDLE;
    public ExtendIntakeActivities extendState = ExtendIntakeActivities.IDLE;
    private boolean clawPinched = false;
    private boolean clawdricopterBack = false;
    protected boolean stowingLift = false;
    protected double intakePower = 0.0;

    protected WayPoint stackPosition = new WayPoint(0, 0, 0, 0);

	// Variables so we only read encoders once per loop
	protected int lifterEncoderValue = 0;

    /* Constructor */
    public HardwareOmnibot(){
        super();
    }

	public void resetReads() {
        super.resetReads();
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

    public void fingersDown() {
        rightFinger.setPosition(RIGHT_FINGER_DOWN);
        leftFinger.setPosition(LEFT_FINGER_DOWN);
    }

    public void fingersUp() {
        rightFinger.setPosition(RIGHT_FINGER_UP);
        leftFinger.setPosition(LEFT_FINGER_UP);
    }

    public boolean startStackAligning() {
        boolean startedAligning = true;
        if(stackPosition.x == 0 && stackPosition.y == 0 && stackPosition.angle == 0) {
            startedAligning = false;
        } else {
            stackAlignmentState = StackAlignActivity.GOTO_POSITION;
            driveToXY(stackPosition.x, stackPosition.y, stackPosition.angle, MIN_DRIVE_MAGNITUDE,
                    1.0, 0.014, 2.0, false);
        }

        return startedAligning;
    }

    public void stopStackAligning() {
        stackAlignmentState = StackAlignActivity.IDLE;
    }

    public void performStackAligning() {
        switch(stackAlignmentState) {
            case GOTO_ANGLE:
                if(rotateToAngle(stackPosition.angle, false, false)) {
                    stackAlignmentState = StackAlignActivity.IDLE;
                }
                break;
            case GOTO_POSITION:
                if(driveToXY(stackPosition.x, stackPosition.y, stackPosition.angle, MIN_DRIVE_MAGNITUDE,
                    1.0, 0.014, 2.0, false)) {
                    // We have reached the position, need to rotate to angle.
                    stackAlignmentState = StackAlignActivity.GOTO_ANGLE;
                    rotateToAngle(stackPosition.angle, false, true);
                }
                break;
            case IDLE:
                break;
        }
    }

    public boolean startExtendingIntake() {
        extendState = ExtendIntakeActivities.GOTO_LIMIT_SWITCH;
        extender.setPower(1.0);

        return true;
    }

    public void performExtendingIntake() {
        switch(extendState) {
            case GOTO_LIMIT_SWITCH:
                // Go to the limit switch, meaning mostly extended.
                if(intakeExtended()) {
                    intakeExtendTimer.reset();
                    extendState = ExtendIntakeActivities.MAX_EXTEND;
                }
                break;
            case MAX_EXTEND:
                if(intakeExtendTimer.milliseconds() >= MAX_EXTEND_TIME) {
                    extender.setPower(0.0);
                    extendState = ExtendIntakeActivities.IDLE;
                }
                break;
            case IDLE:
                break;
        }
    }

    public boolean startCapstone() {
        boolean isCapping;
        if(capstoneState == CapstoneActivity.IDLE) {
            if((releaseState != ReleaseActivity.IDLE) || (stowState != StowActivity.IDLE) ||
                    (liftState != LiftActivity.IDLE)) {
                isCapping = false;
            } else {
                isCapping = true;
                // Extend the intake to make sure it isn't in the way.
                if(!intakeExtended()) {
                    startExtendingIntake();
                }
                // We don't want the intake spinning while we are trying to lift the stone.
                stopIntake();
                capstoneState = CapstoneActivity.CLEARING_LIFT;
            }
        } else {
            isCapping = true;
        }

        return isCapping;
    }

    public void performCapstone() {
        switch(capstoneState) {
            case LOWERING_TO_STONE:
                if(lifterAtPosition(LiftPosition.STOWED)) {
                    capstoneState = CapstoneActivity.IDLE;
                }
                break;
            case RELEASING_CAPSTONE:
                if(clawTimer.milliseconds() >= CLAW_OPEN_TIME) {
                    capstoneState = CapstoneActivity.LOWERING_TO_STONE;
                    moveLift(LiftPosition.STOWED);
                }
                break;
            case LOWERING_TO_RELEASE:
                if(lifterAtPosition(LiftPosition.STONE1_RELEASE)) {
                    capstoneState = CapstoneActivity.RELEASING_CAPSTONE;
                    claw.setPosition(CLAW_OPEN);
                    clawPinched = false;
                    clawTimer.reset();
                }
                break;
            case ROTATING_TO_STONE:
                if(clawdricopterTimer.milliseconds() >= CLAW_ROTATE_CAPSTONE_TIME) {
                    capstoneState = CapstoneActivity.LOWERING_TO_RELEASE;
                    moveLift(LiftPosition.releasePosition(LiftPosition.STONE1_RELEASE));
                }
                break;
            case LIFTING_CAPSTONE:
                if(lifterAtPosition(LiftPosition.CAPSTONE_ROTATE)) {
                    capstoneState = CapstoneActivity.ROTATING_TO_STONE;
                    clawdricopter.setPosition(CLAWDRICOPTER_FRONT);
                    clawdricopterTimer.reset();
                }
                break;
            case GRABBING_CAPSTONE:
                if(clawTimer.milliseconds() >= CLAW_CLOSE_TIME) {
                    clawPinched = true;
                    capstoneState = CapstoneActivity.LIFTING_CAPSTONE;
                    moveLift(LiftPosition.releasePosition(LiftPosition.CAPSTONE_ROTATE));
                }
                break;
            case LOWERING_TO_GRAB:
                if(lifterAtPosition(LiftPosition.CAPSTONE_GRAB)) {
                    capstoneState = CapstoneActivity.GRABBING_CAPSTONE;
                    claw.setPosition(CLAW_CAPSTONE);
                    clawTimer.reset();
                }
                break;
            case ROTATING_TO_GRAB:
                if(clawdricopterTimer.milliseconds() >= CLAW_ROTATE_CAPSTONE_TIME) {
                    capstoneState = CapstoneActivity.LOWERING_TO_GRAB;
                    moveLift(LiftPosition.releasePosition(LiftPosition.CAPSTONE_GRAB));
                }
                break;
            case LIFTING_TO_GRAB:
                if(lifterAtPosition(LiftPosition.CAPSTONE_ROTATE)) {
                    capstoneState = CapstoneActivity.ROTATING_TO_GRAB;
                    clawdricopter.setPosition(CLAWDRICOPTER_CAPSTONE);
                    clawdricopterBack = false;
                    clawdricopterTimer.reset();
                }
                break;
            case CLEARING_LIFT:
                if(intakeExtended()) {
                    capstoneState = CapstoneActivity.LIFTING_TO_GRAB;
                    moveLift(LiftPosition.CAPSTONE_ROTATE);
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    public boolean startLifting() {
         boolean isLifting;
         if(liftState == LiftActivity.IDLE) {
            if((releaseState != ReleaseActivity.IDLE) || (stowState != StowActivity.IDLE)) {
                isLifting = false;
            } else {
                isLifting = true;
                // Extend the intake to make sure it isn't in the way.
                if(!intakeExtended()) {
                    startExtendingIntake();
                }

                // We don't want the intake spinning while we are trying to lift the stone.
                stopIntake();
                liftState = LiftActivity.CLEARING_LIFT;
            }
        } else {
            isLifting = true;
        }

        return isLifting;
    }

    public void performLifting() {
		switch(liftState) {
            case LOWERING_TO_STONE:
                if(lifterAtPosition(liftActivityTargetHeight)) {
                    liftState = LiftActivity.IDLE;
                }
                break;
		    case ROTATING:
			    if(clawdricopterTimer.milliseconds() >= CLAW_ROTATE_BACK_TIME) {
                    if (liftActivityTargetHeight.getEncoderCount() <= LiftPosition.ROTATE.getEncoderCount()) {
                        liftState = LiftActivity.LOWERING_TO_STONE;
                        moveLift(liftActivityTargetHeight);
                    } else {
                        liftState = LiftActivity.IDLE;
                    }
					clawdricopterBack = true;
				}
			    break;
            case LIFTING_TO_STONE:
                if(lifterAtPosition(liftActivityTargetHeight)) {
                    liftState = LiftActivity.ROTATING;
                    clawdricopter.setPosition(CLAWDRICOPTER_BACK);
                    clawdricopterTimer.reset();
                }
                break;
		    case LIFTING_TO_ROTATE:
			    // It has gotten high enough
			    if(lifterAbovePosition(LiftPosition.ROTATE)) {
					liftState = LiftActivity.ROTATING;
					clawdricopter.setPosition(CLAWDRICOPTER_BACK);
					clawdricopterTimer.reset();
				}
			    break;
		    case GRABBING_STONE:
                if((clawTimer.milliseconds() >= CLAW_CLOSE_TIME) || clawPinched)
                {
					clawPinched = true;
					liftActivityTargetHeight = liftTargetHeight;
                    // If our target height is less than rotation height, we have to
                    // stop at rotation height first.
                    if(liftActivityTargetHeight.getEncoderCount() <= LiftPosition.ROTATE.getEncoderCount()) {
                        liftState = LiftActivity.LIFTING_TO_ROTATE;
                        moveLift(LiftPosition.ROTATE);
                    } else {
                        liftState = LiftActivity.LIFTING_TO_STONE;
                        moveLift(liftActivityTargetHeight);
                    }
                }
			    break;
		    case LOWERING_TO_GRAB:
			    // Has it gotten low enough
			    if(lifterAtPosition(LiftPosition.GRABBING)) {
					// Start grabbing the stone.  updateLifting will take over.
					liftState = LiftActivity.GRABBING_STONE;
					claw.setPosition(CLAW_PINCHED);
					clawTimer.reset();
				}
			    break;
            case CLEARING_LIFT:
                if(intakeExtended()) {
                    liftState = LiftActivity.LOWERING_TO_GRAB;
                    moveLift(LiftPosition.GRABBING);
                }
                break;
		    case IDLE:
			default:
				break;
		}
    }

    public boolean startReleasing() {
        boolean isReleasing;
        if(releaseState == ReleaseActivity.IDLE) {
            if (liftState != LiftActivity.IDLE) {
                isReleasing = false;
            } else if (stowState != StowActivity.IDLE) {
                isReleasing = false;
            } else {
                isReleasing = true;

                moveLift(LiftPosition.releasePosition(liftActivityTargetHeight));
                releaseState = ReleaseActivity.LOWER_TO_RELEASE;
            }
        }
        else {
            isReleasing = true;
        }

        return isReleasing;
    }

    public void performReleasing() {
        switch(releaseState)
        {
            case LOWER_TO_RELEASE:
                if(lifterAtPosition(LiftPosition.releasePosition(liftActivityTargetHeight))) {
                    releaseState = ReleaseActivity.IDLE;
                }
                break;
            case IDLE:
		    default:
			    break;
        }
    }

    public boolean startStowing() {
        boolean isStowing;
        if(stowState == StowActivity.IDLE) {
            if (liftState != LiftActivity.IDLE) {
                isStowing = false;
            } else if (releaseState != ReleaseActivity.IDLE) {
                isStowing = false;
            } else {
                isStowing = true;
                stowingLift = true;

                // Extend the intake to make sure it isn't in the way.
                if(!intakeExtended()) {
                    startExtendingIntake();
                }

                // We don't want the intake spinning while we are trying to stow the
                // lift.
                stopIntake();

                // Open the claw to release stone.
                claw.setPosition(CLAW_OPEN);
                clawTimer.reset();

                // Start rotating back to the front.
                stowState = StowActivity.CLEARING_LIFT;
            }
        }
        else {
                isStowing = true;
        }
        return isStowing;
    }

    public void performStowing() {
		switch(stowState) {
			case LOWERING_TO_STOW:
			    if(lifterAtPosition(LiftPosition.STOWED)) {
					stowState = StowActivity.IDLE;
                    stowingLift = false;
				}
				break;
			case ROTATING:
			    if((clawdricopterTimer.milliseconds() >= CLAW_ROTATE_FRONT_TIME) || !clawdricopterBack) {
					stowState = StowActivity.LOWERING_TO_STOW;
					moveLift(LiftPosition.STOWED);
					clawdricopterBack = false;
				}
			    break;
			case RAISING_TO_ROTATE:
			    // It has gotten high enough
				if(clawdricopterBack) {
					if(lifterAtPosition(LiftPosition.releasePosition(liftActivityTargetHeight))) {
						stowState = StowActivity.ROTATING;
						clawdricopter.setPosition(CLAWDRICOPTER_FRONT);
						clawdricopterTimer.reset();
					}
				} else {
					stowState = StowActivity.ROTATING;
				}
			    break;
            case OPENING_CLAW:
                if((clawTimer.milliseconds() >= CLAW_OPEN_TIME) || !clawPinched)
                {
                    // This happens if we are stowing from a non-stone position
                    // like starting the match.
                    if(clawPinched) {
                        addStone();
                        clawPinched = false;
                    }
                    stowState = StowActivity.RAISING_TO_ROTATE;
                    // Capture the position of stone release so we can go back autonomously.
                    stackPosition.x = MyPosition.worldXPosition;
                    stackPosition.y = MyPosition.worldYPosition;
                    stackPosition.angle = MyPosition.worldAngle_rad;
                    if(clawdricopterBack) {
                        if (LiftPosition.rotatePosition(liftActivityTargetHeight).getEncoderCount() < LiftPosition.ROTATE.getEncoderCount()) {
                            liftActivityTargetHeight = LiftPosition.ROTATE;
                        } else {
                            liftActivityTargetHeight = LiftPosition.rotatePosition(liftActivityTargetHeight);
                        }
                        moveLift(LiftPosition.releasePosition(liftActivityTargetHeight));
                    }
                }
                break;
            case CLEARING_LIFT:
			    if(intakeExtended()) {
                    stowState = StowActivity.OPENING_CLAW;
                }
                break;
            case IDLE:
		    default:
			    break;
        }
    }

    public boolean startStoneStacking() {
        boolean isStacking;
        if (stackStone == StackActivities.IDLE) {
            isStacking = true;
            stackStone = StackActivities.LIFT;
            startLifting();
        } else {
            isStacking = true;
        }

        return isStacking;
    }

    public void performStoneStacking() {
        switch(stackStone)
        {
            case STOW:
                if(stowState == StowActivity.IDLE) {
                    stackStone = StackActivities.IDLE;
                }
                break;
            case RELEASE:
                if(releaseState == ReleaseActivity.IDLE) {
                    stackStone = StackActivities.STOW;
                    startStowing();
                }
                break;
            case LIFT:
                if(liftState == LiftActivity.IDLE) {
                    stackStone = StackActivities.RELEASE;
                    startReleasing();
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    public void startEjecting() {
        if (ejectState == EjectActivity.IDLE) {
            ejectState = EjectActivity.EJECT;
            startIntake(true);
            extender.setPower(-1.0);
            intakeExtendTimer.reset();
        }
    }

    public void performEjecting() {
        switch(ejectState)
        {
            case EJECT:
                if(intakeExtendTimer.milliseconds() > EJECT_EXTEND_TIME) {
                    // It  sucks the block back in if we don't stop it.
                    ejectState = EjectActivity.IDLE;
                    stopIntake();
                    startExtendingIntake();
                }
            case IDLE:
                break;
            default:
                break;
        }
    }

    public void addStone() {
        liftTargetHeight = LiftPosition.addStone(liftTargetHeight);
    }

    public void removeStone() {
        liftTargetHeight = LiftPosition.removeStone(liftTargetHeight);
    }

    public void adjustStoneHeight() {
        if(clawdricopterBack) {
            liftActivityTargetHeight = liftTargetHeight;
            moveLift(liftActivityTargetHeight);
        }
    }

    public void moveLift(LiftPosition targetHeight) {
        if(intakeExtended()) {
            int liftPosition = getLifterPosition();
            int targetPosition = targetHeight.getEncoderCount();
            double lifterSpeed;
            if (liftPosition < targetPosition) {
                lifterSpeed = LIFT_MAX_SPEED;
            } else {
                if(stowingLift) {
                    lifterSpeed = LIFT_MID_SPEED;
                } else {
                    lifterSpeed = LIFT_MIN_SPEED;
                }
            }
			setLifterPosition(targetPosition);
            lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter.setPower(lifterSpeed);
        }
    }

    public void startIntake(boolean reverse) {
        // Prevent the intake from starting if it isn't extended far enough.
        if (reverse) {
            if(intakePower != OUTAKE_SPEED) {
                stopIntake();
                intakePower = OUTAKE_SPEED;
                leftIntake.setPower(intakePower);
                rightIntake.setPower(intakePower);
            }
        } else {
            if(intakePower != INTAKE_SPEED) {
                stopIntake();
                intakePower = INTAKE_SPEED;
                leftIntake.setPower(intakePower);
                rightIntake.setPower(intakePower);
            }
        }
    }

    public void stopIntake() {
        if(intakePower != 0.0) {
            intakePower = 0.0;
            leftIntake.setPower(intakePower);
            rightIntake.setPower(intakePower);
        }
    }

    public boolean lifterAbovePosition(LiftPosition targetPosition) {
        return getLifterPosition() >= (targetPosition.getEncoderCount() - ENCODER_ERROR);
    }

	public boolean lifterAtPosition(LiftPosition targetPosition) {
		return Math.abs(getLifterPosition() - targetPosition.getEncoderCount()) < ENCODER_ERROR;
	}

    public boolean intakeExtended() {
        return !intakeLimit.getState();
    }

    public boolean stonePresent() {
        return stoneDetector.getVoltage() > 2.0;
    }

    public int getLifterPosition() {
        lifterEncoderValue = lifter.getCurrentPosition();

        return lifterEncoderValue;
    }

    public void setLifterPosition(int targetPosition) {
        lifter.setTargetPosition(targetPosition);
    }

    public void resetEncoders() {
        if (!encodersReset || forceReset) {
            forceReset = false;
            super.resetEncoders();
            // Reset the encoders that are used.
            int sleepTime = 0;
            int encoderCount = lifter.getCurrentPosition();

            lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            while ((encoderCount != 0) && (sleepTime < 1000)) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    break;
                }
                sleepTime += 10;
                encoderCount = lifter.getCurrentPosition();
            }
            encodersReset = true;

            lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        super.init(ahwMap);

        stoneDetector = hwMap.get(AnalogInput.class, STONE_DETECTOR);
        intakeLimit  = hwMap.get(DigitalChannel.class, INTAKE_LIMIT);
        intakeLimit.setMode(DigitalChannel.Mode.INPUT);
        clawTimer = new ElapsedTime();
        clawdricopterTimer = new ElapsedTime();
        fingerTimer = new ElapsedTime();
        settleTimer = new ElapsedTime();
        intakeExtendTimer = new ElapsedTime();
        rightFinger = hwMap.get(Servo.class, RIGHT_FINGER);
        leftFinger = hwMap.get(Servo.class, LEFT_FINGER);
        claw = hwMap.get(Servo.class, CLAW);
        clawdricopter = hwMap.get(Servo.class, CLAWDRICTOPTER);
        lifter = hwMap.get(DcMotorEx.class, LIFTER);

        // Set motor rotation
        // This makes lift go up with positive encoder values and power
        lifter.setDirection(DcMotor.Direction.FORWARD);
        // This makes extender go out with positive encoder values and power
        extender.setDirection(DcMotor.Direction.FORWARD);
        // This makes intake pull in with positive encoder values and power
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        // Set motor encoder usage
        // Lifter encoder allows us to go to specific heights.
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Without encoder makes the motor run faster, and we are using a limit
        // switch to determine position.
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The intake should just run as fast as it can.
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the stop mode
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fingersUp();

        // Set up the LEDs. Change this to your configured name.
        leds = hwMap.get(DotStarBridgedLED.class, "leds");

        initGroundEffects();
    }
}

