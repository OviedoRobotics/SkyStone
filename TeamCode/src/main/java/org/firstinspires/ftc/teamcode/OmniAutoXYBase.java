package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.WayPoint;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;
import org.firstinspires.ftc.teamcode.RobotUtilities.MovementVars;

/**
 * Created by 12090 STEM Punk
 */
public abstract class OmniAutoXYBase extends LinearOpMode {

    protected ElapsedTime timer;

    public static float mmPerInch = OmniAutoXYBase.MM_PER_INCH;
    public static float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    public static float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
    protected boolean skipThis = false;
    protected boolean integrated = false;

    // The curve points we are going to use to do the whole auto.  Set in the alliance
    // specific autonomous.
    // This is the field location the bot starts at.
    protected WayPoint startLocation;
    // This is if we want to pull the robot away from the wall before maneuvering
    protected WayPoint distanceFromWall;

    // This is the spot to align with the stone.
    protected WayPoint positionToGrabSkystone1;
    // This is the spot to grab the stone.
    protected WayPoint grabSkystone1;
    // This is where we want to pull back to to run with the stone.
    protected WayPoint pullBackSkystone1;

    // This might go away, but is intended to jog to run lane from pulling the foundation and not
    // hit partner.
    protected WayPoint buildSiteDodgingPartner;
    // This is under the bridge on the building side and should be far enough in to be a delivery
    protected WayPoint buildSiteUnderBridge;
    // This gets near the foundation starting position so we can grab it slowly
    protected WayPoint snuggleFoundation;
    // This drives into the foundation for grabbing
    protected WayPoint grabFoundation;
    // This pulls the foundation out from the starting position
    protected WayPoint pullFoundation;
    // This pushes the foundation back against the wall.  Only time it is used is to set the
    // foundation angle.
    protected WayPoint pushFoundation;
    // This is where the robot waits for the lift to go down to go under the bridge, should be
    // extending intake over line to park.
    protected WayPoint buildSiteReadyToRun;

    // This is at the top of the skystone line on the quarry side.
    protected WayPoint quarryUnderBridge;

    // This is the position to line up with the second skystone
    protected WayPoint positionToGrabSkystone2;
    // This is moving forward to grab the skystone.
    protected WayPoint grabSkystone2;
    // This is pulling back with the skystone to run.
    protected WayPoint pullBackSkystone2;

    // This is the location to put stones on the foundation
    protected WayPoint foundationDeposit;
    // This might go away if we can get buildSiteReadyToRun working.
    protected WayPoint park;

    // This is the position to line up with the first stone that isn't a skystone.
    protected WayPoint positionToGrabMundanestone1;
    // This is moving forward to grab the stone.
    protected WayPoint grabMundanestone1;
    // This is pulling back with the stone to run.
    protected WayPoint pullBackMundanestone1;

    // This is the position to line up with the second stone that isn't a skystone.
    protected WayPoint positionToGrabMundanestone2;
    // This is moving forward to grab the stone.
    protected WayPoint grabMundanestone2;
    // This is pulling back with the stone to run.
    protected WayPoint pullBackMundanestone2;

    protected ElapsedTime autoTimer = new ElapsedTime();
    protected ElapsedTime autoTaskTimer = new ElapsedTime();

    HardwareOmnibot robot = new HardwareOmnibot();

    // Default to 4" wheels
    private static double myWheelSize = 4.0;
    // Default to 40:1 motors
    private static double myMotorRatio = 19.2;

    // 20:1 motor = 560
    // 40:1 motor = 1120
    // 60:1 motor = 1680
    private static final double encoderClicksPerRev = 28;
    private static double clicksPerCm = (myMotorRatio * encoderClicksPerRev) / (Math.PI * myWheelSize * 2.54);

    public static final float MM_PER_INCH = 25.4f;

	// Have to set this when we start motions
	public double lastDriveAngle;
	public boolean liftIdle = true;

    /**
     * @param newWheelSize  - The size of the wheels, used to calculate encoder clicks per inch
     * @param newMotorRatio - The motor gearbox ratio, used to calculate encoder clicks per inch
     */
    public void setupRobotParameters(double newWheelSize, double newMotorRatio) {
        robot.init(hardwareMap);
        timer = new ElapsedTime();

        robot.resetEncoders();
        robot.setInputShaping(false);
        myWheelSize = newWheelSize;
        myMotorRatio = newMotorRatio;

        clicksPerCm = (myMotorRatio * encoderClicksPerRev) / (Math.PI * myWheelSize * 2.54);
    }

    public void collectStoneFoundation(WayPoint positionToGrabStone, WayPoint grabStone, WayPoint pullBackStone) {
        // Starting point is approaching bridge from the build plate.  buildSiteReadyToRun is
        // supposed to be close enough to score parking.
        driveToWayPointMindingLift(buildSiteReadyToRun);
        // Make sure the lift is down before going under bridge
        while (robot.stackStone != HardwareOmnibot.StackActivities.IDLE && opModeIsActive()) {
            updatePosition();
        }

        // Go under the bridge
        driveToWayPoint(quarryUnderBridge, true, false);

        // Start the intake spinning
        robot.startIntake(false);

        // Make sure we are at the right angle
        driveToWayPoint(positionToGrabStone, false, false);
        rotateToWayPointAngle(positionToGrabStone, false);
        driveToWayPoint(grabStone, false, false);
        driveToWayPoint(pullBackStone, true, false);

        driveToWayPoint(quarryUnderBridge, true, false);

        // Stop the intake
        robot.stopIntake();
        // Drive under the bridge with our skystone.  buildSiteUnderBridge should be far enough to
        // score delivery points.
        driveToWayPoint(buildSiteUnderBridge, true, false);

        // Start the second skystone deposit
        if (!skipThis) {
            robot.liftTargetHeight = HardwareOmnibot.LiftPosition.STONE_AUTO;
            robot.startStoneStacking();
        }
        driveToWayPoint(foundationDeposit, false, false);
        // Make sure we have released the skystone before leaving
        while ((robot.liftState != HardwareOmnibot.LiftActivity.IDLE ||
                robot.releaseState != HardwareOmnibot.ReleaseActivity.IDLE) && opModeIsActive()) {
            updatePosition();
        }
    }

    public void collectStoneDelivery(WayPoint positionToGrabStone, WayPoint grabStone, WayPoint pullBackStone) {
        // Starting point is approaching bridge from the build plate.  buildSiteReadyToRun is
        // supposed to be close enough to score parking.
        driveToWayPointMindingLift(buildSiteReadyToRun);
        // Make sure the lift is down before going under bridge
        while (robot.stackStone != HardwareOmnibot.StackActivities.IDLE && opModeIsActive()) {
            updatePosition();
        }

        // Go under the bridge
        driveToWayPoint(quarryUnderBridge, true, false);

        // Start the intake spinning
        robot.startIntake(false);

        // Make sure we are at the right angle
        driveToWayPoint(positionToGrabStone, false, false);
        rotateToWayPointAngle(positionToGrabStone, false);
        driveToWayPoint(grabStone, false, false);
        driveToWayPoint(pullBackStone, true, false);

        driveToWayPoint(quarryUnderBridge, true, false);

        // Stop the intake
        robot.stopIntake();
        // Drive under the bridge with our skystone.  buildSiteUnderBridge should be far enough to
        // score delivery points.
        driveToWayPoint(buildSiteUnderBridge, false, false);
    }

    /**
     * @param targetAngle  - The angle the robot should try to face when reaching destination.
	 * @param pullingFoundation - If we are pulling the foundation.
	 * @param resetDriveAngle - When we start a new drive, need to reset the starting drive angle.
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean rotateToAngle(double targetAngle, boolean pullingFoundation, boolean resetDriveAngle) {
		boolean reachedDestination = false;
		double errorMultiplier = pullingFoundation ? 0.022 : 0.016;
		double minSpinRate = pullingFoundation ? robot.MIN_FOUNDATION_SPIN_RATE : robot.MIN_SPIN_RATE;
		double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
		double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;

		// This should be set on the first call to start us on a new path.
        if(resetDriveAngle) {
            lastDriveAngle = deltaAngle;
        }

		// We are done if we are within 2 degrees
		if(Math.abs(Math.toDegrees(deltaAngle)) < 2) {
			// We have reached our destination if the angle is close enough
			robot.setAllDriveZero();
			reachedDestination = true;
		// We are done when we flip signs.
		} else if(lastDriveAngle < 0) {
			// We have reached our destination if the delta angle sign flips from last reading
			if(deltaAngle >= 0) {
				robot.setAllDriveZero();
				reachedDestination = true;
			} else {
				// We still have some turning to do.
                MovementVars.movement_x = 0;
		        MovementVars.movement_y = 0;
		        if(turnSpeed > -minSpinRate) {
		            turnSpeed = -minSpinRate;
                }
        		MovementVars.movement_turn = turnSpeed;
				robot.ApplyMovement();
			}
		} else {
			// We have reached our destination if the delta angle sign flips
			if(deltaAngle <= 0) {
				robot.setAllDriveZero();
				reachedDestination = true;
			} else {
				// We still have some turning to do.
                MovementVars.movement_x = 0;
		        MovementVars.movement_y = 0;
                if(turnSpeed < minSpinRate) {
                    turnSpeed = minSpinRate;
                }
        		MovementVars.movement_turn = turnSpeed;
				robot.ApplyMovement();
			}
		}
		lastDriveAngle = deltaAngle;

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
		boolean reachedDestination = false;
		double errorMultiplier = pullingFoundation ? 0.020 : 0.014;
		double minDriveMagnitude = pullingFoundation ? robot.MIN_FOUNDATION_DRIVE_MAGNITUDE : robot.MIN_DRIVE_MAGNITUDE;
        double deltaX = x - MyPosition.worldXPosition;
        double deltaY = y - MyPosition.worldYPosition;
        double driveAngle = Math.atan2(deltaY, deltaX);
		double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
        double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
		double driveSpeed;
		double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;
		// Have to convert from world angles to robot centric angles.
		double robotDriveAngle = driveAngle - MyPosition.worldAngle_rad + Math.toRadians(90);
		double error = 2;

		if(passThrough) {
		    error = 5;
        }

		// This will allow us to do multi-point routes without huge slowdowns.
		// Such use cases will be changing angles, or triggering activities at
		// certain points.
		if(!passThrough) {
            driveSpeed = magnitude * errorMultiplier;
		} else {
			driveSpeed = maxSpeed;
		}

		// Check if we passed through our point
        if(magnitude <= error) {
			reachedDestination = true;
            if(!passThrough) {
				robot.setAllDriveZero();
			}		
		} else {
		    if(driveSpeed < minDriveMagnitude) {
		        driveSpeed = minDriveMagnitude;
            }
            MovementVars.movement_x = driveSpeed * Math.cos(robotDriveAngle);
            MovementVars.movement_y = driveSpeed * Math.sin(robotDriveAngle);
            MovementVars.movement_turn = turnSpeed;
		    robot.ApplyMovement();
        }

        return reachedDestination;
    }

    protected void updatePosition() {
        // Allow the robot to read sensors again
        robot.resetReads();
        MyPosition.giveMePositions(robot.getLeftEncoderWheelPosition(),
                robot.getRightEncoderWheelPosition(),
                robot.getStrafeEncoderWheelPosition());
        telemetry.addData("WorldX: ", MyPosition.worldXPosition);
        telemetry.addData("WorldY: ", MyPosition.worldYPosition);
        telemetry.addData("WorldAngle: ", Math.toDegrees(MyPosition.worldAngle_rad));
        telemetry.update();

        // Progress the robot actions.
        performRobotActions();
    }

    protected void performRobotActions() {
        robot.performExtendingIntake();
        robot.performStowing();
        robot.performLifting();
        robot.performReleasing();
        robot.performStoneStacking();
        liftIdle = robot.stackStone == HardwareOmnibot.StackActivities.IDLE;
    }

    protected void driveToWayPoint(WayPoint destination, boolean passThrough, boolean pullingFoundation) {
        // Loop until we get to destination.
        updatePosition();
        while(!driveToXY(destination.x, destination.y, destination.angle,
                destination.speed, passThrough, pullingFoundation)
                && opModeIsActive()) {
            updatePosition();
        }
    }

    // This is a special case where we want to pass through a point if we get
    // the lift down in time.
    protected void driveToWayPointMindingLift(WayPoint destination) {
        // Loop until we get to destination.
        updatePosition();
        // When the lift is idle (true) we want pass through to be true
        // When the lift is not idle (false) we want pass through to be false.
        while(!driveToXY(destination.x, destination.y, destination.angle,
                destination.speed, liftIdle, false)
                && opModeIsActive()) {
            updatePosition();
        }
    }

    protected void rotateToWayPointAngle(WayPoint destination, boolean pullingFoundation) {
        // Move the robot away from the wall.
        updatePosition();
        rotateToAngle(destination.angle, pullingFoundation, true);
        // Loop until we get to destination.
        updatePosition();
        while(!rotateToAngle(destination.angle, pullingFoundation, false) && opModeIsActive()) {
            updatePosition();
        }
    }
    /**
     * @param position    - The current encoder position
     * @param destination - The desired encoder position
     * @param speed       - The speed of travel used to get direction
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean reachedClickPosition(int position, int destination, double speed, boolean reverseEncoders) {
        boolean result = false;

        if (reverseEncoders) {
            if (speed < 0) {
                if (position >= destination) {
                    result = true;
                }
            } else {
                if (position <= destination) {
                    result = true;
                }
            }
        } else {
            if (speed > 0) {
                if (position >= destination) {
                    result = true;
                }
            } else {
                if (position <= destination) {
                    result = true;
                }
            }
        }
        return result;
    }

    /**
     * @param speed        - The driving power
     * @param rotateSpeed  - The rotational speed to correct heading errors
     * @param driveAngle   - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeading(double speed, double rotateSpeed, double driveAngle, double headingAngle) {
        double xPower = 0.0;
        double yPower = 0.0;
        double deltaAngle = 0.0;
        final double SAME_ANGLE = 1;
        double gyroReading = robot.readIMU();
        deltaAngle = deltaAngle(headingAngle, gyroReading);

        if (Math.abs(deltaAngle) > SAME_ANGLE) {
            if (deltaAngle > 0.0) {
                rotateSpeed = -rotateSpeed;
            }
        } else {
            rotateSpeed = 0.0;
        }

        xPower = speed * Math.cos(Math.toRadians(driveAngle));
        yPower = speed * Math.sin(Math.toRadians(driveAngle));
        robot.drive(xPower, yPower, rotateSpeed, 0.0, false);
    }

    /**
     * @param destinationAngle - The target angle to reach, between 0.0 and 360.0
     * @param gyroReading      - The current angle of the robot
     * @return The minumum angle to travel to get to the destination angle
     */
    private double deltaAngle(double destinationAngle, double gyroReading) {
        double result = 0.0;
        double leftResult = 0.0;
        double rightResult = 0.0;

        if (gyroReading > destinationAngle) {
            leftResult = gyroReading - destinationAngle;
            rightResult = 360.0 - gyroReading + destinationAngle;
        } else {
            leftResult = gyroReading + 360.0 - destinationAngle;
            rightResult = destinationAngle - gyroReading;
        }

        if (leftResult < rightResult) {
            result = -leftResult;
        } else {
            result = rightResult;
        }

        return result;
    }

    /**
     * @param distanceToTravelMm - How far we are traveling in mm
     * @param maxSpeed           - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledDeceleration(double distanceToTravelMm, double maxSpeed) {
        final double superFastDistance = 400.0;
        final double fastDistance = 200.0;
        final double mediumDistance = 100.0;
        final double fastDivider = 1.4;
        final double mediumDivider = 2.0;
        final double slowDivider = 3.0;

        double result = 0.0;

        if (distanceToTravelMm > superFastDistance) {
            result = maxSpeed;
        } else if (distanceToTravelMm > fastDistance) {
            result = maxSpeed / fastDivider;
        } else if (distanceToTravelMm > mediumDistance) {
            result = maxSpeed / mediumDivider;
        } else {
            result = maxSpeed / slowDivider;
        }

        return result;
    }

    /**
     * @param distanceToTravelMm - How far we are traveling in mm
     * @param maxSpeed           - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledRotationMm(double distanceToTravelMm, double maxSpeed) {
        final double fastDistance = 75.0;
        final double mediumDistance = 50.0;
        final double mediumDivider = 2.0;
        final double slowDivider = 4.0;

        double result = 0.0;

        if (distanceToTravelMm > fastDistance) {
            result = maxSpeed;
        } else if (distanceToTravelMm > mediumDistance) {
            result = maxSpeed / mediumDivider;
        } else {
            result = maxSpeed / slowDivider;
        }

        return result;
    }

    /**
     * @param angleToTravel - How far we are traveling in degrees
     * @param maxSpeed      - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledRotationAngle(double angleToTravel, double maxSpeed) {
        final double fastAngle = 30.0;
        final double mediumAngle = 15.0;
        final double mediumDivider = 1.5;
        final double slowDivider = 3.0;
        double angleToTravelAbs = Math.abs(angleToTravel);

        double result = 0.0;

        if (angleToTravelAbs > fastAngle) {
            result = maxSpeed;
        } else if (angleToTravelAbs > mediumAngle) {
            result = maxSpeed / mediumDivider;
        } else {
            result = maxSpeed / slowDivider;
        }

        return result;
    }
}