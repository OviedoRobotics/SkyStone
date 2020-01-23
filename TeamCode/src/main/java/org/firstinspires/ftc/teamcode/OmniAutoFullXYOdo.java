/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;
import org.firstinspires.ftc.teamcode.HelperClasses.WayPoint;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 12090 STEM Punk
 */
public abstract class OmniAutoFullXYOdo extends OmniAutoXYOdoClass
{
    // Coordinates for the vision pipeline to be overriden in the alliance classes.
    protected Point sub1PointA;
    protected Point sub1PointB;
    protected Point sub2PointA;
    protected Point sub2PointB;
    protected Point sub3PointA;
    protected Point sub3PointB;
    protected int stonePosition = 1;
    public static int position = 0;

    // The curve points we are going to use to do the whole auto.  Set in the alliance
    // specific autonomous.
    protected WayPoint startLocation;
    protected WayPoint distanceFromWall;

    protected WayPoint positionToGrabSkystone1;
    protected WayPoint grabSkystone1;
    protected WayPoint pullBackSkystone1;

    protected WayPoint quarryBeforeBridge;

    protected WayPoint buildSiteUnderBridge;
    protected WayPoint alignToFoundation;
    protected WayPoint snuggleFoundation;
    protected WayPoint grabFoundation;
    protected WayPoint pullFoundation;
    protected WayPoint pushFoundation;
    protected WayPoint buildSiteReadyToRun;

    protected WayPoint quarryUnderBridge;

    protected WayPoint positionToGrabSkystone2;
    protected WayPoint grabSkystone2;
    protected WayPoint pullBackSkystone2;

    protected WayPoint foundationDeposit;
    protected WayPoint park;
    protected ElapsedTime autoTimer = new ElapsedTime();
	protected boolean liftIdle = true;

    OpenCvCamera phoneCam;
    public abstract void setSkystoneValues(int position);
    public abstract void setVisionPoints();

    protected void updatePosition() {
        // Allow the robot to read sensors again
        robot.resetReads();
        MyPosition.giveMePositions(robot.getLeftEncoderWheelPosition(),
                robot.getRightEncoderWheelPosition(),
                robot.getStrafeEncoderWheelPosition());

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
        while(!driveToXY(destination.x, destination.y, destination.angle,
                destination.speed, !liftIdle, false)
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

    @Override
    public void runOpMode()
    {
		boolean skipThis = false;
		boolean integrated = false;

        // Setup the data needed for the vision pipeline
        setVisionPoints();

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(new OmniAutoFullXYOdo.SamplePipeline());

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        setupRobotParameters(4.0, 19.2);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
        robot.encodersReset = true;

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();
        stonePosition = position;
        telemetry.addData("Stone Position: ", stonePosition);
        telemetry.update();

		// Stop the image pipeline.
		phoneCam.stopStreaming();

		// This sets up everything for the auto to run.
		setSkystoneValues(stonePosition);

        //give MyPosition our current positions so that it saves the last positions of the wheels
        //this means we won't teleport when we start the match. Just in case, run this twice
        for(int i = 0; i < 2 ; i ++){
            robot.resetReads();
            MyPosition.initialize(robot.getLeftEncoderWheelPosition(),
                    robot.getRightEncoderWheelPosition(),
                    robot.getStrafeEncoderWheelPosition());
        }

        // Set our robot starting coordinates on the field.
        robot.resetReads();
        MyPosition.setPosition(startLocation.x, startLocation.y, startLocation.angle);

		// Start moving intake out, should be done by the time driving is done.
        robot.startExtendingIntake();
        robot.moveLift(HardwareOmnibot.LiftPosition.AUTO_OVERCOMP);

        driveToWayPoint(distanceFromWall, true, false);
        driveToWayPoint(positionToGrabSkystone1, false, false);

        // Start the intake spinning
        robot.startIntake(false);
        robot.moveLift(HardwareOmnibot.LiftPosition.STOWED);

        // Make sure we are at the right angle
        rotateToWayPointAngle(positionToGrabSkystone1, false);
        while(!robot.intakeExtended() && opModeIsActive()) {
			updatePosition();
        }

        driveToWayPoint(grabSkystone1, false, false);
        driveToWayPoint(pullBackSkystone1, true, false);

        // Stop the intake
        robot.stopIntake();

        // Help line up to go under bridge
        driveToWayPoint(quarryUnderBridge, true, false);

        // Drive under the bridge with our skystone.
        driveToWayPoint(buildSiteUnderBridge, true, false);

        // Might try here to start lift.
//        if (!skipThis) {
//            robot.liftTargetHeight = HardwareOmnibot.LiftPosition.STONE_AUTO;
//            robot.startStoneStacking();
//        }


        // Drive back to the foundation.
        driveToWayPoint(alignToFoundation, false, false);

        if (!skipThis) {
            robot.liftTargetHeight = HardwareOmnibot.LiftPosition.STONE_AUTO;
            robot.startStoneStacking();
        }

        rotateToWayPointAngle(alignToFoundation, false);

        // Drive into foundation to grab it
        driveToWayPoint(snuggleFoundation, false, false);
        driveToWayPoint(grabFoundation, true, false);
        robot.fingersDown();
        autoTimer.reset();
        while (autoTimer.milliseconds() < robot.FINGER_ROTATE_TIME && opModeIsActive()) {
			updatePosition();
        }

        // Pull and rotate the foundation.
        driveToWayPoint(pullFoundation, true, true);
        rotateToWayPointAngle(pushFoundation, true);
        // Release the foundation and push it back into the wall.
        robot.fingersUp();
        autoTimer.reset();

        // Might want to move this to stone 2.
        driveToWayPoint(pushFoundation, false, true);
        while (autoTimer.milliseconds() < robot.FINGER_ROTATE_TIME && opModeIsActive()) {
			updatePosition();
        }

        // Drive back to collect second skystone, drive through point if the lift is down in time.
		driveToWayPointMindingLift(buildSiteReadyToRun);
//        driveToWayPoint(buildSiteReadyToRun, false, false);
        // Make sure the lift is down before going under bridge
        while (robot.stackStone != HardwareOmnibot.StackActivities.IDLE && opModeIsActive()) {
			updatePosition();
        }

        // Go under the bridge
        driveToWayPoint(quarryUnderBridge, true, false);

        // Start the intake spinning
        robot.startIntake(false);

        // Make sure we are at the right angle
        driveToWayPoint(positionToGrabSkystone2, false, false);
        rotateToWayPointAngle(positionToGrabSkystone2, false);
        driveToWayPoint(grabSkystone2, false, false);
        driveToWayPoint(pullBackSkystone2, true, false);

        driveToWayPoint(quarryUnderBridge, true, false);

        // Stop the intake
        robot.stopIntake();
        // Drive under the bridge with our skystone.
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
		driveToWayPointMindingLift(buildSiteReadyToRun);
//        driveToWayPoint(buildSiteReadyToRun);

        // Make sure the lift is down before going under bridge
        while (robot.stackStone != HardwareOmnibot.StackActivities.IDLE && opModeIsActive()) {
			updatePosition();
        }
        driveToWayPoint(park, false, false);
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        private Mat YCrCb = new Mat();
        private Mat Cb = new Mat();
        private Mat subMat1;
        private Mat subMat2;
        private Mat subMat3;
        private int max;
        private int avg1;
        private int avg2;
        private int avg3;
        private Point skystone = new Point();

        @Override
        public Mat processFrame(Mat input)
        {
            // Convert the image from RGB to YCrCb
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

            // Extract the Cb channel from the image
            Core.extractChannel(YCrCb, Cb, 2);

            // The the sample areas fromt the Cb channel
            subMat1 = Cb.submat(new Rect(sub1PointA, sub1PointB));
            subMat2 = Cb.submat(new Rect(sub2PointA, sub2PointB));
            subMat3 = Cb.submat(new Rect(sub3PointA, sub3PointB));

            // Average the sample areas
            avg1 = (int)Core.mean(subMat1).val[0]; // Stone4
            avg2 = (int)Core.mean(subMat2).val[0]; // Stone5
            avg3 = (int)Core.mean(subMat3).val[0]; // Stone6

            // Draw rectangles around the sample areas
            Imgproc.rectangle(input, sub1PointA, sub1PointB, new Scalar(0, 0, 255), 1);
            Imgproc.rectangle(input, sub2PointA, sub2PointB, new Scalar(0, 0, 255), 1);
            Imgproc.rectangle(input, sub3PointA, sub3PointB, new Scalar(0, 0, 255), 1);

            // Figure out which sample zone had the lowest contrast from blue (lightest color)
            max = Math.max(avg1, Math.max(avg2, avg3));

            // Draw a circle on the detected skystone
            if(max == avg1) {
                skystone.x = (sub1PointA.x + sub1PointB.x) / 2;
                skystone.y = (sub1PointA.y + sub1PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                position = 1;
            } else if(max == avg2) {
                skystone.x = (sub2PointA.x + sub2PointB.x) / 2;
                skystone.y = (sub2PointA.y + sub2PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                position = 2;
            } else if(max == avg3) {
                skystone.x = (sub3PointA.x + sub3PointB.x) / 2;
                skystone.y = (sub3PointA.y + sub3PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                position = 3;
            } else {
                position = 1;
            }

            // Free the allocated submat memory
            subMat1.release();
            subMat1 = null;
            subMat2.release();
            subMat2 = null;
            subMat3.release();
            subMat3 = null;

            return input;
        }
    }
}