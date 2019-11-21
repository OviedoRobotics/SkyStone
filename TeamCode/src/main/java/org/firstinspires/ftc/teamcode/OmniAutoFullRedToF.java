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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="Auto: AutoFullRedTof", group ="Auto")
public class OmniAutoFullRed extends OmniAutoClass
{
    OpenCvCamera phoneCam;
    public static int position = 0;

    @Override
    public void runOpMode()
    {
        int timeout = 0;
		double leftDistance;
		double attackAngle;
		double maxSpeed = 1.0;
		double rotateSpeed = 0.3;
		int stonePosition = 1;
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(new OmniAutoFullRed.SamplePipeline());

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

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();
        stonePosition = position;
		robot.stackFromSide = HardwareOmnibot.AlignmentSide.LEFT;

		// Stop the image pipeline.
		phoneCam.stopStreaming();

        // Start moving intake out, should be done by the time driving is done.
        robot.moveLift(HardwareOmnibot.LiftPosition.STOWED);
        robot.moveIntake(HardwareOmnibot.IntakePosition.EXTENDED);

		// Drive out from the starting position, 10cm from the skystones
		distanceFromWall(HardwareOmnibot.AlignmentSide.BACK, 110.0, maxSpeed, 1.0, 5000);

		switch(stonePosition) {
            case 1:
			attackAngle = 45.0;
			leftDistance = 30.3;
			break;
			case 2:
			attackAngle = 45.0;
			leftDistance = 50.6;
			case 3:
			attackAngle = 45.0;
			leftDistance = 71.0;
			break;
		}

		// Drive from the side wall to the collection identified stone position.
		distanceFromWall(HardwareOmnibot.AlignmentSide.LEFT, leftDistance, maxSpeed, 1.0, 5000);

		// Rotate the robot to collection angle.
        rotateRobotToAngle(rotateSpeed, attackAngle, 2000);

        // Make sure the intake is out.
		double endTime = timer.milliseconds() + 1000;
        while(!robot.intakeAtPosition(HardwareOmnibot.IntakePosition.EXTENDED) && endTime < timer.milliseconds()) {
            robot.resetReads();
        }

        // Start the intake to collect.
        robot.startIntake(false);

        // Drive forward to collect the skystone and drive back.
        driveAtHeadingForTime(0.3, 0.05, 90+attackAngle, attackAngle, 500, true);
        driveAtHeadingForTime(0.3, 0.05, 270+attackAngle, attackAngle, 500, true);

		// Stop the intake
		robot.stopIntake();

        // Rotate to running angle to go to other side of the bridge.
        rotateRobotToAngle(rotateSpeed, 90.0, 2000);

        // Move robot to center of lane before launching.  Lane defined as
		// skybridge 48 inches and robot width 18 inches, with our robot width
		// that is 24 inches to 42 inches, leaving 6 inches on either side.
		distanceFromWall(HardwareOmnibot.AlignmentSide.LEFT, 61.0, maxSpeed, 1.0, 5000);

		// Fly to the other side.  Do not put the brakes on, allow the distance
		// from wall function take over.
        driveAtHeadingForTime(maxSpeed, 0.05, 0.0, 90.0, 2000, false);

		// Get to foundation midpoint.
		distanceFromWall(HardwareOmnibot.AlignmentSide.BACK, 31.1, maxSpeed, 1.0, 5000);

		// Rotate to foundation grabbing angle.
        rotateRobotToAngle(rotateSpeed, 180.0, 2000);

		// Back the robot up to the foundation
		parallelRearTarget(0.0, 0.0, 0.05, 0.05, 0.5, 5000);

		// Grab the foundation
		moveFingers(false);

        // Might want to start the process to place the skystone here.
		// We also might want to put the perform function we use in the
		// loops of the autonomous functions.
//		robot.startLifting();
//      robot.performLifting();
		// Move the foundation to parallel back wall.
		driveAtHeadingForTime(maxSpeed, rotateSpeed, 180.0, 90.0, 3000, true);
//      robot.performLifting();

		// Rotate to parallel back wall.
        rotateRobotToAngle(rotateSpeed, 90.0, 2000);
//      robot.performLifting();

		// Drive the foundation into the back wall.
		driveAtHeadingForTime(maxSpeed, 0.05, 0.0, 90.0, 2000, true);
//      robot.performLifting();

		// Release the foundation
		moveFingers(true);
//      robot.performLifting();

		// Back the robot up to the foundation for deploying skystone
		parallelRearTarget(7.0, 7.0, 0.05, 0.05, 0.5, 5000);
//      robot.performLifting();

        // Get to running distance from the wall before placing.
        // Move robot to center of lane before launching.  Lane defined as
		// skybridge 48 inches and robot width 18 inches, with our robot width
		// that is 24 inches to 42 inches, leaving 6 inches on either side.
		distanceFromWall(HardwareOmnibot.AlignmentSide.LEFT, 61.0, maxSpeed, 1.0, 5000);

        // Wait until robot is done lifting the stone.
//      robot.performLifting();
//      while(robot.liftState != HardwareOmnibot.LiftActivity.IDLE && !isStopRequested()) {
//	        robot.performLifting();
//      }
//      
		// Release the stone on the foundation.
//		robot.startReleasing();
//        while(robot.releaseState != HardwareOmnibot.ReleaseActivity.IDLE && !isStopRequested()) {
//			robot.performingReleasing();
//		}

        // Start stowing the lift.
		// We might want to start running at a point in the stow activity before it completes.
//		robot.startStowing();
//		while(robot.stowState != HardwareOmnibot.StowActivity.IDLE && !isStopRequested()) {
//			robot.peformingStowing();
//		}

		// Fly to the other side.  Do not put the brakes on, allow the distance
		// from wall function take over.
        driveAtHeadingForTime(maxSpeed, 0.05, 180.0, 90.0, 2000, true);

        // Rotate the robot to line up to collect.
        rotateRobotToAngle(rotateSpeed, 0.0, 2000);

		// Drive out to 10cm from the skystones
		distanceFromWall(HardwareOmnibot.AlignmentSide.BACK, 110.0, maxSpeed, 1.0, 5000);

        // Stone 1 come from bridge side and go away.
        // Stone 2 and 3, come from the away and go to bridge.
		// This is due to range sensor limits.
		switch(stonePosition) {
            case 1:
			attackAngle = 45.0;
			leftDistance = 81.3;
			break;
			case 2:
			attackAngle = -45.0;
			leftDistance = 71.3;
			case 3:
			attackAngle = -45.0;
			leftDistance = 91.6;
			break;
		}

        // Start the intake to collect.
        robot.startIntake(false);

        // Drive forward to collect the skystone and drive back.
        driveAtHeadingForTime(0.3, 0.05, 90+attackAngle, attackAngle, 500, true);
        driveAtHeadingForTime(0.3, 0.05, 270+attackAngle, attackAngle, 500, true);

		// Stop the intake
		robot.stopIntake();

        // Rotate to running angle to go to other side of the bridge.
        rotateRobotToAngle(rotateSpeed, 90.0, 2000);

        // Move robot to center of lane before launching.  Lane defined as
		// skybridge 48 inches and robot width 18 inches, with our robot width
		// that is 24 inches to 42 inches, leaving 6 inches on either side.
		distanceFromWall(HardwareOmnibot.AlignmentSide.LEFT, 61.0, maxSpeed, 1.0, 5000);

		// Fly to the other side.  Do not put the brakes on, allow the distance
		// from wall function take over.
        driveAtHeadingForTime(maxSpeed, 0.05, 0.0, 90.0, 2000, false);

		// Want to start lifting here.
//		robot.startLifting();
//		robot.performLifiting();

		// Back the robot up to the foundation
        parallelRearTarget(robot.stackBackLeftFoundationDistance, robot.stackBackRightFoundationDistance, 0.05, 0.05, 0.5, 5000);
//		robot.performLifiting();

        // Get to the stacking distance from the wall.
		distanceFromWall(robot.stackFromSide, robot.stackWallDistance, maxSpeed, 1.0, 5000);
//		robot.performLifiting();

        // Wait until robot is done lifting the stone.
//      robot.performLifting();
//      while(robot.liftState != HardwareOmnibot.LiftActivity.IDLE && !isStopRequested()) {
//	        robot.performLifting();
//      }
//      
		// Release the stone on the foundation.
//		robot.startReleasing();
//        while(robot.releaseState != HardwareOmnibot.ReleaseActivity.IDLE && !isStopRequested()) {
//			robot.performingReleasing();
//		}

        // Start stowing the lift.
		// We might want to start running at a point in the stow activity before it completes.
//		robot.startStowing();
//		while(robot.stowState != HardwareOmnibot.StowActivity.IDLE && !isStopRequested()) {
//			robot.peformingStowing();
//		}

		// Park
        driveAtHeadingForTime(maxSpeed, 0.05, 180.0, 90.0, 1000, true);
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
        private Point sub1PointA = new Point(185, 239); // Stone4
        private Point sub1PointB = new Point(195, 249);
        private Point sub2PointA = new Point(185, 174); // Stone5
        private Point sub2PointB = new Point(195, 184);
        private Point sub3PointA = new Point(185, 99); // Stone6
        private Point sub3PointB = new Point(195, 109);

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