package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Math.atan2;
import static java.lang.Math.toDegrees;

/**
 * Created by Ethan on 12/2/2016.
 */

@TeleOp(name="Omni: TeleOpTest", group ="TeleOp")
public class OmniTeleTest extends OpMode {

    public HardwareOmnibot robot = new HardwareOmnibot();

    @Override
    public void init() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        robot.init(hardwareMap);
        robot.setInputShaping(true);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
    }

    private double driverAngle = 0.0;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private boolean aHeld = false;
    private boolean bHeld = false;
    private boolean yHeld = false;
    private boolean xHeld = false;
    private boolean a2Held = false;
    private boolean b2Held = false;
    private boolean y2Held = false;
    private boolean x2Held = false;
    private boolean up2Held = false;
    private boolean down2Held = false;
    private boolean aPressed;
    private boolean bPressed;
    private boolean yPressed;
    private boolean xPressed;
    private boolean a2Pressed;
    private boolean b2Pressed;
    private boolean y2Pressed;
    private boolean x2Pressed;
    private boolean up2Pressed;
    private boolean down2Pressed;
    private double yPower;
    private double xPower;
    private double spin;
    private double gyroAngle;
    private double liftPower;
    private double extendPower;
    private double collectPower;

    @Override
    public void start()
    {
    }

    @Override
    public void loop() {
        //left joystick is for moving
        //right joystick is for rotation
        gyroAngle = robot.readIMU();

        yPower = -gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        spin = gamepad1.right_stick_x;
        aPressed = gamepad1.a;
        bPressed = gamepad1.b;
        yPressed = gamepad1.y;
        xPressed = gamepad1.x;
        a2Pressed = gamepad2.a;
        b2Pressed = gamepad2.b;
        y2Pressed = gamepad2.y;
        x2Pressed = gamepad2.x;
        up2Pressed = gamepad2.dpad_up;
        down2Pressed = gamepad2.dpad_down;

        if(!aHeld && aPressed)
        {
            aHeld = true;
            robot.extendIntake();
        } else if(!aPressed) {
            aHeld = false;
        }

        if(!bHeld && bPressed)
        {
            bHeld = true;
            robot.toggleIntake(false);
        } else if(!bPressed) {
            bHeld = false;
        }

        if(!yHeld && yPressed)
        {
            yHeld = true;
            robot.toggleIntake(true);
        } else if(!yPressed) {
            yHeld = false;
        }

        if(!xHeld && xPressed)
        {
            xHeld = true;
            robot.retractIntake();
        } else if(!xPressed) {
            xHeld = false;
        }

        if(!a2Held && a2Pressed)
        {
            a2Held = true;
            if(robot.frontLeftMotorPower > 0.5) {
                robot.setFrontLeftMotorPower(0.0);
            } else {
                robot.setFrontLeftMotorPower(1.0);
            }
        } else if(!a2Pressed) {
            a2Held = false;
        }

        if(!b2Held && b2Pressed)
        {
            b2Held = true;
            if(robot.frontRightMotorPower > 0.5) {
                robot.setFrontRightMotorPower(0.0);
            } else {
                robot.setFrontRightMotorPower(1.0);
            }
        } else if(!b2Pressed) {
            b2Held = false;
        }

        if(!y2Held && y2Pressed)
        {
            y2Held = true;
            if(robot.rearLeftMotorPower > 0.5) {
                robot.setRearLeftMotorPower(0.0);
            } else {
                robot.setRearLeftMotorPower(1.0);
            }
        } else if(!y2Pressed) {
            y2Held = false;
        }

        if(!x2Held && x2Pressed)
        {
            x2Held = true;
            robot.runLift();
        } else if(!x2Pressed) {
            x2Held = false;
        }

        if(!up2Held && up2Pressed)
        {
            up2Held = true;
            robot.addStone();
        } else if (!up2Pressed) {
			up2Held = false;
		}

        if(!down2Held && down2Pressed)
        {
            down2Held = true;
            robot.removeStone();
        } else if (!down2Pressed) {
			down2Held = false;
		}

        if(Math.abs(xPower) > 0.1) {
            robot.manualExtendIntake(xPower);
        } else {
            robot.manualExtendIntake(0.0);
        }

        if(Math.abs(spin) > 0.1) {
            robot.manualLift(spin);
        } else {
            robot.manualLift(0.0);
        }
        // If the activity is not performing, it will be idle and return.
//        robot.performLifting();
//        robot.performReleasing();
//        robot.performStowing();


		telemetry.addData("Lift Target Height: ", robot.liftTargetHeight.toString());
        telemetry.addData("Y Power: ", yPower);
        telemetry.addData("X Power: ", xPower);
        telemetry.addData("Spin: ", spin);
        telemetry.addData("Offset Angle: ", driverAngle);
        telemetry.addData("Gyro Angle: ", gyroAngle);
        telemetry.addData("Gyro X Angle: ", robot.xAngle);
        telemetry.addData("Gyro Y Angle: ", robot.yAngle);
        telemetry.addData("Gyro Z Angle: ", robot.zAngle);
        telemetry.addData("Front Left Encoder: ", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Encoder: ", robot.frontRight.getCurrentPosition());
        telemetry.addData("Rear Left Encoder: ", robot.rearLeft.getCurrentPosition());
        telemetry.addData("Rear Right Encoder: ", robot.rearRight.getCurrentPosition());
        telemetry.addData("Lifter Encoder: ", robot.lifter.getCurrentPosition());
        telemetry.addData("Extender Encoder: ", robot.extender.getCurrentPosition());
        updateTelemetry(telemetry);
    }

    @Override
    public void stop() {
        robot.stopGroundEffects();
    }
}
