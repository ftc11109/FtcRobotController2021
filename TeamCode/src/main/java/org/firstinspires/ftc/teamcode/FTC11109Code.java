/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

// @Disabled
public class FTC11109Code extends LinearOpMode {


    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private DcMotor driveLF = null;
    private DcMotor driveLB = null;
    private DcMotor driveRF = null;
    private DcMotor driveRB = null;

    boolean teleop;
    boolean initIMU;
    boolean fieldOrientated;
    boolean parabolicDriving;

    boolean telemetryEnabled;

    Orientation angles;
    BNO055IMU.Parameters imuParameters;
    private BNO055IMU imu;

    final double headingOffset = 0.0;
    final double P_DRIVE_GAIN = 0.03;

    final double COUNTS_PER_INCH = 31.5;


    final int slidePickupLow = 1000;
    final int armPickupLow = 1001;

    final int slidePickupHigh = 2000;
    final int armPickupHigh = 2001;

    final int slideDeliverGround = 100;
    final int armDeliverGround = 101;

    final int slideDeliverLow = 200;
    final int armDeliverLow = 201;

    final int slideDeliverMedium = 300;
    final int armDeliverMedium = 301;

    final int slideDeliverHigh = 400;
    final int armDeliverHigh = 401;

    final double intakePowerDeliver = -1.0;
    final double intakePowerPickup = 1.0;
    final double intakePowerHold = 0.0;

    int slideTarget;
    int armTarget;
    double intakePower;

    final double manualSlideMultiplier = 10.0;
    final double manualArmMultiplier = 10.0;


    String startColor;
    String startAorJ;

    DetectSignalSleeveSide detectSignalSleeveSide;
    DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition parkingPosition;

    public void setTeleop(boolean newTeleop) {
        teleop = newTeleop;
    }

    public void setInitIMU(boolean newInitIMU) {
        initIMU = newInitIMU;
    }

    @Override
    public void runOpMode() {
        myInit();
        while (!opModeIsActive()) {
            if (isStopRequested()) {
                myStop();
                return;
            }
            myInitLoop();
        }

        if (isStopRequested()) {
            myStop();
            return;
        }
        myStart();

        while (opModeIsActive()) {
            if (isStopRequested()) {
                myStop();
                return;
            }
            myLoop();
        }
        myStop();
    }


    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void myInit() {
        fieldOrientated = true;
        parabolicDriving = teleop;

        telemetryEnabled = true;

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        driveLF = hardwareMap.get(DcMotor.class, "left_driveF");
        driveLB = hardwareMap.get(DcMotor.class, "left_driveB");
        driveRF = hardwareMap.get(DcMotor.class, "right_driveF");
        driveRB = hardwareMap.get(DcMotor.class, "right_driveB");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        driveLF.setDirection(DcMotor.Direction.FORWARD);
        driveLB.setDirection(DcMotor.Direction.FORWARD);
        driveRF.setDirection(DcMotor.Direction.REVERSE);
        driveRB.setDirection(DcMotor.Direction.REVERSE);

        driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;

        if (initIMU) {
            // Initialize IMU.
            imu.initialize(imuParameters);
        }

        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (!teleop) {

            detectSignalSleeveSide = new DetectSignalSleeveSide();
            detectSignalSleeveSide.init(hardwareMap);
        }

        slideTarget = 0;
        armTarget = 0;


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void myInitLoop() {

        if (gamepad1.x) {
            startColor = "blue";
        } else if (gamepad1.b) {
            startColor = "red";
        }

        if (gamepad1.y) {
            startAorJ = "judges";
        } else if (gamepad1.a) {
            startAorJ = "audience";
        }


        if (!teleop) {
            parkingPosition = detectSignalSleeveSide.getParkingPosition();
        }

        telemetry.addData("teleop", teleop);

        telemetry.addData("startColor", startColor);
        telemetry.addData("startAudienceOrWarehouse", startAorJ);
        telemetry.addData("parkingPosition", parkingPosition);

        if (detectSignalSleeveSide != null) {
            telemetry.addData("average red", detectSignalSleeveSide.getAverageRed());
            telemetry.addData("average green", detectSignalSleeveSide.getAverageGreen());
            telemetry.addData("average blue", detectSignalSleeveSide.getAverageBlue());
        }
        telemetry.update(); //sends telemetry to the driver controller. all telemetry must come BEFORE this line.


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void myStart() {
        runtime.reset();
        if (detectSignalSleeveSide != null) {
            parkingPosition = detectSignalSleeveSide.start();
        }

        if (!teleop) {
            int sleepTime = 500;
            double tolerance = 1;
            double turnTolerance = .5;
            int failSafeCountThreshold = 4;
            double power = .3;
            double powerin2 = .15;
            strafeToPosition(24, .3, sleepTime, tolerance);
            turn(0, .3, powerin2, turnTolerance, 2, failSafeCountThreshold);
            runToPosition(40, .3, sleepTime, tolerance);
            turn(-90, .3, powerin2, turnTolerance, 2, failSafeCountThreshold);
            //deliver cone
            sleep(800);
            strafeToPosition(-12, .3, sleepTime, tolerance);
            turn(-90, .3, powerin2, turnTolerance, 2, failSafeCountThreshold);
            runToPosition(-48, .3, sleepTime, tolerance);
            turn(-90, .3, powerin2, turnTolerance, 2, failSafeCountThreshold);
            //pickup cone

            runToPositionLeftRight(58, 44, .3, .3, sleepTime, tolerance);
            //deliver cone

            // if we didn't detect a parking spot, pick a good default
            if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.DETECTING){
                parkingPosition = DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.RIGHT;
            }
            // actually park!
            if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.LEFT) {
                runToPositionLeftRight(-14, 0, .3, .3, sleepTime, tolerance);
                turn(-90, .3, powerin2, turnTolerance, 2, failSafeCountThreshold);
                runToPositionLeftRight(-44, -44, .3, .3, sleepTime, tolerance);
                //pickup cone

            }
            else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER) {
                runToPositionLeftRight(-14, 0, .3, .3, sleepTime, tolerance);
                turn(-90, .3, powerin2, turnTolerance, 2, failSafeCountThreshold);
                runToPositionLeftRight(-20, -20, .3, .3, sleepTime, tolerance);
                turn(-180, .3, powerin2, turnTolerance, 2, failSafeCountThreshold);

            }
            else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.RIGHT){
                runToPositionLeftRight(-14, 0, .3, .3, sleepTime, tolerance);
                turn(-180, .3, powerin2, turnTolerance, 2, failSafeCountThreshold);
            }



        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void myLoop() {
        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (telemetryEnabled) {
            telemetry.addData("rot about Z", angles.firstAngle);
            telemetry.addData("rot about Y", angles.secondAngle);
            telemetry.addData("rot about X", angles.thirdAngle);

        }

        gamepadDriveMotors();
        pickupAndDelivery();

        if (telemetryEnabled) {
            // updateDriverStation
            telemetry.update();
        }


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void myStop() {
        driveLF.setPower(0);
        driveLB.setPower(0);
        driveRF.setPower(0);
        driveRB.setPower(0);
    }


    //teleop drive code
    private double adjustForVoltage(double power) {
        return power;
    }

    private void driveMotors(double fwd, double strafe, double rotate) {
        double tempForward;
        double denominator;


        if (telemetryEnabled) {
            telemetry.addData("fwd", fwd);
            telemetry.addData("strafe", strafe);
        }
        if (teleop) {
            if (fieldOrientated) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double currentAngle = angles.firstAngle;
                tempForward = fwd;
                fwd = -(strafe * -Math.sin(currentAngle / 180 * Math.PI) + tempForward * Math.cos(currentAngle / 180 * Math.PI));
                strafe = 1.35 * -(strafe * Math.cos(currentAngle / 180 * Math.PI) + tempForward * Math.sin(currentAngle / 180 * Math.PI));
                if (gamepad1.a) {
                    // green
                    rotate = rotate + teleopTurnToAngle(180, currentAngle, 0.4, 0.1);
                } else if (gamepad1.b) {
                    // red
                    rotate = rotate + teleopTurnToAngle(-90, currentAngle, 0.4, 0.1);
                } else if (gamepad1.x) {
                    // blue
                    rotate = rotate + teleopTurnToAngle(90, currentAngle, 0.4, 0.1);
                } else if (gamepad1.y) {
                    // yellow
                    rotate = rotate + teleopTurnToAngle(0, currentAngle, 0.4, 0.1);
                }
            }
        }
        if (telemetryEnabled) {
            telemetry.addData("field fwd", fwd);
            telemetry.addData("field strafe", strafe);
        }
        driveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        denominator = Math.max(Math.abs(fwd) + Math.abs(strafe) + Math.abs(rotate), 1.000001);
        if (telemetryEnabled) {
            telemetry.addData("denominator", denominator);
        }
        driveLF.setPower(adjustForVoltage((fwd + strafe + rotate) / denominator));
        driveLB.setPower(adjustForVoltage(((fwd - strafe) + rotate) / denominator));
        driveRF.setPower(adjustForVoltage(((fwd - strafe) - rotate) / denominator));
        driveRB.setPower(adjustForVoltage(((fwd + strafe) - rotate) / denominator));
        if (telemetryEnabled) {
            telemetry.addData("left_driveF", (fwd + strafe + rotate) / denominator);
            telemetry.addData("left_driveB", ((fwd - strafe) + rotate) / denominator);
            telemetry.addData("right_driveF", ((fwd - strafe) - rotate) / denominator);
            telemetry.addData("right_driveB", ((fwd + strafe) - rotate) / denominator);
        }
    }

    private void gamepadDriveMotors() {
        double fwdSign;
        double strafeSign;
        double dpadFwd;
        double dpadStrafe;


        fwdSign = 0;
        strafeSign = 0;
        // gamepad 1 joysticks and slow mode
        dpadFwd = 0.25;
        dpadStrafe = 0.25;
        if (gamepad1.dpad_up) {
            if (gamepad1.dpad_right) {
                driveMotors(-(dpadFwd / 1.4), -(dpadStrafe / 1.4), gamepad1.right_stick_x / 3);
            } else if (gamepad1.dpad_left) {
                driveMotors(-(dpadFwd / 1.4), dpadStrafe / 1.4, gamepad1.right_stick_x / 3);
            } else {
                driveMotors(-dpadFwd, 0, gamepad1.right_stick_x / 3);
            }
        } else if (gamepad1.dpad_down) {
            if (gamepad1.dpad_right) {
                driveMotors(dpadFwd / 1.4, -(dpadStrafe / 1.4), gamepad1.right_stick_x / 3);
            } else if (gamepad1.dpad_left) {
                driveMotors(dpadFwd / 1.4, dpadStrafe / 1.4, gamepad1.right_stick_x / 3);
            } else {
                driveMotors(dpadFwd, 0, gamepad1.right_stick_x / 3);
            }
        } else if (gamepad1.dpad_left) {
            driveMotors(0, dpadStrafe, gamepad1.right_stick_x / 3);
        } else if (gamepad1.dpad_right) {
            driveMotors(0, -dpadStrafe, gamepad1.right_stick_x / 3);
        } else {
            if (parabolicDriving) {
                driveMotors(0.6 * parabolicTransform(gamepad1.left_stick_y), 0.6 * -parabolicTransform(gamepad1.left_stick_x), gamepad1.right_stick_x / 3);
            } else {
                driveMotors(fwdSign + gamepad1.left_stick_y * 0.5, -(strafeSign + gamepad1.left_stick_x * 0.5), gamepad1.right_stick_x / 4);
            }
        }

        if (telemetryEnabled) {
            telemetry.addData("GP1 LeftX", gamepad1.left_stick_x);
            telemetry.addData("GP1 LeftY", gamepad1.left_stick_y);
            telemetry.addData("GP1 RightX", gamepad1.right_stick_x);

        }
    }

    private double parabolicTransform(double stick) {
        if (stick < 0) {
            stick = -Math.pow(stick, 2);
        } else {
            stick = Math.pow(stick, 2);
        }
        return stick;
    }

    private double teleopTurnToAngle(double targetAngle, double currentAngle, double powerIn, double powerIn2) {
        double angleDifference = targetAngle - currentAngle;
        if (angleDifference > 180) {
            angleDifference += -360;
        }
        if (angleDifference < -180) {
            angleDifference += 360;
        }
        if (telemetryEnabled) {
            telemetry.addData("angleDifference", angleDifference);
            telemetry.addData("currentAngle", currentAngle);
        }

        double rotateOut;

        if (Math.abs(angleDifference) < 15) {
            rotateOut = powerIn2;
        } else if (Math.abs(angleDifference) < 45) {
            rotateOut = (Math.abs(angleDifference) / 45) * (powerIn - powerIn2) + powerIn2;
        } else {
            rotateOut = powerIn;
        }
        if (angleDifference < -2) {
            rotateOut = rotateOut;
        } else if (angleDifference > 2) {
            rotateOut = -1 * rotateOut;
        } else {
            rotateOut = 0;
        }
        return rotateOut;
    }


    //autonomous drive code
    private void runToPositionInit(int targetPosition, double power) {
        driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLF.setPower(power);
        driveLB.setPower(power);
        driveRF.setPower(power);
        driveRB.setPower(power);
        driveLF.setTargetPosition(targetPosition);
        driveLB.setTargetPosition(targetPosition);
        driveRF.setTargetPosition(targetPosition);
        driveRB.setTargetPosition(targetPosition);
        driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void strafeToPosition(double targetInches, double power, int sleepTime, double tolerance) {
        int targetPosition = (int) (targetInches * 32.5);
        tolerance = tolerance * 32.5;
        ((DcMotorEx) driveLF).setTargetPositionTolerance((int) tolerance);
        ((DcMotorEx) driveRF).setTargetPositionTolerance((int) tolerance);
        ((DcMotorEx) driveLB).setTargetPositionTolerance((int) tolerance);
        ((DcMotorEx) driveRB).setTargetPositionTolerance((int) tolerance);
        driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLF.setPower(power);
        driveRF.setPower(power);
        driveLB.setPower(power);
        driveRB.setPower(power);
        driveLF.setTargetPosition(targetPosition);
        driveRF.setTargetPosition(-targetPosition);
        driveLB.setTargetPosition(-targetPosition);
        driveRB.setTargetPosition(targetPosition);
        driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {

            if (driveLF.isBusy() || driveLB.isBusy() || driveRF.isBusy() || driveRB.isBusy()) {
                continue;
            }
            break;
        }
        sleep(sleepTime);
    }


    private void strafeToPositionIMU(double targetInches, double power, int sleepTime, double tolerance, double targetHeading) {
        int targetPosition = (int) (targetInches * 32.5);
        tolerance = tolerance * 32.5;
        ((DcMotorEx) driveLF).setTargetPositionTolerance((int) tolerance);
        ((DcMotorEx) driveRF).setTargetPositionTolerance((int) tolerance);
        ((DcMotorEx) driveLB).setTargetPositionTolerance((int) tolerance);
        ((DcMotorEx) driveRB).setTargetPositionTolerance((int) tolerance);
        driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLF.setPower(power);
        driveRF.setPower(power);
        driveLB.setPower(power);
        driveRB.setPower(power);
        driveLF.setTargetPosition(targetPosition);
        driveRF.setTargetPosition(-targetPosition);
        driveLB.setTargetPosition(-targetPosition);
        driveRB.setTargetPosition(targetPosition);
        driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive()) {
            if (driveLF.isBusy() || driveLB.isBusy() || driveRF.isBusy() || driveRB.isBusy()) {
                // Determine required steering to keep on heading
                double turnPower = getSteeringCorrection(targetHeading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (targetPosition < 0)
                    turnPower *= -1.0;

                driveLF.setPower(power);
                driveRF.setPower(power);
                driveLB.setPower(power);
                driveRB.setPower(power);

                continue;
            }
            break;
        }
        sleep(sleepTime);
    }

    public double getSteeringCorrection(double targetHeading, double proportionalGain) {
        // Get the robot heading by applying an offset to the IMU heading
        double robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        double headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }


    private void driveTelemetryRunToPosition() {
        // set how close to the target position the robot stops at
        // set last position to current position
        // current position to the current position
        double currentPositionLF = driveLF.getCurrentPosition();
        double currentPositionLB = driveLB.getCurrentPosition();
        double currentPositionRF = driveRF.getCurrentPosition();
        double currentPositionRB = driveRB.getCurrentPosition();
        telemetry.addData("currentPositionLF", currentPositionLF);
        telemetry.addData("currentPositionLB", currentPositionLB);
        telemetry.addData("currentPositionRF", currentPositionRF);
        telemetry.addData("currentPositionRB", currentPositionRB);
        telemetry.addData("isBusyLF", driveLF.isBusy());
        telemetry.addData("isBusyLB", driveLB.isBusy());
        telemetry.addData("isBusyRF", driveRF.isBusy());
        telemetry.addData("isBusyRB", driveRB.isBusy());

    }

    private void runToPosition(double targetInches, double power, int sleepTime, double tolerance) {
        int targetPosition = (int) (targetInches * 31.25);
        tolerance = tolerance * 31.25;
        runToPositionInit(targetPosition, power);
        while (opModeIsActive()) {
            if (telemetryEnabled) {
                driveTelemetryRunToPosition();
            }

            if (Math.abs(targetPosition - driveLF.getCurrentPosition()) < tolerance && Math.abs(targetPosition - driveRF.getCurrentPosition()) < tolerance && Math.abs(targetPosition - driveLB.getCurrentPosition()) < tolerance && Math.abs(targetPosition - driveRB.getCurrentPosition()) < tolerance) {
                break;
            }
            if (driveLF.isBusy() || driveLB.isBusy() || driveRF.isBusy() || driveRB.isBusy()) {
                continue;
            }
            break;
        }
        sleep(sleepTime);
    }

    private void initRunToPositionLeftRight(int targetPositionL, int targetPositionR, double powerL, double powerR, int sleepTime) {
        driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLF.setPower(powerL);
        driveRF.setPower(powerR);
        driveLB.setPower(powerL);
        driveRB.setPower(powerR);
        driveLF.setTargetPosition(targetPositionL);
        driveRF.setTargetPosition(targetPositionR);
        driveLB.setTargetPosition(targetPositionL);
        driveRB.setTargetPosition(targetPositionR);
        driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    private void runToPositionLeftRight(double targetInchesL, double targetInchesR, double powerL, double powerR, int sleepTime, double tolerance) {
        int targetPositionL = (int) (targetInchesL * 31.25);
        int targetPositionR = (int) (targetInchesR * 31.25);
        tolerance = tolerance * 31.25;
        ((DcMotorEx) driveLF).setTargetPositionTolerance((int) tolerance);
        ((DcMotorEx) driveRF).setTargetPositionTolerance((int) tolerance);
        ((DcMotorEx) driveLB).setTargetPositionTolerance((int) tolerance);
        ((DcMotorEx) driveRB).setTargetPositionTolerance((int) tolerance);
        initRunToPositionLeftRight(targetPositionL, targetPositionR, powerL, powerR, sleepTime);
        while (opModeIsActive()) {

            if (driveLF.isBusy() || driveLB.isBusy() || driveRF.isBusy() || driveRB.isBusy()) {
                continue;
            }
            break;
        }
        sleep(sleepTime);
    }

/*
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {
        private double  targetHeading = 0;
        private double  driveSpeed    = 0;
        private double  turnSpeed     = 0;
        private double  leftSpeed     = 0;
        private double  rightSpeed    = 0;
        private int     leftTarget    = 0;
        private int     rightTarget   = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = leftDrive.getCurrentPosition() + moveCounts;
            rightTarget = rightDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (0 > distance)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

*/


    private void turnToAngle(int targetAngle, double turnPower, double turnPower2, int sleepTime) {
        turn(targetAngle, turnPower, turnPower2, 2, 4, 4);
    }


    private void turn(int targetAngle, double powerIn, double powerIn2, double tolerance, int targetReachedCountThreshold, int failSafeCountThreshold) {
        double rotate;

        int failSafeCount = 0;
        int targetReachedCount = 0;
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentAngle = angles.firstAngle;
            double angleDifference = targetAngle - currentAngle;
            if (angleDifference > 180) {
                angleDifference += -360;
            }
            if (angleDifference < -180) {
                angleDifference += 360;
            }
            if (telemetryEnabled) {
                telemetry.addData("angleDifference", angleDifference);
                telemetry.addData("currentAngle", currentAngle);
            }

            double power;

            if (Math.abs(angleDifference) < 10) {
                power = powerIn2;
            } else if (Math.abs(angleDifference) < 40) {
                power = (Math.abs(angleDifference) / 40) * (powerIn - powerIn2) + powerIn2;
            } else {
                power = powerIn;
            }
            if (angleDifference < -tolerance) {
                targetReachedCount = 0;
                rotate = power;
            } else if (angleDifference > tolerance) {
                targetReachedCount = 0;
                rotate = -1 * power;
            } else {
                targetReachedCount += 1;
                failSafeCount += 1;
                rotate = 0;
                driveMotors(0, 0, 0);
                if (targetReachedCount >= targetReachedCountThreshold || failSafeCount >= failSafeCountThreshold) {
                    break;
                }
            }
            driveMotors(0, 0, (float) rotate);
            telemetry.update();
        }
        driveMotors(0, 0, 0);
        telemetry.update();
    }


    private void setTargets(int arm, int slide) {
        armTarget = arm;
        slideTarget = slide;
    }

    private void pickupAndDelivery() {
        int slideTargetOld = slideTarget;
        int armTargetOld = armTarget;

        double intakePowerOld = intakePower;

        if (gamepad2.y) {
            setTargets(armDeliverHigh, slideDeliverHigh);
        } else if (gamepad2.b) {
            setTargets(armDeliverMedium, slideDeliverMedium);
        } else if (gamepad2.a) {
            setTargets(armDeliverLow, slideDeliverLow);
        } else if (gamepad2.x) {
            setTargets(armDeliverGround, slideDeliverGround);
        }

        if (gamepad2.dpad_up) {
            setTargets(armPickupHigh, slidePickupHigh);
        } else if (gamepad2.dpad_down) {
            setTargets(armPickupLow, slidePickupLow);
        }

        {
            double leftStick = -gamepad2.left_stick_y;
            if (Math.abs(leftStick) > 0.05) {
                slideTarget += (int) (leftStick * manualSlideMultiplier);
            }
        }

        {
            double rightStick = -gamepad2.right_stick_y;
            if (Math.abs(rightStick) > 0.05) {
                armTarget += (int) (rightStick * manualArmMultiplier);
            }
        }

        {
            if (gamepad1.right_bumper) {
                intakePower = -1.0;
            } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakePower = 1.0;
            } else {
                intakePower = 0.0;
            }
        }


        if (armTarget != armTargetOld) {
            //motorArm.setTargetPosition(armTarget);
        }

        if (slideTarget != slideTargetOld) {
            //motorSlide.setTargetPosition(slideTarget);
        }

        if (intakePower != intakePowerOld) {
            //motorIntake.setPower(intakePower);
        }

        if (telemetryEnabled) {
            telemetry.addData("armTarget", armTarget);
            telemetry.addData("slideTarget", slideTarget);
        }


    }


}
