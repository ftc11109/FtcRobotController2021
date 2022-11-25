/* TODO disable telemetry, calibrate camera detection
* Required:
 * Auto: tune for new robot
 * New robot can park
  * Teleop: make sure slide and arm positions carry over (don't reset encoders?)


 * Auto: Write AutoPickupCone() and use it.

* High priority:
 * Teleop: One button to press to pick up cone.  Releasing button moves to slideDeliverMedium?
 * Auto: detect red line to center on cone stack.

* Low Priority:
 * Teleop: Pressing button picks up cone, detects when it's picked up, then goes to slideDeliverMedium.
 * Auto: if camera init fails, pick best parking spot to start tele.
 * All: Split myStart and myLoop to AutoStart and TeleStart and TeleLoop.
 * All: use camera to detect junction left/right/centered and automatically center if needed.

* Done:
 * Auto: park based on signal side
 * Auto: Test Red Audience.
 * Auto: For StartColor and StartAorJ use finals instead of literal strings.
 * Auto: Make a helper: boolean StartSpot(color, side) to check if they match.
 * Auto: Write and test Blue Audience.
 * Auto: Write AutoDeliverCone() and use it.
 * Auto: Write and test Red Judge.
 * Auto: Write and test Blue Judge.
 * Teleop: save & restore IMU (and/or have a "reset heading" button?  check if static class variables carry over?)
 * All: Configure arm and slide positions for pickup and delivery.
 */


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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

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
    private DistanceSensor sensorDistances[];
    final boolean useDistance = true;
    final double furtherDistance = 48;
    final double deliverDistance = 3;// TODO: Tune deliver distance



    NormalizedColorSensor sensorColorLeft;
    NormalizedColorSensor sensorColorRight;
    final boolean useColor = true;
    float gain = 10;
    final float[] hsvValues = new float[3];
    final int saturationDivisor = 100; //what the difference in saturation is devided by to get you're strafe power




   private final ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private DcMotor driveLF = null;
    private DcMotor driveLB = null;
    private DcMotor driveRF = null;
    private DcMotor driveRB = null;

    private DcMotor motorSlideL = null;
    private DcMotor motorSlideR = null;
    private DcMotor motorArm = null;
    private DcMotor motorIntake = null;

    boolean teleop;
    boolean teleopFollowsAuto;
    boolean fieldOrientated;
    boolean parabolicDriving;

    boolean telemetryEnabled;

    Orientation angles;
    BNO055IMU.Parameters imuParameters;
    private BNO055IMU imu;
    double angleOffset = 0.0;

    final double headingOffset = 0.0;
    final double P_DRIVE_GAIN = 0.03;

    final double COUNTS_PER_INCH = 57.005;


    final int slidePickupLow = 137;
    final int armPickupLow = 5;

    // TODO calibrate slidePickupHigh
    final int slidePickupHigh = 335;
    final int armPickupHigh = 5;

    final int slideDeliverGround = 0;
    final int armDeliverGround = 5;

    final int slidePowerOff = -50;

    final int slideDeliverLow = 315;
    final int armDeliverLow = 2750;

    final int slideDeliverMedium = 190;
    final int armDeliverMedium = 1960;
    final double distanceToJunctionMedium = 5.0;

    final int slideDeliverHigh = 465;
    final int armDeliverHigh = 1720;
    final double getDistanceToJunctionHigh = 3.0;

    int conesRemaining = 5;

    final int slideMax = 466;
    final int armMax = 2800;

    final int slidePickupTarget = 5;

    final double intakePowerDeliver = -1.0;
    final double intakePowerPickup = 1.0;
    final double intakePowerHold = 0.2;
    final double intakePowerOff = 0.0;

    int slideTarget;
    int armTarget;
    double intakePower;

    final double manualSlideMultiplier = 10.0;
    final double manualArmMultiplier = 26.0;
    final int armTolerance = 10;
    final double armPower = 0.7;
    final int slideTolerance = 4;
    final double slidePower = 0.5;


    final int slideTargetPickup = -1000;

    final boolean bothSlideMotors = true;




    String startColor;
    String startAorJ;

    final String RED = "red";
    final String BLUE = "blue";
    final String AUDIENCE = "audience";
    final String JUDGE = "judge";

    DetectSignalSleeveSide detectSignalSleeveSide;
    DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition parkingPosition;

    boolean buttonPushedLast = false;

    public void setTeleop(boolean newTeleop) {
        teleop = newTeleop;
    }

    public void setTeleopFollowsAuto(boolean newTeleopFollowsAuto) {
        teleopFollowsAuto = newTeleopFollowsAuto;
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
        driveLF = hardwareMap.get(DcMotor.class, "driveLF");
        driveLB = hardwareMap.get(DcMotor.class, "driveLB");
        driveRF = hardwareMap.get(DcMotor.class, "driveRF");
        driveRB = hardwareMap.get(DcMotor.class, "driveRB");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        driveLF.setDirection(DcMotor.Direction.REVERSE);
        driveLB.setDirection(DcMotor.Direction.FORWARD);
        driveRF.setDirection(DcMotor.Direction.REVERSE);
        driveRB.setDirection(DcMotor.Direction.FORWARD);

        driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (bothSlideMotors) motorSlideL = hardwareMap.get(DcMotor.class, "motorSlideL");
        motorSlideR = hardwareMap.get(DcMotor.class, "motorSlideR");
        motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");

        if (bothSlideMotors) {motorSlideL.setDirection(DcMotor.Direction.REVERSE);}
        motorSlideR.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setDirection(DcMotor.Direction.FORWARD);
        motorIntake.setDirection(DcMotor.Direction.REVERSE);

        if (!teleopFollowsAuto) {
            if (bothSlideMotors) {motorSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
            motorSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (bothSlideMotors) {motorSlideL.setPower(slidePower);}
        motorSlideR.setPower(slidePower);
        motorArm.setPower(armPower);
        motorIntake.setPower(0);

        ((DcMotorEx) motorSlideR).setTargetPositionTolerance(slideTolerance);

        if (bothSlideMotors) {
            ((DcMotorEx) motorSlideL).setTargetPositionTolerance(slideTolerance);
            motorSlideL.setTargetPosition(0);
        }

        motorSlideR.setTargetPosition(0);
        motorArm.setTargetPosition(0);
        ((DcMotorEx) motorArm).setTargetPositionTolerance(armTolerance);

        if (bothSlideMotors) {motorSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
        motorSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // sensorz
        if (useDistance) {
            sensorDistances = new DistanceSensor[]{
                    hardwareMap.get(DistanceSensor.class, "distance0"),
                    hardwareMap.get(DistanceSensor.class, "distance1"),
                    hardwareMap.get(DistanceSensor.class, "distance2"),
                    hardwareMap.get(DistanceSensor.class, "distance3"),
                    hardwareMap.get(DistanceSensor.class, "distance4"),
            };
        }

        if (useColor) {
            sensorColorLeft = hardwareMap.get(NormalizedColorSensor.class, "sensorColorLeft");
            sensorColorRight = hardwareMap.get(NormalizedColorSensor.class, "sensorColorRight");
            sensorColorLeft.setGain(gain);
            sensorColorRight.setGain(gain);


        }


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

        imu.initialize(imuParameters);


        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (teleopFollowsAuto) {
            Files files = new Files();
            angleOffset = -files.readFileDouble("IMUOffset");
        }

        if (!teleop) {

            detectSignalSleeveSide = new DetectSignalSleeveSide();
            detectSignalSleeveSide.init(hardwareMap);
        }

        startColor = RED;
        startAorJ = AUDIENCE;

        slideTarget = 0;
        armTarget = 0;


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void myInitLoop() {

        if (gamepad1.x) {
            startColor = BLUE;
        } else if (gamepad1.b) {
            startColor = RED;
        }

        if (gamepad1.y) {
            startAorJ = JUDGE;
        } else if (gamepad1.a) {
            startAorJ = AUDIENCE;
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
        if (telemetryEnabled) {
            telemetry.addData("motorArm", motorArm.getCurrentPosition());
            if (bothSlideMotors) {telemetry.addData("motorSlideL", motorSlideL.getCurrentPosition());}
            telemetry.addData("motorSlideR", motorSlideR.getCurrentPosition());
            telemetry.addData("motorIntake", motorIntake.getCurrentPosition());
        }

        telemetry.update(); //sends telemetry to the driver controller. all telemetry must come BEFORE this line.


    }

    public boolean Spot(String color, String AJ) {
        return color == startColor && AJ == startAorJ;
    }
    public void autoDeliverCone (){
        //motorIntake.setPower(intakePowerDeliver);
        sleep(500);
        //motorIntake.setPower(intakePowerOff);
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
//            turnToAngle(45, .6, .3);
//            motorSlideL.setTargetPosition(150);
//            motorSlideR.setTargetPosition(150);
//            motorArm.setTargetPosition(100);
//            auto1();
//            autoDeliverPark();
//            autoTest();
              autoDeliverPark4();

        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void myLoop() {
        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (telemetryEnabled) {
            telemetry.addData("angleOffset", angleOffset);
            telemetry.addData("rot about Z", angles.firstAngle);
            telemetry.addData("rot about Z with offset", angles.firstAngle + angleOffset);
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
        if (!teleop) {
            Files files = new Files();
            double endAngle = getRawHeading();
            files.saveFileDouble("IMUOffset", endAngle);
        }

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
                double currentAngle = angles.firstAngle + angleOffset;
                tempForward = fwd;
                fwd = -(strafe * -Math.sin(currentAngle / 180 * Math.PI) + tempForward * Math.cos(currentAngle / 180 * Math.PI));
                strafe = 1.35 * -(strafe * Math.cos(currentAngle / 180 * Math.PI) + tempForward * Math.sin(currentAngle / 180 * Math.PI));
                if (gamepad1.a) {
                    // green
                    rotate = rotate + teleopTurnToAngle(180, currentAngle, 0.6, 0.2);
                } else if (gamepad1.b) {
                    // red
                    rotate = rotate + teleopTurnToAngle(-90, currentAngle, 0.6, 0.2);
                } else if (gamepad1.x) {
                    // blue
                    rotate = rotate + teleopTurnToAngle(90, currentAngle, 0.6, 0.2);
                } else if (gamepad1.y) {
                    // yellow
                    rotate = rotate + teleopTurnToAngle(0, currentAngle, 0.6, 0.2);
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
                driveMotors(0.8 * parabolicTransform(gamepad1.left_stick_y), 0.8 * -parabolicTransform(gamepad1.left_stick_x), gamepad1.right_stick_x / 1.6);
            } else {
                driveMotors(fwdSign + gamepad1.left_stick_y * 0.5, -(strafeSign + gamepad1.left_stick_x * 0.5), gamepad1.right_stick_x / 4);
            }
        }

        if (telemetryEnabled) {
            telemetry.addData("GP1 LeftX", gamepad1.left_stick_x);
            telemetry.addData("GP1 LeftY", gamepad1.left_stick_y);
            telemetry.addData("GP1 RightX", gamepad1.right_stick_x);

            if (useDistance) {
                for (int i = 0; i < sensorDistances.length; i++) {
                    double currentDistance = sensorDistances[i].getDistance(DistanceUnit.INCH);
                    telemetry.addData("distance", currentDistance);
                }
            }
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
        int targetPosition = (int) (targetInches * COUNTS_PER_INCH);
        tolerance = tolerance * COUNTS_PER_INCH;
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
        int targetPosition = (int) (targetInches * COUNTS_PER_INCH);
        tolerance = tolerance * COUNTS_PER_INCH;
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
        double robotHeading = getRawHeading() + angleOffset - headingOffset;

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

    private void autoJunctionDeliver(int junctionHeight){
        int sleepTime = 500;

        double lowestDistance = sensorDistances[0].getDistance(DistanceUnit.INCH);
        int lowestSensor = 0;
        for (int i = 1; i < sensorDistances.length; i++){
            double currentDistance = sensorDistances[i].getDistance(DistanceUnit.INCH);
            if (currentDistance < lowestDistance){
                lowestDistance = currentDistance;
                lowestSensor = i;
            }
        }
        // center robot
        //ToDo get how far to strafe correct

        if(lowestSensor == 0){
            strafeToPosition(-2,.3,sleepTime,.1);
        }
        if(lowestSensor == 1){
            strafeToPosition(-1,.3,sleepTime,.1);
        }
        if(lowestSensor == 2){
            strafeToPosition(0,.3,sleepTime,.1);
        }
        if(lowestSensor == 3){
            strafeToPosition(1,.3,sleepTime,.1);
        }
        if(lowestSensor == 4){
            strafeToPosition(2,.3,sleepTime,.1);
        }


        //move forward

        lowestDistance = sensorDistances[0].getDistance(DistanceUnit.INCH);
        lowestSensor = 0;
        for (int i = 1; i < sensorDistances.length; i++){
            double currentDistance = sensorDistances[i].getDistance(DistanceUnit.INCH);
            if (currentDistance < lowestDistance){
                lowestDistance = currentDistance;
                lowestSensor = i;
            }
        }
        telemetry.addData("lowestSensor", lowestSensor);

        if (junctionHeight == 4) {
            runToPosition(-(lowestDistance-distanceToJunctionMedium),.3,sleepTime,.1);

        } else if (junctionHeight == 5){
            runToPosition(-(lowestDistance-distanceToJunctionMedium),.3,sleepTime,.1);

        } else {
            runToPosition(-(lowestDistance-3),.3,sleepTime,.1);

        }


        //center robot
        lowestDistance = sensorDistances[0].getDistance(DistanceUnit.INCH);
        lowestSensor = 0;
        for (int i = 1; i < sensorDistances.length; i++){
            double currentDistance = sensorDistances[i].getDistance(DistanceUnit.INCH);
            if (currentDistance < lowestDistance){
                lowestDistance = currentDistance;
                lowestSensor = i;
            }
        }
        if(lowestSensor == 0){
            strafeToPosition(-2,.3,sleepTime,.1);
        }
        if(lowestSensor == 1){
            strafeToPosition(-1,.3,sleepTime,.1);
        }
        if(lowestSensor == 2){
            strafeToPosition(0,.3,sleepTime,.1);
        }
        if(lowestSensor == 3){
            strafeToPosition(1,.3,sleepTime,.1);
        }
        if(lowestSensor == 4){
            strafeToPosition(2,.3,sleepTime,.1);

        }




        //deliver cone

        if (junctionHeight == 4) {
            motorSlideL.setTargetPosition(slideDeliverMedium-50);
            motorSlideR.setTargetPosition(slideDeliverMedium-50);
            motorIntake.setPower(intakePowerDeliver);
            sleep(1000);
//            motorSlideL.setTargetPosition(slideDeliverMedium);
//            motorSlideR.setTargetPosition(slideDeliverMedium);
            motorIntake.setPower(0);
        }
        if (junctionHeight == 5) {
            motorSlideL.setTargetPosition(slideDeliverHigh-50);
            motorSlideR.setTargetPosition(slideDeliverHigh-50);
            motorIntake.setPower(intakePowerDeliver);
            sleep(1000);
//            motorSlideL.setTargetPosition(slideDeliverHigh);
//            motorSlideR.setTargetPosition(slideDeliverHigh);
            motorIntake.setPower(0);
        }





        //if (lowestDistance < furtherDistance) return;
        //if (lowestDistance < deliverDistance){

        //}
    }


    private void runToPosition(double targetInches, double power, int sleepTime, double tolerance) {
        int targetPosition = (int) (targetInches * COUNTS_PER_INCH);
        tolerance = tolerance * COUNTS_PER_INCH;
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
        int targetPositionL = (int) (targetInchesL * COUNTS_PER_INCH);
        int targetPositionR = (int) (targetInchesR * COUNTS_PER_INCH);
        tolerance = tolerance * COUNTS_PER_INCH;
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


    private void turnToAngle(int targetAngle, double turnPower, double turnPower2){
        turn(targetAngle, turnPower, turnPower2, 2, 4, 4);
    }


    private void turn(int targetAngle, double powerIn, double powerIn2, double tolerance, int targetReachedCountThreshold, int failSafeCountThreshold) {
        double rotate;

        int failSafeCount = 0;
        int targetReachedCount = 0;
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentAngle = angles.firstAngle + angleOffset;
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


//        // TODO fix this
//        boolean buttonPushed = false;
//
//        { //pickup cone
//            if (gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5) {
//                buttonPushed = true;
//
//            } else {
//                buttonPushed = false;
//
//            }
//
//            if (buttonPushed) {
//                slideTarget = slideTargetPickup;
//                intakePower = intakePowerPickup;
//            }
//            else if(buttonPushed == false && buttonPushedLast == true){
//                slideTarget = slidePickupHigh;
//                intakePower = intakePowerHold;
//            }
//
//            buttonPushedLast = buttonPushed;
//        }





        {
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakePower = intakePowerDeliver;
            } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakePower = intakePowerPickup;
            } else {
                intakePower = intakePowerHold;
            }
        }


        if (slideTarget > slideMax) {
            slideTarget = slideMax;
        } else if (slideTarget < 0) {
            slideTarget = 0;
        }

        if (armTarget < 0) {
            armTarget = 0;
        } else if (armTarget > armMax) {
            armTarget = armMax;
        }


        if (armTarget != armTargetOld) {
            motorArm.setTargetPosition(armTarget);
        }

        if (slideTarget != slideTargetOld) {
            setSlideTarget(slideTarget);
        }

        if (intakePower != intakePowerOld) {
            motorIntake.setPower(intakePower);
        }

        if (telemetryEnabled) {
            telemetry.addData("armTarget", armTarget);
            telemetry.addData("slideTarget", slideTarget);
        }


        if (telemetryEnabled) {
            telemetry.addData("motorArm current", motorArm.getCurrentPosition());
            if (bothSlideMotors) {telemetry.addData("motorSlideL current", motorSlideL.getCurrentPosition());}
            telemetry.addData("motorSlideR current", motorSlideR.getCurrentPosition());
            telemetry.addData("motorIntake current", motorIntake.getCurrentPosition());

            telemetry.addData("motorArm target", motorArm.getTargetPosition());
            if (bothSlideMotors) {telemetry.addData("motorSlideL target", motorSlideL.getTargetPosition());}
            telemetry.addData("motorSlideR target", motorSlideR.getTargetPosition());
            telemetry.addData("motorIntake target", motorIntake.getTargetPosition());
        }




    }


    private void setSlideTarget(int position) {
        if (bothSlideMotors) {motorSlideL.setTargetPosition(position);}
        motorSlideR.setTargetPosition(position);
        if (position <= slidePowerOff) {
            if (bothSlideMotors) {motorSlideL.setPower(0);}
            motorSlideR.setPower(0);
        }else {
            if (bothSlideMotors) {motorSlideL.setPower(slidePower);}
            motorSlideR.setPower(slidePower);
        }
    }
    //arm deliver high 1895
    //slide deliver high 470
    //arm high release 1935

    private void autoPickupCone() {
        int pickupTarget = (conesRemaining-1)*35+10;
            motorIntake.setPower(intakePowerPickup);
            if (bothSlideMotors) {
                motorSlideL.setTargetPosition(slidePickupTarget);
            }
            motorSlideR.setTargetPosition(slidePickupTarget);

            sleep(1000);

            if (bothSlideMotors) {
                motorSlideL.setTargetPosition(slidePickupHigh);
            }
            motorSlideR.setTargetPosition(slidePickupHigh);
            motorIntake.setPower(intakePowerHold);

            sleep(1000);

        conesRemaining = conesRemaining - 1;
    }

    private void auto1() {
        int sleepTime = 500;
        double tolerance = 0.5;
        double turnTolerance = 0.2;
        int failSafeCountThreshold = 4;
        int targetReachedCountThreshold = 3;
        double power = .3;
        double powerin2 = .17;

        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            strafeToPosition(24, .3, sleepTime, tolerance);
        } else {
            strafeToPosition(-24, .3, sleepTime, tolerance);
        }
        turn(0, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        //motorArm.setTargetPosition(armDeliverHigh);
        setSlideTarget(slideDeliverHigh);
        runToPosition(40, .3, sleepTime, tolerance);

        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            turn(90, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        }else{
            turn(-90, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        }
        //deliver cone
        sleep(800);
        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            strafeToPosition(12, .3, sleepTime, tolerance);
        }else{
            strafeToPosition(-12, .3, sleepTime, tolerance);
        }
        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            turn(90, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        }else{
            turn(-90, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        }
        runToPosition(48, .3, sleepTime, tolerance);
        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            turn(90, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        }else{
            turn(-90, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        }
        //pickup cone
        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            runToPositionLeftRight(-44, -58, .3, .3, sleepTime, tolerance);
        }else{
            runToPositionLeftRight(-58, -44, .3, .3, sleepTime, tolerance);
        }
        //deliver cone

        // if we didn't detect a parking spot, pick a good default
        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.DETECTING) {
            parkingPosition = DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.RIGHT;
        }
        // actually park!
        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.LEFT) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                runToPositionLeftRight(0, 14, .3, .3, sleepTime, tolerance);
                turn(90, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
                runToPositionLeftRight(44, 44, .3, .3, sleepTime, tolerance);
                //pickup cone
            }else{
                runToPositionLeftRight(14, 0, .3, .3, sleepTime, tolerance);
                turn(-180, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
            }



        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                runToPositionLeftRight(0, 14, .3, .3, sleepTime, tolerance);
                turn(90, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
            }else{
                runToPositionLeftRight(14, 0, .3, .3, sleepTime, tolerance);
                turn(-90, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
            }
            runToPositionLeftRight(20, 20, .3, .3, sleepTime, tolerance);
            turn(-180, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);

        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.RIGHT) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                runToPositionLeftRight(0, 14, .3, .3, sleepTime, tolerance);
                turn(-180, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
            }else{
                runToPositionLeftRight(14, 0, .3, .3, sleepTime, tolerance);
                turn(-90, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
                runToPositionLeftRight(44, 44, .3, .3, sleepTime, tolerance);
                //pickup cone
            }
        }

    }



    private void autoPark() {
        int sleepTime = 500;
        double tolerance = 0.2;
        double turnTolerance = 0.2;
        int failSafeCountThreshold = 4;
        int targetReachedCountThreshold = 3;
        double power = .3;
        double powerin2 = .17;

        runToPosition(2, .3, sleepTime, tolerance);
        turn(0, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);

        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            strafeToPosition(24, .3, sleepTime, tolerance);
        } else {
            strafeToPosition(-24, .3, sleepTime, tolerance);
        }
        turn(0, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);


        runToPosition(46, .3, sleepTime, tolerance);
        turn(0, .3, powerin2, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);

        // if we didn't detect a parking spot, pick a good default
        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.DETECTING) {
            parkingPosition = DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER;
        }

        // actually park!
        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.LEFT) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                 strafeToPosition(-48, 0.3, sleepTime, tolerance);
            }else{
                 strafeToPosition(48, 0.3, sleepTime, tolerance);
            }


        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                strafeToPosition(-24, 0.3, sleepTime, tolerance);

            }else{
                strafeToPosition(24, 0.3, sleepTime, tolerance);
            }


        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.RIGHT) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {

            }else{

            }
        }


        turn(0, 3, .1, 0.2, 4, 4);


    }



    private void autoDeliverPark() {
        int sleepTime = 500;
        double tolerance = 0.2;
        double turnTolerance = 0.2;
        int failSafeCountThreshold = 4;
        int targetReachedCountThreshold = 3;
        double powerTurnHigh = .3;
        double powerTurnLow = .17;
        double powerDriveHigh = .3;
        double powerDriveLow = .17;

        runToPosition(2, powerDriveHigh, sleepTime, tolerance);
        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);

        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            strafeToPosition(24, powerDriveHigh, sleepTime, tolerance);
        } else {
            strafeToPosition(-24, powerDriveHigh, sleepTime, tolerance);
        }
        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);


        runToPosition(35.5, powerDriveHigh, sleepTime, tolerance);


        motorArm.setTargetPosition(armDeliverMedium);
        motorSlideL.setTargetPosition(slideDeliverMedium);
        motorSlideR.setTargetPosition(slideDeliverMedium);
        motorIntake.setPower(intakePowerPickup);

        sleep(3000);

        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            turn(-90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        }else{
            turn(90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        }

        if (Spot(RED, AUDIENCE)) runToPosition(-6, powerDriveHigh, sleepTime, tolerance);
        if (Spot(RED, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, JUDGE)) runToPosition(-4, powerDriveHigh, sleepTime, tolerance);


        motorSlideL.setTargetPosition(slideDeliverMedium - 70);
        motorSlideR.setTargetPosition(slideDeliverMedium - 70);
        sleep(1000);

        motorIntake.setPower(intakePowerDeliver);
        sleep(1000);
        motorIntake.setPower(0);

        if (Spot(RED, AUDIENCE)) runToPosition(6, powerDriveHigh, sleepTime, tolerance);
        if (Spot(RED, JUDGE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, AUDIENCE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, JUDGE)) runToPosition(4, powerDriveHigh, sleepTime, tolerance);

        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);

        motorArm.setTargetPosition(0);
        motorSlideL.setTargetPosition(0);
        motorSlideR.setTargetPosition(0);

        sleep(1500);

        runToPosition(10.5, powerDriveHigh, sleepTime, tolerance);



        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);


        // if we didn't detect a parking spot, pick a good default
        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.DETECTING) {
            parkingPosition = DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER;
        }

        // actually park!
        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.LEFT) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                strafeToPosition(-48, powerDriveHigh, sleepTime, tolerance);
            }else{

            }


        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                strafeToPosition(-24, powerDriveHigh, sleepTime, tolerance);

            }else{
                strafeToPosition(24, powerDriveHigh, sleepTime, tolerance);
            }


        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.RIGHT) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {

            }else{
                strafeToPosition(48, powerDriveHigh, sleepTime, tolerance);
            }
        }


        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        runToPosition(-1, powerDriveHigh, sleepTime, tolerance);


        turn(0, powerTurnHigh, .1, 0.2, 4, 4);


    }



    private void autoTest() {

        double turnTolerance = 0.2;
        int failSafeCountThreshold = 4;
        int targetReachedCountThreshold = 3;
        double powerTurnHigh = .3;
        double powerTurnLow = .17;

        turn(90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);


        while (opModeIsActive()) {
            autoFollowLine(0.3,0,0.1,28,driveLF);

        }
    }



    private void autoFieldOriented(double fwd, double strafe, int targetAngle, double rotate) {
        double tempForward;
        double denominator;


        if (telemetryEnabled) {
            telemetry.addData("fwd", fwd);
            telemetry.addData("strafe", strafe);
        }

        if (fieldOrientated) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentAngle = angles.firstAngle + angleOffset;
            tempForward = fwd;
            fwd = (strafe * -Math.sin(currentAngle / 180 * Math.PI) + tempForward * Math.cos(currentAngle / 180 * Math.PI));
            strafe = -(1.35 * -(strafe * Math.cos(currentAngle / 180 * Math.PI) + tempForward * Math.sin(currentAngle / 180 * Math.PI)));

            rotate = rotate + teleopTurnToAngle(targetAngle, currentAngle, 0.6, 0.2);

        }

        if (telemetryEnabled) {
            telemetry.addData("field fwd", fwd);
            telemetry.addData("field strafe", strafe);
        }
        driveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        denominator = Math.max(Math.abs(fwd) + Math.abs(strafe) + Math.abs(rotate), 1.0);
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



    private void autoDeliverPark2() {
        int sleepTime = 500;
        double tolerance = 0.2;
        double turnTolerance = 0.2;
        int failSafeCountThreshold = 4;
        int targetReachedCountThreshold = 3;
        double powerTurnHigh = .3;
        double powerTurnLow = .17;
        double powerDriveHigh = .3;
        double powerDriveLow = .17;

        runToPosition(37.5, powerDriveHigh, sleepTime, tolerance);
        motorArm.setTargetPosition(armDeliverMedium);
        motorSlideL.setTargetPosition(slideDeliverMedium);
        motorSlideR.setTargetPosition(slideDeliverMedium);
        motorIntake.setPower(intakePowerPickup);
        sleep(3000);
        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            turn(90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        } else {
            turn(-90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        }








        if (Spot(RED, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(RED, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);


        motorSlideL.setTargetPosition(slideDeliverMedium - 70);
        motorSlideR.setTargetPosition(slideDeliverMedium - 70);
        sleep(1000);

        motorIntake.setPower(intakePowerDeliver);
        sleep(1000);
        motorIntake.setPower(0);

        if (Spot(RED, AUDIENCE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(RED, JUDGE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, AUDIENCE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, JUDGE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);

        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);

        motorArm.setTargetPosition(0);
        motorSlideL.setTargetPosition(0);
        motorSlideR.setTargetPosition(0);

        sleep(1500);

        runToPosition(10.5, powerDriveHigh, sleepTime, tolerance);



        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);


        // if we didn't detect a parking spot, pick a good default
        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.DETECTING) {
            parkingPosition = DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER;
        }

        // actually park!
        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.LEFT) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                strafeToPosition(-24, powerDriveHigh, sleepTime, tolerance);
            }else{
                strafeToPosition(24, powerDriveHigh, sleepTime, tolerance);
            }


        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                strafeToPosition(0, powerDriveHigh, sleepTime, tolerance);

            }else{
                strafeToPosition(0, powerDriveHigh, sleepTime, tolerance);
            }


        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.RIGHT) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                strafeToPosition(24, powerDriveHigh, sleepTime, tolerance);
            }else{
                strafeToPosition(-24, powerDriveHigh, sleepTime, tolerance);
            }
        }


        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        runToPosition(-1, powerDriveHigh, sleepTime, tolerance);


        turn(0, powerTurnHigh, .1, 0.2, 4, 4);


    }



    private void autoDeliverPark3() {
        int sleepTime = 500;
        double tolerance = 0.2;
        double turnTolerance = 0.2;
        int failSafeCountThreshold = 4;
        int targetReachedCountThreshold = 3;
        double powerTurnHigh = .3;
        double powerTurnLow = .17;
        double powerDriveHigh = .3;
        double powerDriveLow = .17;

        runToPosition(37.5, powerDriveHigh, sleepTime, tolerance);
        motorArm.setTargetPosition(armDeliverMedium);
        motorSlideL.setTargetPosition(slideDeliverMedium);
        motorSlideR.setTargetPosition(slideDeliverMedium);
        motorIntake.setPower(intakePowerHold);
        sleep(3000);
        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            turn(90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        } else {
            turn(-90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        }








//        if (Spot(RED, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(RED, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);



        autoJunctionDeliver(4);
        sleep(1000);




        if (Spot(RED, AUDIENCE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(RED, JUDGE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, AUDIENCE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, JUDGE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);

        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);

        motorArm.setTargetPosition(0);
        motorSlideL.setTargetPosition(0);
        motorSlideR.setTargetPosition(0);


        sleep(1500);

        runToPosition(10.5, powerDriveHigh, sleepTime, tolerance);



        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);


        // if we didn't detect a parking spot, pick a good default
        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.DETECTING) {
            parkingPosition = DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER;
        }

        // actually park!
        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.LEFT) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                strafeToPosition(-24, powerDriveHigh, sleepTime, tolerance);
            }else{
                strafeToPosition(24, powerDriveHigh, sleepTime, tolerance);
            }


        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                strafeToPosition(0, powerDriveHigh, sleepTime, tolerance);

            }else{
                strafeToPosition(0, powerDriveHigh, sleepTime, tolerance);
            }


        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.RIGHT) {
            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                strafeToPosition(24, powerDriveHigh, sleepTime, tolerance);
            }else{
                strafeToPosition(-24, powerDriveHigh, sleepTime, tolerance);
            }
        }


        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        runToPosition(-1, powerDriveHigh, sleepTime, tolerance);


        turn(0, powerTurnHigh, .1, 0.2, 4, 4);


    }




    private void autoDeliverPark4() {
        int sleepTime = 500;
        double tolerance = 0.2;
        double turnTolerance = 0.2;
        int failSafeCountThreshold = 4;
        int targetReachedCountThreshold = 3;
        double powerTurnHigh = .3;
        double powerTurnLow = .17;
        double powerDriveHigh = .3;
        double powerDriveLow = .17;

        runToPosition(39, powerDriveHigh, sleepTime, tolerance);
        motorArm.setTargetPosition(armDeliverMedium);
        motorSlideL.setTargetPosition(slideDeliverMedium);
        motorSlideR.setTargetPosition(slideDeliverMedium);
        motorIntake.setPower(intakePowerHold);
        sleep(3000);
        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
            turn(90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        } else {
            turn(-90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
        }



//        if (Spot(RED, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(RED, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);



        autoJunctionDeliver(4);

        motorArm.setTargetPosition(armPickupHigh);
        motorSlideL.setTargetPosition(slidePickupHigh);
        motorSlideR.setTargetPosition(slidePickupHigh);

        if (Spot(RED, AUDIENCE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(RED, JUDGE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, AUDIENCE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
        if (Spot(BLUE, JUDGE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);

        strafeToPosition(12,powerDriveHigh,0,0.5);

        autoFollowLine(powerDriveHigh,0,0.1,28,driveLF);

        while (opModeIsActive()) {
            autoPickupCone();


            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                runToPositionLeftRight(-20, -34, .3, .3, sleepTime, tolerance);
            } else{
                runToPositionLeftRight(-34, -20, .3, .3, sleepTime, tolerance);
            }


            motorArm.setTargetPosition(armDeliverMedium);
            motorSlideL.setTargetPosition(slideDeliverMedium);
            motorSlideR.setTargetPosition(slideDeliverMedium);

            autoJunctionDeliver(4);

            if (true) {
                break;
            }


            motorArm.setTargetPosition(armPickupHigh);
            motorSlideL.setTargetPosition(slidePickupHigh);
            motorSlideR.setTargetPosition(slidePickupHigh);



            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
                autoFollowLine(powerDriveHigh, powerDriveHigh * 0.3, 0.1, 34,driveLF);
            } else{
                autoFollowLine(powerDriveHigh, powerDriveHigh * 0.3, 0.1, 34,driveRF);
            }
        }
//
//        motorArm.setTargetPosition(0);
//        motorSlideL.setTargetPosition(0);
//        motorSlideR.setTargetPosition(0);
//
//        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//            runToPositionLeftRight(0, 14, .3, .3, sleepTime, tolerance);
//        } else{
//            runToPositionLeftRight(14, 0, .3, .3, sleepTime, tolerance);
//        }
//
//
//        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//            turn(90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//        } else{
//            turn(-90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//        }
//

        // if we didn't detect a parking spot, pick a good default
//        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.DETECTING) {
//            parkingPosition = DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER;
//        }
//
//        // actually park!
//        if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.LEFT) {
//            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//                strafeToPosition(-24, powerDriveHigh, sleepTime, tolerance);
//            }else{
//                strafeToPosition(24, powerDriveHigh, sleepTime, tolerance);
//            }
//
//
//        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.CENTER) {
//            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//                strafeToPosition(0, powerDriveHigh, sleepTime, tolerance);
//
//            }else{
//                strafeToPosition(0, powerDriveHigh, sleepTime, tolerance);
//            }
//
//
//        } else if (parkingPosition == DetectSignalSleeveSide.PowerPlayDeterminationPipeline.ParkingPosition.RIGHT) {
//            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//                strafeToPosition(24, powerDriveHigh, sleepTime, tolerance);
//            }else{
//                strafeToPosition(-24, powerDriveHigh, sleepTime, tolerance);
//            }
//        }
//
//
//        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//        runToPosition(-1, powerDriveHigh, sleepTime, tolerance);
//
//
//        turn(0, powerTurnHigh, .1, 0.2, 4, 4);


    }






    private void autoFollowLine(double speedTowardsCone, double speedSideways, double lineCorrectionPower, double targetInches, DcMotor driveSide) {
        double targetTics = targetInches * COUNTS_PER_INCH;

        float saturationLeft = getSaturation(sensorColorLeft);
        float saturationRight = getSaturation(sensorColorRight);

        driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(driveSide.getCurrentPosition() < targetTics){
            if (saturationLeft >= 0.6 && saturationRight >= 0.6) {
                autoFieldOriented(0.0, -speedTowardsCone, 90, 0);

            } else if (saturationLeft >= 0.6) {
                autoFieldOriented(-lineCorrectionPower, -speedTowardsCone, 90, 0);

            } else if (saturationRight >= 0.6) {
                autoFieldOriented(lineCorrectionPower, -speedTowardsCone, 90, 0);

            } else {
                autoFieldOriented(speedSideways, -speedTowardsCone, 90, 0);
            }

            if (telemetryEnabled) {
                telemetry.addData("Motor Front Left", driveLF.getCurrentPosition());
                telemetry.addData("Motor Front Right", driveRF.getCurrentPosition());
                telemetry.addData("Motor Back Right", driveRB.getCurrentPosition());
                telemetry.addData("Motor Back Left", driveLB.getCurrentPosition());
                telemetry.update();
            }
        }
    }


    private float getSaturation(NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (telemetryEnabled) {
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);


        }

        return hsvValues[1];

    }





    private void autoCenterRobot(int sleepTime) {
        double lowestDistance = sensorDistances[0].getDistance(DistanceUnit.INCH);
        int lowestSensor = 0;
        for (int i = 1; i < sensorDistances.length; i++){
            double currentDistance = sensorDistances[i].getDistance(DistanceUnit.INCH);
            if (currentDistance < lowestDistance){
                lowestDistance = currentDistance;
                lowestSensor = i;
            }
        }
        // center robot
        //ToDo get how far to strafe correct

        if(lowestSensor == 0){
            strafeToPosition(-2,.3,sleepTime,.1);
        }
        if(lowestSensor == 1){
            strafeToPosition(-1,.3,sleepTime,.1);
        }
        if(lowestSensor == 2){
            strafeToPosition(0,.3,sleepTime,.1);
        }
        if(lowestSensor == 3){
            strafeToPosition(1,.3,sleepTime,.1);
        }
        if(lowestSensor == 4){
            strafeToPosition(2,.3,sleepTime,.1);
        }
    }




    private void autoLoop(){

    }



}


