//package org.firstinspires.ftc.teamcode;
//
//import android.graphics.Color;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import java.util.List;
//import org.firstinspires.ftc.robotcore.external.JavaUtil;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
//
//@Autonomous(name = "ExtraFreightAuto (Blocks to Java)")
//@Disabled
//public class ExtraFreightAuto extends LinearOpMode {
//
//  private CRServo servoCap;
//  private Servo servoString;
//  private DcMotor motorCarousel;
//  private Tfod tfod;
//  private VuforiaCurrentGame vuforiaFreightFrenzy;
//  private Servo servoBucket;
//  private BNO055IMU imu;
//  private DistanceSensor sensorDistanceHub;
//  private DistanceSensor sensorDistanceIntake;
//  private ColorSensor sensorColorLine;
//  private DcMotor motorArm;
//  private DcMotor motorIntake;
//  private DcMotor motorElevator;
//  private DcMotor driveLF;
//  private DcMotor driveLB;
//  private DcMotor driveRF;
//  private DcMotor driveRB;
//
//  double power;
//  double targetPositionL;
//  double targetPositionR;
//  double targetInches;
//  int targetPosition;
//  float currentAngle;
//  boolean teleop;
//  boolean fieldOrientated;
//  String armTarget;
//  double intakePower;
//  double carouselSpeed;
//  float angleDifference;
//  double rotateOut;
//  boolean parabolicDriving;
//  ElapsedTime timerIntake;
//  boolean telemetryEnabled;
//  double servoBucketDown;
//  int carouselLastPosition;
//  boolean lineDetected;
//  int loopNumber;
//  ElapsedTime pickUpFailSafe;
//  int duckLocation;
//  int elevatorPos1;
//  int elevatorPos2;
//  int elevatorPos3;
//  double servoBucketUp;
//  String startColor;
//  double hubDistance;
//  long carouselLastTime;
//  int elevatorPosDown;
//  Orientation angles;
//  Recognition recognition;
//  boolean alreadyBackingUp;
//  double stringIn;
//  int stringOut;
//  double stringRelease;
//  int currentPositionLF;
//  int elevTarget;
//  BNO055IMU.Parameters imuParameters;
//  int currentPositionLB;
//  String stoppingSpot;
//  String startCOrW;
//  int currentPositionRF;
//  int currentPositionRB;
//  int elevatorPos15;
//  int positionLF;
//  double intakeThreshold;
//  ElapsedTime backingUpFailSafe;
//  int positionRF;
//  int whiteLineThreshold;
//  int positionLB;
//  int positionRB;
//  float initialAngle;
//  int carouselDuckVelocity;
//  boolean handledThisX;
//  boolean carouselOn;
//
//  /**
//   * This function is executed when this Op Mode is selected from the Driver Station.
//   */
//  @Override
//  public void runOpMode() {
//    boolean elevatorEnabled;
//    Acceleration gravity;
//    int sensorLineARGB;
//
//    servoCap = hardwareMap.get(CRServo.class, "servoCap");
//    servoString = hardwareMap.get(Servo.class, "servoString");
//    motorCarousel = hardwareMap.get(DcMotor.class, "motorCarousel");
//    tfod = new Tfod();
//    vuforiaFreightFrenzy = new VuforiaCurrentGame();
//    servoBucket = hardwareMap.get(Servo.class, "servoBucket");
//    imu = hardwareMap.get(BNO055IMU.class, "imu");
//    sensorDistanceHub = hardwareMap.get(DistanceSensor.class, "sensorDistanceHub");
//    sensorDistanceIntake = hardwareMap.get(DistanceSensor.class, "sensorDistanceIntake");
//    sensorColorLine = hardwareMap.get(ColorSensor.class, "sensorColorLine");
//    motorArm = hardwareMap.get(DcMotor.class, "motorArm");
//    motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
//    motorElevator = hardwareMap.get(DcMotor.class, "motorElevator");
//    driveLF = hardwareMap.get(DcMotor.class, "left_driveF");
//    driveLB = hardwareMap.get(DcMotor.class, "left_driveB");
//    driveRF = hardwareMap.get(DcMotor.class, "right_driveF");
//    driveRB = hardwareMap.get(DcMotor.class, "right_driveB");
//
//    teleop = false;
//    fieldOrientated = teleop;
//    parabolicDriving = teleop;
//    // choose red or blue and carousel or warehouse
//    telemetryEnabled = false;
//    servoCap.setDirection(DcMotorSimple.Direction.FORWARD);
//    servoString.setDirection(Servo.Direction.REVERSE);
//    stringOut = 1;
//    stringIn = 0.66;
//    stringRelease = 0.5;
//    if (teleop) {
//      // Read a file (custom block).
//      startColor = Files.readFile("startColor");
//    } else {
//      startColor = "red";
//    }
//    startCOrW = "warehouse";
//    stoppingSpot = "sharedEntrance";
//    elevTarget = 1;
//    intakeThreshold = 1.95;
//    whiteLineThreshold = 120;
//    if (teleop) {
//      carouselDuckVelocity = 850;
//    } else {
//      carouselDuckVelocity = 550;
//    }
//    motorCarousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    motorCarousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    carouselLastTime = 0;
//    carouselLastPosition = 0;
//    rotateOut = 0;
//    // servo
//    servoBucketUp = 0.6;
//    servoBucketDown = 0.98;
//    elevatorEnabled = true;
//    if (elevatorEnabled) {
//      if (teleop == false) {
//        elevatorPos1 = 60;
//        elevatorPos2 = 540;
//        elevatorPos3 = 1000;
//        elevatorPosDown = 0;
//      } else {
//        elevatorPos1 = 5;
//        elevatorPos15 = 200;
//        elevatorPos2 = 500;
//        elevatorPos3 = 1000;
//        elevatorPosDown = 0;
//      }
//    } else {
//      elevatorPos1 = 0;
//      elevatorPos15 = 0;
//      elevatorPos2 = 0;
//      elevatorPos3 = 0;
//      elevatorPosDown = 0;
//    }
//    oneTimeInit();
//    initLoop();
//    // Wait for start command from Driver Station.
//    if (teleop == false) {
//      // Save to a file (custom block).
//      if (Files.saveFile("startColor", startColor)) {
//      } else if (false) {
//      }
//    }
//    if (teleop == false) {
//      tfod.deactivate();
//      vuforiaFreightFrenzy.deactivate();
//    }
//    waitForStart();
//    if (opModeIsActive()) {
//      // Put run blocks here.
//      servoBucket.setPosition(servoBucketUp);
//      if (teleop == false) {
//        if (startCOrW.equals("carousel")) {
//          AutoCarousel3(0.25, 0.3, 0.2, 0);
//        } else if (startCOrW.equals("warehouse")) {
//          autoWarehouse6(0.4, 0.4, 0.15, 0);
//        }
//      }
//      if (teleop == true) {
//        while (opModeIsActive()) {
//          if (gamepad2.right_stick_button && gamepad2.left_stick_button) {
//            telemetryEnabled = !telemetryEnabled;
//          }
//          // Get absolute orientation
//          angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//          // Get acceleration due to force of gravity.
//          gravity = imu.getGravity();
//          // Display orientation info.
//          if (telemetryEnabled) {
//            telemetry.addData("rot about Z", angles.firstAngle);
//            telemetry.addData("rot about Y", angles.secondAngle);
//            telemetry.addData("rot about X", angles.thirdAngle);
//          }
//          // Display gravitational acceleration.
//          // Enter your comment here!
//          gamepadDriveMotors();
//          if (telemetryEnabled) {
//            hubDistance = sensorDistanceHub.getDistance(DistanceUnit.INCH);
//            telemetry.addData("sensorHub distance", hubDistance);
//            telemetry.addData("sensorIntake distance", sensorDistanceIntake.getDistance(DistanceUnit.INCH));
//            telemetry.addData("sensorLine Argb", sensorColorLine.argb());
//            sensorLineARGB = sensorColorLine.argb();
//            telemetry.addData("sensorLine red", Color.red(sensorLineARGB));
//            telemetry.addData("sensorLine green", Color.green(sensorLineARGB));
//            telemetry.addData("sensorLine blue", Color.blue(sensorLineARGB));
//            telemetry.addData("sensorLine alpha", Color.alpha(sensorLineARGB));
//            telemetry.addData("red", sensorColorLine.red());
//            telemetry.addData("green", sensorColorLine.green());
//            telemetry.addData("blue", sensorColorLine.blue());
//            telemetry.addData("alpha", sensorColorLine.alpha());
//            // updateDriverStation
//            telemetry.update();
//          }
//        }
//      }
//    }
//    if (teleop == false) {
//      // Deactivate TFOD.
//    }
//
//    tfod.close();
//    vuforiaFreightFrenzy.close();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void pickUpAllianceTeleop(double power, double turnPower, double turnPower2, int sleep2) {
//    fieldOrientated = false;
//    parabolicDriving = false;
//    lineDetected = false;
//    timerIntake.reset();
//    if (gamepad1.left_bumper) {
//      if (startColor.equals("red")) {
//        driveMotors(0.35, 0.2, 0);
//      } else if (startColor.equals("blue")) {
//        driveMotors(0.35, -0.2, 0);
//      }
//    }
//    pickUpFailSafe = new ElapsedTime(System.nanoTime());
//    if (gamepad1.left_bumper) {
//      armTarget = "out";
//    }
//    while (gamepad1.left_bumper) {
//      if (motorArm.getCurrentPosition() >= 80 && sensorDistanceIntake.getDistance(DistanceUnit.INCH) <= intakeThreshold || alreadyBackingUp == false && pickUpFailSafe.seconds() >= 4) {
//        armTarget = "in";
//        if (startColor.equals("red")) {
//          turn(-90, 0.4, 0.2, 5, 1, 1);
//          driveMotors(-0.2, 0.25, 0);
//        } else if (startColor.equals("blue")) {
//          turn(90, 0.4, 0.2, 5, 1, 1);
//          driveMotors(-0.2, -0.25, 0);
//        }
//      }
//      if (armTarget.equals("out")) {
//        timerIntake.reset();
//        if (motorArm.getCurrentPosition() > 80) {
//          motorArm.setPower(0.1);
//          intakePower = 1;
//        } else {
//          motorArm.setPower(0.5);
//          intakePower = 0;
//        }
//      } else {
//        // arm target == in
//        if (motorArm.getCurrentPosition() < 10) {
//          if (timerIntake.seconds() <= 2) {
//            intakePower = -1;
//            motorArm.setPower(-0.1);
//          } else {
//            intakePower = 0;
//            motorArm.setPower(0);
//          }
//        } else if (motorArm.getCurrentPosition() < 60) {
//          timerIntake.reset();
//          motorArm.setPower(-0.5);
//          intakePower = 0.5;
//        } else {
//          timerIntake.reset();
//          motorArm.setPower(-0.8);
//          intakePower = 1;
//        }
//      }
//      motorIntake.setPower(intakePower);
//      if (sensorColorLine.alpha() > whiteLineThreshold) {
//        if (armTarget.equals("in")) {
//          lineDetected = true;
//          break;
//        } else {
//          lineDetected = false;
//          driveMotors(0.2, 0, 0);
//        }
//      }
//    }
//    fieldOrientated = true;
//    parabolicDriving = true;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private double adjustForVoltage(double power) {
//    return power;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void deliverTeleopAlliance(double power, double turnPower, double turnPower2, int sleep2) {
//    armTarget = "in";
//    fieldOrientated = false;
//    parabolicDriving = false;
//    motorIntake.setPower(-1);
//    if (startColor.equals("red")) {
//      driveMotors(-0.4, 0.15, 0);
//    } else if (startColor.equals("blue")) {
//      driveMotors(-0.4, -0.15, 0);
//    }
//    if (!lineDetected) {
//      while (gamepad1.left_bumper) {
//        if (sensorColorLine.alpha() > whiteLineThreshold) {
//          break;
//        }
//      }
//    }
//    if (gamepad1.left_bumper) {
//      motorElevator.setTargetPosition(elevatorPos3);
//      elevTarget = 3;
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      motorIntake.setPower(0);
//    }
//    if (gamepad1.left_bumper) {
//      if (startColor.equals("red")) {
//        runToPositionLeftRight(-12, -35, 0.35, 0.35, 0, 0.5, false);
//      } else if (startColor.equals("blue")) {
//        runToPositionLeftRight(-35, -15, 0.35, 0.35, 0, 0.5, false);
//      }
//      runToPositionLeftRight(-23, -23, 0.35, 0.35, 0, 1, false);
//    }
//    if (gamepad1.left_bumper) {
//      toWarehouseAllianceTeleopContinueous(power, turnPower, turnPower2, sleep2);
//    }
//    fieldOrientated = true;
//    parabolicDriving = true;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void autoWarehouse6(double power, double turnPower, double turnPower2, int sleep2) {
//    initialDelivery(power, turnPower, turnPower2, sleep2);
//    allianceHubToWarehouse(power, turnPower, turnPower2, sleep2);
//    loopNumber = 0;
//    for (int count = 0; count < 2; count++) {
//      loopNumber = loopNumber + 1;
//      if (!pickUpFreight(power, turnPower, turnPower2, sleep2)) {
//        return;
//      }
//      deliverAdditionalFreight(power, turnPower, turnPower2, sleep2);
//      allianceHubToWarehouse(power, turnPower, turnPower2, sleep2);
//    }
//    if (stoppingSpot.equals("sharedEntrance")) {
//      runToPosition2(25, power, sleep2, 0.5, false);
//      warehouseSharedEntrance(power, sleep2, turnPower, turnPower2);
//    } else {
//      runToPosition2(27, power, sleep2, 0.5, false);
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void toWarehouseAllianceTeleopContinueous(double power, double turnPower, double turnPower2, int sleep2) {
//    fieldOrientated = false;
//    parabolicDriving = false;
//    runToPosition2(22, power, sleep2, 2, false);
//    servoBucket.setPosition(servoBucketUp);
//    motorElevator.setTargetPosition(elevatorPosDown);
//    elevTarget = 1;
//    if (startColor.equals("red")) {
//      turn(-90, turnPower, turnPower2, 10, 1, 1);
//    } else if (startColor.equals("blue")) {
//      turn(90, turnPower, turnPower2, 10, 1, 1);
//    }
//    if (startColor.equals("red")) {
//      driveMotors(0, 0.35, 0);
//    } else if (startColor.equals("blue")) {
//      driveMotors(0, -0.35, 0);
//    }
//    sleep(1000);
//    fieldOrientated = true;
//    parabolicDriving = true;
//    pickUpAllianceTeleop(power, turnPower, turnPower2, sleep2);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void runToPositionInitLeftRight(int targetPositionL, int targetPositionR, double powerL, double powerR, int sleep2) {
//    driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveLF.setPower(powerL);
//    driveRF.setPower(powerR);
//    driveLB.setPower(powerL);
//    driveRB.setPower(powerR);
//    driveLF.setTargetPosition((int) targetPositionL);
//    driveRF.setTargetPosition((int) targetPositionR);
//    driveLB.setTargetPosition((int) targetPositionL);
//    driveRB.setTargetPosition((int) targetPositionR);
//    driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private boolean pickUpFreight(double power, double turnPower, double turnPower2, int sleep2) {
//    int pickUpFailSafeTimeout;
//
//    timerIntake.reset();
//    if (startColor.equals("red")) {
//      driveMotors(0.35, 0.2, 0);
//    } else if (startColor.equals("blue")) {
//      driveMotors(0.35, -0.2, 0);
//    }
//    pickUpFailSafe = new ElapsedTime(System.nanoTime());
//    armTarget = "out";
//    alreadyBackingUp = false;
//    if (loopNumber == 1) {
//      pickUpFailSafeTimeout = 3;
//    } else {
//      pickUpFailSafeTimeout = 4;
//    }
//    while (true) {
//      if (motorArm.getCurrentPosition() >= 80 && sensorDistanceIntake.getDistance(DistanceUnit.INCH) <= intakeThreshold || alreadyBackingUp == false && pickUpFailSafe.seconds() >= pickUpFailSafeTimeout) {
//        armTarget = "in";
//        if (loopNumber == 2) {
//          if (startColor.equals("red")) {
//            driveMotors(-0.4, 0, 0);
//            sleep(100);
//            turn(-90, turnPower, turnPower2, 10, 2, 2);
//            driveMotors(0, 0.4, 0);
//            sleep(100);
//          } else if (startColor.equals("blue")) {
//            driveMotors(-0.4, 0, 0);
//            sleep(100);
//            turn(90, turnPower, turnPower2, 10, 2, 2);
//            driveMotors(0, -0.4, 0);
//            sleep(100);
//          }
//        }
//        if (startColor.equals("red")) {
//          driveMotors(-0.2, 0.15, 0);
//        } else if (startColor.equals("blue")) {
//          driveMotors(-0.2, -0.15, 0);
//        }
//        if (alreadyBackingUp == false) {
//          backingUpFailSafe = new ElapsedTime(System.nanoTime());
//        }
//        alreadyBackingUp = true;
//      }
//      if (armTarget.equals("out")) {
//        timerIntake.reset();
//        if (motorArm.getCurrentPosition() > 80) {
//          motorArm.setPower(0.1);
//          intakePower = 1;
//        } else {
//          motorArm.setPower(0.5);
//          intakePower = 0;
//        }
//      } else {
//        // arm target == in
//        if (motorArm.getCurrentPosition() < 10) {
//          if (timerIntake.seconds() <= 2) {
//            intakePower = -1;
//            motorArm.setPower(-0.1);
//          } else {
//            intakePower = 0;
//            motorArm.setPower(0);
//          }
//        } else if (motorArm.getCurrentPosition() < 60) {
//          timerIntake.reset();
//          motorArm.setPower(-0.5);
//          intakePower = 0.5;
//        } else {
//          timerIntake.reset();
//          motorArm.setPower(-0.8);
//          intakePower = 1;
//        }
//      }
//      motorIntake.setPower(intakePower);
//      if (sensorColorLine.alpha() > whiteLineThreshold) {
//        if (armTarget.equals("in")) {
//          break;
//        } else {
//          if (startColor.equals("red")) {
//            if (loopNumber == 1) {
//              driveMotors(0.2, 0, 0);
//            } else if (loopNumber == 2) {
//              turn(-70, turnPower, 0.25, 3, 2, 2);
//              driveMotors(0.2, 0, 0);
//            }
//          } else if (startColor.equals("blue")) {
//            if (loopNumber == 1) {
//              driveMotors(0.2, 0, 0);
//            } else if (loopNumber == 2) {
//              turn(70, turnPower, 0.25, 3, 2, 2);
//              driveMotors(0.2, 0, 0);
//            }
//          }
//        }
//      }
//      if (alreadyBackingUp == true && backingUpFailSafe.seconds() >= 3) {
//        runToPosition2(12, power, sleep2, 1, false);
//        if (stoppingSpot.equals("sharedEntrance")) {
//          warehouseSharedEntrance(power, sleep2, turnPower, turnPower2);
//        }
//        if (true) {
//          return false;
//        }
//      }
//    }
//    return true;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void allianceHubToWarehouse(double power, double turnPower, double turnPower2, int sleep2) {
//    runToPosition2(22, power, sleep2, 2, false);
//    servoBucket.setPosition(servoBucketUp);
//    motorElevator.setTargetPosition(elevatorPosDown);
//    if (startColor.equals("red")) {
//      turn(-90, turnPower, turnPower2, 10, 1, 1);
//    } else if (startColor.equals("blue")) {
//      turn(90, turnPower, turnPower2, 10, 1, 1);
//    }
//    if (startColor.equals("red")) {
//      driveMotors(0, 0.35, 0);
//    } else if (startColor.equals("blue")) {
//      driveMotors(0, -0.35, 0);
//    }
//    sleep(1000);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void gamepadDriveMotors() {
//    double fwdSign;
//    double strafeSign;
//    double dpadFwd;
//    double dpadStrafe;
//
//    // capping
//    if (Math.abs(gamepad2.left_stick_y) > 0.05) {
//      servoCap.setPower(gamepad2.left_stick_y);
//    } else {
//      servoCap.setPower(0);
//    }
//    if (gamepad2.right_stick_y > 0.9) {
//      servoString.setPosition(stringIn);
//    } else if (gamepad2.right_stick_y < -0.9) {
//      servoString.setPosition(stringOut);
//    } else if (gamepad2.right_trigger > 0.9) {
//      servoString.setPosition(stringRelease);
//    }
//    // gamepad 1
//    if (gamepad1.left_trigger > 0.1) {
//      servoBucket.setPosition(servoBucketUp);
//    }
//    if (gamepad1.right_trigger > 0.1) {
//      servoBucket.setPosition(servoBucketDown);
//    }
//    // gamepad 2
//    if (gamepad2.left_trigger > 0.1) {
//      servoBucket.setPosition(servoBucketUp);
//    }
//    // carousel
//    carouselSpeed = setCarouselSpeed();
//    if (gamepad1.back || gamepad2.back) {
//      // red side
//      startColor = "red";
//      ((DcMotorEx) motorCarousel).setVelocity(-carouselDuckVelocity);
//    } else if (gamepad1.start || gamepad2.start) {
//      // blue side
//      startColor = "blue";
//      ((DcMotorEx) motorCarousel).setVelocity(carouselDuckVelocity);
//    } else {
//      // old system, onlyuse if coach places duck
//      ((DcMotorEx) motorCarousel).setVelocity(0);
//    }
//    if (telemetryEnabled) {
//      telemetry.addData("carouselPower", carouselDuckVelocity);
//    }
//    if (gamepad1.left_bumper) {
//      lineDetected = false;
//      deliverTeleopAlliance(0.4, 0.4, 0.15, 0);
//    } else if (gamepad1.right_bumper) {
//      lineDetected = false;
//      deliverTeleopShared(0.4, 0.4, 0.15, 0);
//    }
//    // elevator
//    if (gamepad2.y) {
//      servoBucket.setPosition(servoBucketUp);
//      elevTarget = 3;
//    } else if (gamepad2.b) {
//      servoBucket.setPosition(servoBucketUp);
//      elevTarget = 2;
//    } else if (gamepad2.a) {
//      servoBucket.setPosition(servoBucketUp);
//      elevTarget = 1;
//    } else if (gamepad2.x) {
//      elevTarget = 15;
//    }
//    if (elevTarget == 1) {
//      motorElevator.setTargetPosition(elevatorPos1);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    } else if (elevTarget == 2) {
//      motorElevator.setTargetPosition(elevatorPos2);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    } else if (elevTarget == 3) {
//      motorElevator.setTargetPosition(elevatorPos3);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    } else if (elevTarget == 15) {
//      motorElevator.setTargetPosition(elevatorPos15);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    if (telemetryEnabled) {
//      telemetry.addData("elevTarget ", elevTarget);
//      telemetry.addData("elevCurrent", motorElevator.getCurrentPosition());
//    }
//    if (motorArm.getCurrentPosition() >= 80 && sensorDistanceIntake.getDistance(DistanceUnit.INCH) <= intakeThreshold) {
//      armTarget = "in";
//    }
//    // ^can be overidden by manual
//    // up = out
//    if (gamepad2.dpad_up) {
//      armTarget = "out";
//    }
//    // down = in
//    if (gamepad2.dpad_down) {
//      armTarget = "in";
//    }
//    // move arm
//    if (armTarget.equals("out")) {
//      timerIntake.reset();
//      if (motorArm.getCurrentPosition() > 80) {
//        motorArm.setPower(0.1);
//        intakePower = 1;
//      } else {
//        motorArm.setPower(0.5);
//        intakePower = 0;
//      }
//    } else {
//      // arm target == in
//      if (motorArm.getCurrentPosition() < 10) {
//        if (timerIntake.seconds() <= 2) {
//          intakePower = -1;
//          motorArm.setPower(-0.1);
//        } else {
//          intakePower = 0;
//          motorArm.setPower(0);
//        }
//      } else if (motorArm.getCurrentPosition() < 60) {
//        timerIntake.reset();
//        motorArm.setPower(-0.5);
//        intakePower = 0.5;
//      } else {
//        timerIntake.reset();
//        motorArm.setPower(-0.8);
//        intakePower = 1;
//      }
//    }
//    fwdSign = 0;
//    strafeSign = 0;
//    // gamepad 1 joysticks and slow mode
//    dpadFwd = 0.25;
//    dpadStrafe = 0.25;
//    if (gamepad1.dpad_up) {
//      if (gamepad1.dpad_right) {
//        driveMotors(-(dpadFwd / 1.4), -(dpadStrafe / 1.4), gamepad1.right_stick_x / 3);
//      } else if (gamepad1.dpad_left) {
//        driveMotors(-(dpadFwd / 1.4), dpadStrafe / 1.4, gamepad1.right_stick_x / 3);
//      } else {
//        driveMotors(-dpadFwd, 0, gamepad1.right_stick_x / 3);
//      }
//    } else if (gamepad1.dpad_down) {
//      if (gamepad1.dpad_right) {
//        driveMotors(dpadFwd / 1.4, -(dpadStrafe / 1.4), gamepad1.right_stick_x / 3);
//      } else if (gamepad1.dpad_left) {
//        driveMotors(dpadFwd / 1.4, dpadStrafe / 1.4, gamepad1.right_stick_x / 3);
//      } else {
//        driveMotors(dpadFwd, 0, gamepad1.right_stick_x / 3);
//      }
//    } else if (gamepad1.dpad_left) {
//      driveMotors(0, dpadStrafe, gamepad1.right_stick_x / 3);
//    } else if (gamepad1.dpad_right) {
//      driveMotors(0, -dpadStrafe, gamepad1.right_stick_x / 3);
//    } else {
//      if (parabolicDriving) {
//        driveMotors(0.6 * parabolicTransform(gamepad1.left_stick_y), 0.6 * -parabolicTransform(gamepad1.left_stick_x), gamepad1.right_stick_x / 3);
//      } else {
//        driveMotors(fwdSign + gamepad1.left_stick_y * 0.5, -(strafeSign + gamepad1.left_stick_x * 0.5), gamepad1.right_stick_x / 4);
//      }
//    }
//    // manual overide
//    // Left Bumper
//    if (gamepad2.left_bumper) {
//      intakePower = -1;
//    }
//    // Right Bumper
//    if (gamepad2.right_bumper) {
//      intakePower = 1;
//    }
//    // left joystick
//    motorIntake.setPower(intakePower);
//    if (telemetryEnabled) {
//      telemetry.addData("GP1 LeftX", gamepad1.left_stick_y);
//      telemetry.addData("GP1 LeftY", gamepad1.left_stick_y);
//      telemetry.addData("GP1 RightX", gamepad1.right_stick_x);
//      //
//      telemetry.addData("Elevator Position", motorElevator.getCurrentPosition());
//      telemetry.addData("servo position", servoBucket.getPosition());
//      telemetry.addData("motor arm current position", motorArm.getCurrentPosition());
//      telemetry.addData("motor arm target position", motorArm.getTargetPosition());
//      telemetry.addData("motor intake current position", motorIntake.getCurrentPosition());
//      telemetry.addData("motor intake target position", motorIntake.getTargetPosition());
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void initialDelivery(double power, double turnPower, double turnPower2, int sleep2) {
//    if (duckLocation == 1) {
//      motorElevator.setTargetPosition(elevatorPos1);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    } else if (duckLocation == 2) {
//      motorElevator.setTargetPosition(elevatorPos2);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    } else if (duckLocation == 3) {
//      motorElevator.setTargetPosition(elevatorPos3);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    if (startColor.equals("red")) {
//      driveMotors(0.25, -0.3, 0);
//      sleep(250);
//      turnToAngle(-155, turnPower, turnPower2, sleep2);
//      runToPosition2(-19, power, sleep2, 0.5, true);
//    } else if (startColor.equals("blue")) {
//      driveMotors(0.25, 0.3, 0);
//      sleep(250);
//      turnToAngle(148, turnPower, turnPower2, sleep2);
//      runToPosition2(-21, power, sleep2, 0.5, true);
//    }
//    servoBucket.setPosition(servoBucketDown);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void deliverTeleopShared(double power, double turnPower, double turnPower2, int sleep2) {
//    armTarget = "in";
//    fieldOrientated = false;
//    parabolicDriving = false;
//    motorIntake.setPower(-1);
//    if (startColor.equals("red")) {
//      driveMotors(-0.4, -0.15, 0);
//    } else if (startColor.equals("blue")) {
//      driveMotors(-0.4, 0.15, 0);
//    }
//    if (!lineDetected) {
//      while (gamepad1.right_bumper) {
//        if (sensorColorLine.alpha() > whiteLineThreshold) {
//          break;
//        }
//      }
//    }
//    if (gamepad1.right_bumper) {
//      motorElevator.setTargetPosition(elevatorPos15);
//      elevTarget = 15;
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      motorIntake.setPower(0);
//    }
//    if (gamepad1.right_bumper) {
//      if (startColor.equals("red")) {
//        runToPositionLeftRight(-30, -10, 0.35, 0.35, 0, 0.5, false);
//      } else if (startColor.equals("blue")) {
//        runToPositionLeftRight(-8, -28, 0.35, 0.35, 0, 0.5, false);
//      }
//    }
//    fieldOrientated = true;
//    parabolicDriving = true;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void deliverAdditionalFreight(double power, double turnPower, double turnPower2, int sleep2) {
//    motorIntake.setPower(0);
//    motorElevator.setTargetPosition(elevatorPos3);
//    motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    if (startColor.equals("red")) {
//      runToPositionLeftRight(-12, -35, 0.35, 0.35, 0, 0.5, false);
//    } else if (startColor.equals("blue")) {
//      runToPositionLeftRight(-35, -15, 0.35, 0.35, 0, 0.5, false);
//    }
//    runToPositionLeftRight(-23, -23, 0.35, 0.35, 0, 1, true);
//    servoBucket.setPosition(servoBucketDown);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void driveMotors(double fwd, double strafe, float rotate) {
//    double tempForward;
//    double denominator;
//
//    if (telemetryEnabled) {
//      telemetry.addData("fwd", fwd);
//      telemetry.addData("strafe", strafe);
//    }
//    if (teleop) {
//      if (fieldOrientated) {
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        currentAngle = angles.firstAngle;
//        tempForward = fwd;
//        fwd = -(strafe * -Math.sin(currentAngle / 180 * Math.PI) + tempForward * Math.cos(currentAngle / 180 * Math.PI));
//        strafe = 1.35 * -(strafe * Math.cos(currentAngle / 180 * Math.PI) + tempForward * Math.sin(currentAngle / 180 * Math.PI));
//        if (gamepad1.a) {
//          // green
//          rotate = rotate + teleopTurnToAngle(180, currentAngle, 0.4, 0.1);
//        } else if (gamepad1.b) {
//          // red
//          rotate = rotate + teleopTurnToAngle(-90, currentAngle, 0.4, 0.1);
//        } else if (gamepad1.x) {
//          // blue
//          rotate = rotate + teleopTurnToAngle(90, currentAngle, 0.4, 0.1);
//        } else if (gamepad1.y) {
//          // yellow
//          rotate = rotate + teleopTurnToAngle(0, currentAngle, 0.4, 0.1);
//        }
//      }
//    }
//    if (telemetryEnabled) {
//      telemetry.addData("field fwd", fwd);
//      telemetry.addData("field strafe", strafe);
//    }
//    driveLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    driveLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    driveRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    driveRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    denominator = max(Math.abs(fwd) + Math.abs(strafe) + Math.abs(rotate), 1.000001);
//    if (telemetryEnabled) {
//      telemetry.addData("denominator", denominator);
//    }
//    driveLF.setPower(adjustForVoltage((fwd + strafe + rotate) / denominator));
//    driveLB.setPower(adjustForVoltage(((fwd - strafe) + rotate) / denominator));
//    driveRF.setPower(adjustForVoltage(((fwd - strafe) - rotate) / denominator));
//    driveRB.setPower(adjustForVoltage(((fwd + strafe) - rotate) / denominator));
//    if (telemetryEnabled) {
//      telemetry.addData("left_driveF", (fwd + strafe + rotate) / denominator);
//      telemetry.addData("left_driveB", ((fwd - strafe) + rotate) / denominator);
//      telemetry.addData("right_driveF", ((fwd - strafe) - rotate) / denominator);
//      telemetry.addData("right_driveB", ((fwd + strafe) - rotate) / denominator);
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void autoWarehouse4(double power, double turnPower, double turnPower2, int sleep2) {
//    if (duckLocation == 1) {
//      motorElevator.setTargetPosition(elevatorPos1);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    } else if (duckLocation == 2) {
//      motorElevator.setTargetPosition(elevatorPos2);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    } else if (duckLocation == 3) {
//      motorElevator.setTargetPosition(elevatorPos3);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    runToPosition(2, power, sleep2);
//    if (startColor.equals("red")) {
//      turnToAngle(-140, turnPower, turnPower2, sleep2);
//      runToPosition(-26, power, sleep2);
//      turnToAngle(-160, turnPower, turnPower2, sleep2);
//      runToPosition(-2, power, sleep2);
//    } else if (startColor.equals("blue")) {
//      turnToAngle(140, turnPower, turnPower2, sleep2);
//      runToPosition(-26, power, sleep2);
//      turnToAngle(155, turnPower, turnPower2, sleep2);
//      runToPosition(-2, power, sleep2);
//    }
//    servoBucket.setPosition(servoBucketDown);
//    sleep(2500);
//    runToPosition(22, power, sleep2);
//    servoBucket.setPosition(servoBucketUp);
//    motorElevator.setTargetPosition(elevatorPosDown);
//    if (startColor.equals("red")) {
//      turnToAngle(-90, turnPower, turnPower2, sleep2);
//    } else if (startColor.equals("blue")) {
//      turnToAngle(90, turnPower, turnPower2, sleep2);
//    }
//    if (startColor.equals("red")) {
//      driveMotors(0, 0.3, 0);
//    } else if (startColor.equals("blue")) {
//      driveMotors(0, -0.3, 0);
//    }
//    sleep(1000);
//    if (startColor.equals("red")) {
//      runToPosition(35, power, sleep2);
//    } else if (startColor.equals("blue")) {
//      runToPosition(32, power, sleep2);
//    }
//    if (startColor.equals("red")) {
//      turnToAngle(-90, turnPower, turnPower2, sleep2);
//    } else if (startColor.equals("blue")) {
//      turnToAngle(90, turnPower, turnPower2, sleep2);
//    }
//    if (stoppingSpot.equals("sharedEntrance")) {
//      if (startColor.equals("red")) {
//        driveMotors(0, -0.35, 0);
//      } else if (startColor.equals("blue")) {
//        driveMotors(0, 0.35, 0);
//      }
//      sleep(1600);
//      turnToAngle(180, turnPower, turnPower2, sleep2);
//      if (startColor.equals("red")) {
//        driveMotors(0, -0.4, 0);
//      } else if (startColor.equals("blue")) {
//        driveMotors(0, 0.4, 0);
//      }
//      sleep(1200);
//      if (startColor.equals("red")) {
//        driveMotors(0, -0.3, 0);
//      } else if (startColor.equals("blue")) {
//        driveMotors(0, 0.3, 0);
//      }
//      sleep(300);
//      // put arm down here once sensor is intalled
//      sleep(200);
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void oneTimeInit() {
//    int armDegrees;
//
//    intakePower = 0;
//    // set start time minus 4 billion so the intake doesn't start spinning when you turn the robot on
//    timerIntake = new ElapsedTime(System.nanoTime() - 4000000000L);
//    duckLocation = 3;
//    // Create new IMU Parameters object.
//    imuParameters = new BNO055IMU.Parameters();
//    // Use degrees as angle unit.
//    imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//    // Express acceleration as m/s^2.
//    imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//    // Disable logging.
//    imuParameters.loggingEnabled = false;
//    if (!teleop) {
//      // so we don't reset the imu and mess up field oriented in teleop
//      // Initialize IMU.
//      imu.initialize(imuParameters);
//    }
//    // Get absolute orientation
//    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//    initialAngle = angles.firstAngle;
//    // Initialize Motors.
//    armTarget = "in";
//    armDegrees = 10;
//    motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
//    motorArm.setPower(0);
//    if (!teleop) {
//      motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//    motorIntake.setPower(0);
//    motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
//    motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    motorElevator.setDirection(DcMotorSimple.Direction.FORWARD);
//    if (!teleop) {
//      motorElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//    motorElevator.setPower(1);
//    if (teleop) {
//      motorElevator.setTargetPosition(elevatorPos1);
//      elevTarget = 1;
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    // carousel
//    carouselOn = false;
//    handledThisX = false;
//    // Initialize Motors.
//    driveLF.setDirection(DcMotorSimple.Direction.FORWARD);
//    driveLB.setDirection(DcMotorSimple.Direction.FORWARD);
//    driveRF.setDirection(DcMotorSimple.Direction.REVERSE);
//    driveRB.setDirection(DcMotorSimple.Direction.REVERSE);
//    driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveLF.setPower(0.3);
//    driveLB.setPower(0.3);
//    driveRF.setPower(0.3);
//    driveRB.setPower(0.3);
//    // set the range of the servo
//    servoBucket.setPosition(servoBucketUp);
//    if (teleop == false) {
//      // Sample TFOD Op Mode
//      // Initialize Vuforia.
//      // This sample assumes phone is in landscape mode.
//      // Rotate phone -90 so back camera faces "forward" direction on robot.
//      // We need Vuforia to provide TFOD with camera images.
//      vuforiaFreightFrenzy.initialize(
//          "", // vuforiaLicenseKey
//          VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
//          false, // useExtendedTracking
//          false, // enableCameraMonitoring
//          VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
//          0, // dx
//          0, // dy
//          0, // dz
//          AxesOrder.XYZ, // axesOrder
//          0, // firstAngle
//          -90, // secondAngle
//          0, // thirdAngle
//          true); // useCompetitionFieldTargetLocations
//      // Set min confidence threshold to 0.7
//      tfod.initialize(vuforiaFreightFrenzy, (float) 0.3, true, true);
//      // Set clipping margins
//      tfod.setClippingMargins(0, 0, 0, 0);
//      // Initialize TFOD before waitForStart.
//      // Init TFOD here so the object detection labels are visible
//      // in the Camera Stream preview window on the Driver Station.
//      tfod.activate();
//      // Enable following block to zoom in on target.
//      tfod.setZoom(1.001, 16 / 8);
//      telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//      telemetry.addData(">", "Press Play to start");
//    }
//    // Prompt user to press start buton.
//    telemetry.addData("IMU Example", "Press start to continue...");
//    telemetry.update();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void autoWarehouse5(double power, double turnPower, double turnPower2, int sleep2) {
//    if (duckLocation == 1) {
//      motorElevator.setTargetPosition(elevatorPos1);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    } else if (duckLocation == 2) {
//      motorElevator.setTargetPosition(elevatorPos2);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    } else if (duckLocation == 3) {
//      motorElevator.setTargetPosition(elevatorPos3);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    if (startColor.equals("red")) {
//      driveMotors(0.25, -0.3, 0);
//      sleep(250);
//      turnToAngle(-150, turnPower, turnPower2, sleep2);
//      runToPosition2(-21, power, sleep2, 0.5, true);
//    } else if (startColor.equals("blue")) {
//      driveMotors(0.25, 0.3, 0);
//      sleep(250);
//      turnToAngle(148, turnPower, turnPower2, sleep2);
//      runToPosition2(-21, power, sleep2, 0.5, true);
//    }
//    servoBucket.setPosition(servoBucketDown);
//    runToPosition2(22, power, sleep2, 2, false);
//    servoBucket.setPosition(servoBucketUp);
//    motorElevator.setTargetPosition(elevatorPosDown);
//    if (startColor.equals("red")) {
//      turn(-90, turnPower, turnPower2, 10, 1, 1);
//    } else if (startColor.equals("blue")) {
//      turn(90, turnPower, turnPower2, 10, 1, 1);
//    }
//    if (startColor.equals("red")) {
//      driveMotors(0, 0.3, 0);
//    } else if (startColor.equals("blue")) {
//      driveMotors(0, -0.3, 0);
//    }
//    sleep(1000);
//    colorSensorTest(power, turnPower, turnPower2, sleep2);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void AutoCarousel3(double power, double turnPower, double turnPower2, int sleep2) {
//    if (duckLocation == 1) {
//      motorElevator.setTargetPosition(elevatorPos1);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    } else if (duckLocation == 2) {
//      motorElevator.setTargetPosition(elevatorPos2);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    } else if (duckLocation == 3) {
//      motorElevator.setTargetPosition(elevatorPos3);
//      motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    runToPosition(2, power, sleep2);
//    if (startColor.equals("red")) {
//      turnToAngle(145, turnPower, turnPower2, sleep2);
//      runToPosition(-25.5, power, sleep2);
//    } else if (startColor.equals("blue")) {
//      turnToAngle(-140, turnPower, turnPower2, sleep2);
//      runToPosition(-26, power, sleep2);
//      turnToAngle(-160, turnPower, turnPower2, sleep2);
//      runToPosition(-2, power, sleep2);
//    }
//    // deliver block
//    servoBucket.setPosition(servoBucketDown);
//    sleep(2500);
//    if (startColor.equals("red")) {
//      runToPosition(12, power, sleep2);
//    } else if (startColor.equals("blue")) {
//      runToPosition(10, power, sleep2);
//    }
//    servoBucket.setPosition(servoBucketUp);
//    motorElevator.setTargetPosition(elevatorPosDown);
//    if (startColor.equals("red")) {
//      turnToAngle(90, turnPower, turnPower2, sleep2);
//      runToPosition(28, power, sleep2);
//    } else if (startColor.equals("blue")) {
//      turnToAngle(90, turnPower, turnPower2, sleep2);
//      runToPosition(-32, power, sleep2);
//    }
//    if (startColor.equals("red")) {
//      turnToAngle(45, turnPower, turnPower2, sleep2);
//    } else if (startColor.equals("blue")) {
//      turnToAngle(135, turnPower, turnPower2, sleep2);
//    }
//    if (startColor.equals("red")) {
//      driveMotors(0, -0.275, 0);
//    } else if (startColor.equals("blue")) {
//      driveMotors(0, -0.275, 0);
//    }
//    sleep(1000);
//    if (startColor.equals("red")) {
//      ((DcMotorEx) motorCarousel).setVelocity(-1 * carouselDuckVelocity);
//      telemetry.addData("carouselPower", carouselDuckVelocity);
//    } else if (startColor.equals("blue")) {
//      ((DcMotorEx) motorCarousel).setVelocity(carouselDuckVelocity);
//      telemetry.addData("carouselPower", carouselDuckVelocity);
//    }
//    sleep(4000);
//    ((DcMotorEx) motorCarousel).setVelocity(0);
//    if (stoppingSpot.equals("sharedEntrance") || stoppingSpot.equals("allianceEntrance")) {
//      if (startColor.equals("red")) {
//        turnToAngle(-90, turnPower, turnPower2, sleep2);
//        runToPosition(60, power, sleep2);
//        driveMotors(0, 0.4, 0);
//        sleep(1500);
//        driveMotors(0, 0.3, 0);
//        sleep(300);
//        runToPosition(40, power, sleep2);
//        turnToAngle(-90, turnPower, turnPower2, sleep2);
//      } else if (startColor.equals("blue")) {
//        turnToAngle(90, turnPower, turnPower2, sleep2);
//        runToPosition(60, power, sleep2);
//        driveMotors(0, -0.4, 0);
//        sleep(1500);
//        driveMotors(0, -0.3, 0);
//        sleep(300);
//        runToPosition(40, power, sleep2);
//        turnToAngle(90, turnPower, turnPower2, sleep2);
//      }
//    } else {
//      if (startColor.equals("red")) {
//        turnToAngle(0, turnPower, turnPower2, sleep2);
//      } else if (startColor.equals("blue")) {
//        turnToAngle(180, turnPower, turnPower2, sleep2);
//      }
//      ((DcMotorEx) motorCarousel).setVelocity(0);
//      if (startColor.equals("red")) {
//        runToPosition(20, power, sleep2);
//        turnToAngle(0, turnPower, turnPower2, sleep2);
//      } else if (startColor.equals("blue")) {
//        runToPosition(-20, power, sleep2);
//        turnToAngle(180, turnPower, turnPower2, sleep2);
//      }
//      driveMotors(0, 0, 0);
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void colorSensorTest(double power, double turnPower, double turnPower2, int sleep2) {
//    timerIntake.reset();
//    if (startColor.equals("red")) {
//      driveMotors(0.3, 0.2, 0);
//    } else if (startColor.equals("blue")) {
//      driveMotors(0.3, -0.2, 0);
//    }
//    pickUpFailSafe = new ElapsedTime(System.nanoTime());
//    armTarget = "out";
//    alreadyBackingUp = false;
//    while (true) {
//      if (motorArm.getCurrentPosition() >= 80 && sensorDistanceIntake.getDistance(DistanceUnit.INCH) <= intakeThreshold || alreadyBackingUp == false && pickUpFailSafe.seconds() >= 4) {
//        armTarget = "in";
//        if (startColor.equals("red")) {
//          driveMotors(-0.2, 0.15, 0);
//        } else if (startColor.equals("blue")) {
//          driveMotors(-0.2, -0.15, 0);
//        }
//        if (alreadyBackingUp == false) {
//          backingUpFailSafe = new ElapsedTime(System.nanoTime());
//        }
//        alreadyBackingUp = true;
//      }
//      if (armTarget.equals("out")) {
//        timerIntake.reset();
//        if (motorArm.getCurrentPosition() > 80) {
//          motorArm.setPower(0.1);
//          intakePower = 1;
//        } else {
//          motorArm.setPower(0.5);
//          intakePower = 0;
//        }
//      } else {
//        // arm target == in
//        if (motorArm.getCurrentPosition() < 10) {
//          if (timerIntake.seconds() <= 2) {
//            intakePower = -1;
//            motorArm.setPower(-0.1);
//          } else {
//            intakePower = 0;
//            motorArm.setPower(0);
//          }
//        } else if (motorArm.getCurrentPosition() < 60) {
//          timerIntake.reset();
//          motorArm.setPower(-0.5);
//          intakePower = 0.5;
//        } else {
//          timerIntake.reset();
//          motorArm.setPower(-0.8);
//          intakePower = 1;
//        }
//      }
//      motorIntake.setPower(intakePower);
//      if (sensorColorLine.alpha() > 120 && armTarget.equals("in")) {
//        break;
//      }
//      if (alreadyBackingUp == true && backingUpFailSafe.seconds() >= 2) {
//        runToPosition2(12, power, sleep2, 1, false);
//        if (stoppingSpot.equals("sharedEntrance")) {
//          warehouseSharedEntrance(power, sleep2, turnPower, turnPower2);
//        }
//        if (true) {
//          return;
//        }
//      }
//    }
//    runToPosition2(-23, power, sleep2, 0, false);
//    motorIntake.setPower(0);
//    motorElevator.setTargetPosition(elevatorPos3);
//    motorElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    if (startColor.equals("red")) {
//      turnToAngle(-155, turnPower, turnPower2, sleep2);
//      runToPosition2(-25, power, sleep2, 0.5, true);
//    } else if (startColor.equals("blue")) {
//      turnToAngle(150, turnPower, turnPower2, sleep2);
//      runToPosition2(-26, power, sleep2, 0.5, true);
//    }
//    servoBucket.setPosition(servoBucketDown);
//    runToPosition2(22, power, sleep2, 2, false);
//    servoBucket.setPosition(servoBucketUp);
//    motorElevator.setTargetPosition(elevatorPosDown);
//    if (startColor.equals("red")) {
//      turn(-90, turnPower, turnPower2, 10, 1, 1);
//    } else if (startColor.equals("blue")) {
//      turn(90, turnPower, turnPower2, 10, 1, 1);
//    }
//    if (startColor.equals("red")) {
//      driveMotors(0, 0.3, 0);
//    } else if (startColor.equals("blue")) {
//      driveMotors(0, -0.3, 0);
//    }
//    sleep(1000);
//    runToPosition2(28, power, sleep2, 0.5, false);
//    if (stoppingSpot.equals("sharedEntrance")) {
//      warehouseSharedEntrance(power, sleep2, turnPower, turnPower2);
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void initLoop() {
//    List<Recognition> recognitions;
//    int index;
//
//    while (!opModeIsActive()) {
//      telemetry.addData("elevator tics", motorElevator.getCurrentPosition());
//      telemetry.addData("sensorLine Alpha", sensorColorLine.alpha());
//      // ^remove later once we make  a copy of the code for teleop
//      if (gamepad1.x) {
//        startColor = "blue";
//      } else if (gamepad1.b) {
//        startColor = "red";
//      }
//      if (gamepad1.y) {
//        startCOrW = "warehouse";
//      } else if (gamepad1.a) {
//        startCOrW = "carousel";
//      }
//      if (gamepad1.dpad_up) {
//        stoppingSpot = "sharedEntrance";
//      } else if (gamepad1.dpad_down) {
//        stoppingSpot = "allianceEntrance";
//      } else if (gamepad1.dpad_left) {
//        stoppingSpot = "storage";
//      } else if (gamepad1.dpad_right) {
//        stoppingSpot = "storage";
//      }
//      telemetry.addData("startColor", startColor);
//      telemetry.addData("startCOrW", startCOrW);
//      telemetry.addData("stoppingSpot", stoppingSpot);
//      if (teleop == false) {
//        // Locate duck!
//        // Get a list of recognitions from TFOD.
//        recognitions = tfod.getRecognitions();
//        // If list is empty, inform the user. Otherwise, go
//        // through list and display info for each recognition.
//        if (recognitions.size() == 0) {
//          telemetry.addData("TFOD", "No items detected.");
//        } else {
//          index = 0;
//          // Iterate through list and call a function to
//          // display info for each recognized object.
//          for (Recognition recognition_item : recognitions) {
//            recognition = recognition_item;
//            if (!recognition.getLabel().equals("Duck") && !recognition.getLabel().equals("Cube")) {
//              continue;
//            }
//            // determine duck location
//            if (recognition.estimateAngleToObject(AngleUnit.DEGREES) < -15) {
//              duckLocation = 1;
//            } else if (recognition.estimateAngleToObject(AngleUnit.DEGREES) > -10 && recognition.estimateAngleToObject(AngleUnit.DEGREES) < 10) {
//              duckLocation = 2;
//            } else if (recognition.estimateAngleToObject(AngleUnit.DEGREES) > 15) {
//              duckLocation = 3;
//            }
//            // Display info.
//            displayInfo(index);
//            // Increment index.
//            index = index + 1;
//          }
//        }
//      }
//      telemetry.addData("duckLocation", duckLocation);
//      telemetry.addData("initialAngle", initialAngle);
//      telemetry.update();
//    }
//  }
//
//  /**
//   * Display info (using telemetry) for a recognized object.
//   */
//  private void displayInfo(int i) {
//    // Display label info.
//    // Display the label and index number for the recognition.
//    telemetry.addData("label " + i, recognition.getLabel());
//    // Display upper corner info.
//    // Display the location of the top left corner
//    // of the detection boundary for the recognition
//    telemetry.addData("Left, Top " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
//    // Display lower corner info.
//    // Display the location of the bottom right corner
//    // of the detection boundary for the recognition
//    telemetry.addData("Right, Bottom " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
//    // Display the label and index number for the recognition.
//    telemetry.addData("confidence" + i, recognition.getConfidence());
//    // Display the label and index number for the recognition.
//    telemetry.addData("estimateAngleToObject", recognition.estimateAngleToObject(AngleUnit.DEGREES));
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void warehouseSharedEntrance(double power, int sleep2, double turnPower, double turnPower2) {
//    if (startColor.equals("red")) {
//      driveMotors(0, -0.35, 0);
//    } else if (startColor.equals("blue")) {
//      driveMotors(0, 0.35, 0);
//    }
//    if (startColor.equals("red")) {
//      sleep(1650);
//    } else if (startColor.equals("blue")) {
//      sleep(1650);
//    }
//    // put arm down here once sensor is intalled
//  }
//
//  /**
//   * Describe this function...
//   */
//  private double max(double input1, double input2) {
//    if (input1 > input2) {
//      return input1;
//    }
//    return input2;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private double parabolicTransform(double stick) {
//    if (stick < 0) {
//      stick = -Math.pow(stick, 2);
//    } else {
//      stick = Math.pow(stick, 2);
//    }
//    return stick;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void runToPosition(double targetInches, double power, int sleep2) {
//    runToPosition2((int) targetInches, power, sleep2, 0.01, false);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void runToPosition2(int targetInches, double power, int sleep2, double tolerance, boolean freight) {
//    targetPosition = (int) (targetInches * 31.25);
//    tolerance = tolerance * 31.25;
//    runToPositionInit(targetPosition, power, sleep2);
//    while (opModeIsActive()) {
//      if (freight) {
//        if (Math.abs(targetPosition - driveLF.getCurrentPosition()) < 4 * 31.25) {
//          servoBucket.setPosition(servoBucketDown);
//        }
//      }
//      if (Math.abs(targetPosition - driveLF.getCurrentPosition()) < tolerance && Math.abs(targetPosition - driveRF.getCurrentPosition()) < tolerance && Math.abs(targetPosition - driveLB.getCurrentPosition()) < tolerance && Math.abs(targetPosition - driveRB.getCurrentPosition()) < tolerance) {
//        break;
//      }
//      if (driveLF.isBusy() || driveLB.isBusy() || driveRF.isBusy() || driveRB.isBusy()) {
//        continue;
//      }
//      break;
//    }
//    sleep(sleep2);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void runToPositionInit(int targetPosition, double power, int sleep2) {
//    driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveLF.setPower(power);
//    driveLB.setPower(power);
//    driveRF.setPower(power);
//    driveRB.setPower(power);
//    driveLF.setTargetPosition(targetPosition);
//    driveLB.setTargetPosition(targetPosition);
//    driveRF.setTargetPosition(targetPosition);
//    driveRB.setTargetPosition(targetPosition);
//    driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void runToPositionHub(int targetInches, double power, int sleep2) {
//    runToPositionInit((int) targetInches, power, sleep2);
//    while (opModeIsActive()) {
//      hubDistance = sensorDistanceHub.getDistance(DistanceUnit.INCH);
//      if (telemetryEnabled) {
//        telemetry.addData("hub sensor distance", hubDistance);
//      }
//      if (hubDistance > 5 && hubDistance < 20) {
//        if (telemetryEnabled) {
//          telemetry.addData("runToPositionDistance", 123);
//        }
//        targetPosition = (int) ((driveLF.getCurrentPosition() + driveRF.getCurrentPosition()) / 2 - (hubDistance - 10) * 31.25);
//        driveLF.setTargetPosition(targetPosition);
//        driveLB.setTargetPosition(targetPosition);
//        driveRF.setTargetPosition(targetPosition);
//        driveRB.setTargetPosition(targetPosition);
//        driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      }
//      runToPositionLoop(targetPosition / 31.25, power, sleep2);
//      telemetry.update();
//      if (driveLF.isBusy() || driveLB.isBusy() || driveRF.isBusy() || driveRB.isBusy()) {
//        continue;
//      }
//      break;
//    }
//    sleep(sleep2);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void runToPositionLoop(double targetInches, double power, int sleep2) {
//    if (telemetryEnabled) {
//      // set how close to the target position the robot stops at
//      // set last position to current position
//      // current position to the current position
//      currentPositionLF = driveLF.getCurrentPosition();
//      currentPositionLB = driveLB.getCurrentPosition();
//      currentPositionRF = driveRF.getCurrentPosition();
//      currentPositionRB = driveRB.getCurrentPosition();
//      telemetry.addData("currentPositionLF", currentPositionLF);
//      telemetry.addData("currentPositionLB", currentPositionLB);
//      telemetry.addData("currentPositionRF", currentPositionRF);
//      telemetry.addData("currentPositionRB", currentPositionRB);
//      telemetry.addData("isBusyLF", driveLF.isBusy());
//      telemetry.addData("isBusyLB", driveLB.isBusy());
//      telemetry.addData("isBusyRF", driveRF.isBusy());
//      telemetry.addData("isBusyRB", driveRB.isBusy());
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void runToPositionOld(double targetInches, double power, long sleep2) {
//    targetPosition = (int) (targetInches * 31.25);
//    driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    driveLF.setPower(power);
//    driveLB.setPower(power);
//    driveRF.setPower(power);
//    driveRB.setPower(power);
//    driveLF.setTargetPosition(targetPosition);
//    driveLB.setTargetPosition(targetPosition);
//    driveRF.setTargetPosition(targetPosition);
//    driveRB.setTargetPosition(targetPosition);
//    driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    // set how close to the target position the robot stops at
//    while (opModeIsActive()) {
//      // set last position to current position
//      // current position to the current position
//      currentPositionLF = driveLF.getCurrentPosition();
//      currentPositionLB = driveLB.getCurrentPosition();
//      currentPositionRF = driveRF.getCurrentPosition();
//      currentPositionRB = driveRB.getCurrentPosition();
//      if (telemetryEnabled) {
//        telemetry.addData("currentPositionLF", currentPositionLF);
//        telemetry.addData("currentPositionLB", currentPositionLB);
//        telemetry.addData("currentPositionRF", currentPositionRF);
//        telemetry.addData("currentPositionRB", currentPositionRB);
//        telemetry.addData("isBusyLF", driveLF.isBusy());
//        telemetry.addData("isBusyLB", driveLB.isBusy());
//        telemetry.addData("isBusyRF", driveRF.isBusy());
//        telemetry.addData("isBusyRB", driveRB.isBusy());
//        telemetry.update();
//      }
//      if (driveLF.isBusy() || driveLB.isBusy() || driveRF.isBusy() || driveRB.isBusy()) {
//        continue;
//      }
//      break;
//    }
//    sleep(sleep2);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private double setCarouselSpeed() {
//    carouselSpeed = 1000000000 * ((motorCarousel.getCurrentPosition() - carouselLastPosition) / (System.nanoTime() - carouselLastTime));
//    carouselLastPosition = motorCarousel.getCurrentPosition();
//    carouselLastTime = System.nanoTime();
//    if (telemetryEnabled) {
//      telemetry.addData("carouselSpeed", carouselSpeed);
//    }
//    return carouselSpeed;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private double teleopTurnToAngle(int targetAngle, double currentAngle, double powerIn, double powerIn2) {
//    angleDifference = targetAngle - currentAngle;
//    if (angleDifference > 180) {
//      angleDifference += -360;
//    }
//    if (angleDifference < -180) {
//      angleDifference += 360;
//    }
//    if (telemetryEnabled) {
//      telemetry.addData("angleDifference", angleDifference);
//      telemetry.addData("currentAngle", currentAngle);
//    }
//    if (Math.abs(angleDifference) < 15) {
//      rotateOut = powerIn2;
//    } else if (Math.abs(angleDifference) < 45) {
//      rotateOut = (Math.abs(angleDifference) / 45) * (powerIn - powerIn2) + powerIn2;
//    } else {
//      rotateOut = powerIn;
//    }
//    if (angleDifference < -2) {
//      rotateOut = rotateOut;
//    } else if (angleDifference > 2) {
//      rotateOut = -1 * rotateOut;
//    } else {
//      rotateOut = 0;
//    }
//    return rotateOut;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void runToPositionLeftRight2(double targetInchesL, double targetInchesR, double powerL, double powerR, int sleep2, int tolerance, boolean freight) {
//    targetPositionL = targetInchesL * 31.25;
//    targetPositionR = targetInchesR * 31.25;
//    tolerance = tolerance * 31.25;
//    ((DcMotorEx) driveLF).setTargetPositionTolerance(tolerance);
//    ((DcMotorEx) driveRF).setTargetPositionTolerance(tolerance);
//    ((DcMotorEx) driveLB).setTargetPositionTolerance(tolerance);
//    ((DcMotorEx) driveRB).setTargetPositionTolerance(tolerance);
//    runToPositionInitLeftRight((int) targetPositionL, (int) targetPositionR, powerL, powerR, sleep2);
//    while (opModeIsActive()) {
//      positionLF = driveLF.getCurrentPosition();
//      positionRF = driveRF.getCurrentPosition();
//      positionLB = driveLB.getCurrentPosition();
//      positionRB = driveRB.getCurrentPosition();
//      if (freight) {
//        if (Math.abs(targetPositionL - positionLF) < 4 * 31.25 && Math.abs(targetPositionR - positionRF) < 4 * 31.25) {
//          servoBucket.setPosition(servoBucketDown);
//        }
//      }
//      if (driveLF.isBusy() || driveLB.isBusy() || driveRF.isBusy() || driveRB.isBusy()) {
//        continue;
//      }
//      break;
//    }
//    sleep(sleep2);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void turnToAngle(int targetAngle, double turnPower, double turnPower2, int sleep2) {
//    turn(targetAngle, turnPower, turnPower2, 2, 4, 4);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void turn(int targetAngle, double powerIn, double powerIn2, int tolerance, int targetReachedCountThreshold, int failSafeCountThreshold) {
//    double rotate;
//    int failSafeCount;
//    int targetReachedCount;
//
//    failSafeCount = 0;
//    targetReachedCount = 0;
//    while (opModeIsActive()) {
//      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//      currentAngle = angles.firstAngle;
//      angleDifference = targetAngle - currentAngle;
//      if (angleDifference > 180) {
//        angleDifference += -360;
//      }
//      if (angleDifference < -180) {
//        angleDifference += 360;
//      }
//      if (telemetryEnabled) {
//        telemetry.addData("angleDifference", angleDifference);
//        telemetry.addData("currentAngle", currentAngle);
//      }
//      if (Math.abs(angleDifference) < 10) {
//        power = powerIn2;
//      } else if (Math.abs(angleDifference) < 40) {
//        power = (Math.abs(angleDifference) / 40) * (powerIn - powerIn2) + powerIn2;
//      } else {
//        power = powerIn;
//      }
//      if (angleDifference < -tolerance) {
//        targetReachedCount = 0;
//        rotate = power;
//      } else if (angleDifference > tolerance) {
//        targetReachedCount = 0;
//        rotate = -1 * power;
//      } else {
//        targetReachedCount += 1;
//        failSafeCount += 1;
//        rotate = 0;
//        driveMotors(0, 0, 0);
//        if (targetReachedCount >= targetReachedCountThreshold || failSafeCount >= failSafeCountThreshold) {
//          break;
//        }
//      }
//      driveMotors(0, 0, (float) rotate);
//      telemetry.update();
//    }
//    driveMotors(0, 0, 0);
//    telemetry.update();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void runToPositionLeftRight(int targetInchesL, int targetInchesR, double powerL, double powerR, int sleep2, double tolerance, boolean freight) {
//    targetPositionL = targetInchesL * 31.25;
//    targetPositionR = targetInchesR * 31.25;
//    tolerance = tolerance * 31.25;
//    ((DcMotorEx) driveLF).setTargetPositionTolerance((int) tolerance);
//    ((DcMotorEx) driveRF).setTargetPositionTolerance((int) tolerance);
//    ((DcMotorEx) driveLB).setTargetPositionTolerance((int) tolerance);
//    ((DcMotorEx) driveRB).setTargetPositionTolerance((int) tolerance);
//    runToPositionInitLeftRight((int) targetPositionL, (int) targetPositionR, powerL, powerR, sleep2);
//    while (opModeIsActive()) {
//      positionLF = driveLF.getCurrentPosition();
//      positionRF = driveRF.getCurrentPosition();
//      positionLB = driveLB.getCurrentPosition();
//      positionRB = driveRB.getCurrentPosition();
//      if (freight) {
//        if (Math.abs(targetPositionL - positionLF) < 4 * 31.25 && Math.abs(targetPositionR - positionRF) < 4 * 31.25) {
//          servoBucket.setPosition(servoBucketDown);
//        }
//      }
//      if (driveLF.isBusy() || driveLB.isBusy() || driveRF.isBusy() || driveRB.isBusy()) {
//        continue;
//      }
//      break;
//    }
//    sleep(sleep2);
//  }
//}
