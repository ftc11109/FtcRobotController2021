package org.firstinspires.ftc.teamcode;

public class AutoDeliverParkVersions {

//    private void autoDeliverPark2() {
//        int sleepTime = 500;
//        double tolerance = 0.2;
//        double turnTolerance = 0.2;
//        int failSafeCountThreshold = 4;
//        int targetReachedCountThreshold = 3;
//        double powerTurnHigh = .3;
//        double powerTurnLow = .17;
//        double powerDriveHigh = .3;
//        double powerDriveLow = .17;
//
//        runToPosition(37.5, powerDriveHigh, sleepTime, tolerance);
//        motorArm.setTargetPosition(armDeliverMedium);
//        motorSlideL.setTargetPosition(slideDeliverMedium);
//        motorSlideR.setTargetPosition(slideDeliverMedium);
//        motorIntake.setPower(intakePowerPickup);
//        sleep(3000);
//        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//            turn(90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//        } else {
//            turn(-90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//        }
//
//
//
//
//
//
//
//
//        if (Spot(RED, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(RED, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//
//
//        motorSlideL.setTargetPosition(slideDeliverMedium - 70);
//        motorSlideR.setTargetPosition(slideDeliverMedium - 70);
//        sleep(1000);
//
//        motorIntake.setPower(intakePowerDeliver);
//        sleep(1000);
//        motorIntake.setPower(0);
//
//        if (Spot(RED, AUDIENCE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(RED, JUDGE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, AUDIENCE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, JUDGE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
//
//        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//
//        motorArm.setTargetPosition(0);
//        motorSlideL.setTargetPosition(0);
//        motorSlideR.setTargetPosition(0);
//
//        sleep(1500);
//
//        runToPosition(10.5, powerDriveHigh, sleepTime, tolerance);
//
//
//
//        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//
//
//        // if we didn't detect a parking spot, pick a good default
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
//
//
//    }
//
//
//
//    private void autoDeliverPark3() {
//        int sleepTime = 500;
//        double tolerance = 0.2;
//        double turnTolerance = 0.2;
//        int failSafeCountThreshold = 4;
//        int targetReachedCountThreshold = 3;
//        double powerTurnHigh = .3;
//        double powerTurnLow = .17;
//        double powerDriveHigh = .3;
//        double powerDriveLow = .17;
//
//        runToPosition(37.5, powerDriveHigh, sleepTime, tolerance);
//        motorArm.setTargetPosition(armDeliverMedium);
//        motorSlideL.setTargetPosition(slideDeliverMedium);
//        motorSlideR.setTargetPosition(slideDeliverMedium);
//        motorIntake.setPower(intakePowerHold);
//        sleep(3000);
//        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//            turn(90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//        } else {
//            turn(-90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//        }
//
//
//
//
//
//
//
//
////        if (Spot(RED, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
////        if (Spot(RED, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
////        if (Spot(BLUE, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
////        if (Spot(BLUE, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//
//
//
//        autoJunctionDeliver(4);
//        sleep(1000);
//
//
//
//
//        if (Spot(RED, AUDIENCE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(RED, JUDGE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, AUDIENCE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, JUDGE)) runToPosition(5, powerDriveHigh, sleepTime, tolerance);
//
//        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//
//        motorArm.setTargetPosition(0);
//        motorSlideL.setTargetPosition(0);
//        motorSlideR.setTargetPosition(0);
//
//
//        sleep(1500);
//
//        runToPosition(10.5, powerDriveHigh, sleepTime, tolerance);
//
//
//
//        turn(0, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//
//
//        // if we didn't detect a parking spot, pick a good default
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
//
//
//    }
//
//
//
//
//    private void autoDeliverPark4() {
//        int sleepTime = 100;
//        double tolerance = 0.2;
//        double turnTolerance = 0.2;
//        int failSafeCountThreshold = 4;
//        int targetReachedCountThreshold = 3;
//        double powerTurnHigh = .3;
//        double powerTurnLow = .17;
//        double powerDriveHigh = .4;
//        double powerDriveLow = .17;
//        motorArm.setTargetPosition(armDeliverMedium);
//        motorSlideL.setTargetPosition(slideDeliverMedium);
//        motorSlideR.setTargetPosition(slideDeliverMedium);
//        motorIntake.setPower(intakePowerHold);
//        runToPosition(39, powerDriveHigh, sleepTime, tolerance);
//
//
//        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//            turn(90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//        } else {
//            turn(-90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
//        }
//
//
//
////        if (Spot(RED, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
////        if (Spot(RED, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
////        if (Spot(BLUE, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
////        if (Spot(BLUE, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//
//
//
//        autoJunctionDeliver(4);
//
//        motorArm.setTargetPosition(armPickupHigh);
//        motorSlideL.setTargetPosition(slidePickupHigh);
//        motorSlideR.setTargetPosition(slidePickupHigh);
//
//        if (Spot(RED, AUDIENCE)) runToPosition(3, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(RED, JUDGE)) runToPosition(3, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, AUDIENCE)) runToPosition(3, powerDriveHigh, sleepTime, tolerance);
//        if (Spot(BLUE, JUDGE)) runToPosition(3, powerDriveHigh, sleepTime, tolerance);
//
//        strafeToPosition(12,powerDriveHigh,0,0.5);
//
//        autoFollowLine(powerDriveHigh,0,0.1,28, driveLF);
//
//        while (opModeIsActive()) {
//            autoPickupCone();
//
//
//            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//                runToPositionLeftRightWithJunction(-20, -34, powerDriveHigh, powerDriveHigh, sleepTime, tolerance,4);
//            } else{
//                runToPositionLeftRightWithJunction(-34, -20, powerDriveHigh, powerDriveHigh, sleepTime, tolerance,4);
//            }
//
//
////            motorArm.setTargetPosition(armDeliverMedium);
////            motorSlideL.setTargetPosition(slideDeliverMedium);
////            motorSlideR.setTargetPosition(slideDeliverMedium);
//
//            autoJunctionDeliver(4);
//
//
//
//            if (conesRemaining == 3) {
//                break;
//            }
//
//
//            motorArm.setTargetPosition(armPickupHigh);
//            motorSlideL.setTargetPosition(slidePickupHigh);
//            motorSlideR.setTargetPosition(slidePickupHigh);
//
//
//
//            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//                autoFollowLine(powerDriveHigh, powerDriveHigh * 0.3, 0.1, 34,driveLF);
//            } else{
//                autoFollowLine(powerDriveHigh, powerDriveHigh * 0.3, 0.1, 34,driveRF);
//            }
//        }
////
////        motorArm.setTargetPosition(0);
////        motorSlideL.setTargetPosition(0);
////        motorSlideR.setTargetPosition(0);
////
////        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
////            runToPositionLeftRight(0, 14, .3, .3, sleepTime, tolerance);
////        } else{
////            runToPositionLeftRight(14, 0, .3, .3, sleepTime, tolerance);
////        }
////
////
////        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
////            turn(90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
////        } else{
////            turn(-90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
////        }
////
//
//
//        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//            runToPositionLeftRight(0, 6, .3, .3, sleepTime, tolerance);
//        } else{
//            runToPositionLeftRight(6, 0, .3, .3, sleepTime, tolerance);
//        }
//        turn(0,.3,.15,1,4,3);
//
//
//        motorArm.setTargetPosition(0);
//        motorSlideL.setTargetPosition(0);
//        motorSlideR.setTargetPosition(0);
//
//        // if we didn't detect a parking spot, pick a good default
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
//
//
//
//    }
//    private void autoDeliverPark5() {
//        int sleepTime = 100;
//        double tolerance = 0.2;
//        double turnTolerance = 0.2;
//        int failSafeCountThreshold = 4;
//        int targetReachedCountThreshold = 3;
//        double powerTurnHigh = .3;
//        double powerTurnLow = .17;
//        double powerDriveHigh = .4;
//        double powerDriveLow = .17;
//        motorArm.setTargetPosition(armDeliverMedium);
//        motorSlideL.setTargetPosition(slideDeliverMedium);
//        motorSlideR.setTargetPosition(slideDeliverMedium);
//        motorIntake.setPower(intakePowerHold);
//        runToPosition(57, powerDriveHigh, sleepTime, tolerance);
//
//
//        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//            runToPositionLeftRight(-14,0,powerDriveHigh,powerDriveHigh,sleepTime,1);
//        } else {
//            runToPositionLeftRight(0,-14,powerDriveHigh,powerDriveHigh,sleepTime,1);
//        }
//        autoJunctionDeliver(4);
//
//        motorArm.setTargetPosition(armPickupHigh);
//        motorSlideL.setTargetPosition(slidePickupHigh);
//        motorSlideR.setTargetPosition(slidePickupHigh);
//
//        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//            autoFollowLine(powerDriveHigh, powerDriveHigh * 0.3, 0.1, 34,driveLF);
//        } else{
//            autoFollowLine(powerDriveHigh, powerDriveHigh * 0.3, 0.1, 34,driveRF);
//        }
//
//
//
////        if (Spot(RED, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
////        if (Spot(RED, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
////        if (Spot(BLUE, AUDIENCE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
////        if (Spot(BLUE, JUDGE)) runToPosition(-5, powerDriveHigh, sleepTime, tolerance);
//
//
//
//
//
//
//        while (opModeIsActive()) {
//            autoPickupCone();
//
//
//            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//                runToPositionLeftRightWithJunction(-20, -34, .3, .3, sleepTime, tolerance,4);
//            } else{
//                runToPositionLeftRightWithJunction(-34, -20, .3, .3, sleepTime, tolerance,4);
//            }
//
//
////            motorArm.setTargetPosition(armDeliverMedium);
////            motorSlideL.setTargetPosition(slideDeliverMedium);
////            motorSlideR.setTargetPosition(slideDeliverMedium);
//
//            autoJunctionDeliver(4);
//
//
//
//            if (conesRemaining == 3) {
//                break;
//            }
//
//
//            motorArm.setTargetPosition(armPickupHigh);
//            motorSlideL.setTargetPosition(slidePickupHigh);
//            motorSlideR.setTargetPosition(slidePickupHigh);
//
//
//
//            if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//                autoFollowLine(powerDriveHigh, powerDriveHigh * 0.3, 0.1, 34,driveLF);
//            } else{
//                autoFollowLine(powerDriveHigh, powerDriveHigh * 0.3, 0.1, 34,driveRF);
//            }
//        }
////
////        motorArm.setTargetPosition(0);
////        motorSlideL.setTargetPosition(0);
////        motorSlideR.setTargetPosition(0);
////
////        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
////            runToPositionLeftRight(0, 14, .3, .3, sleepTime, tolerance);
////        } else{
////            runToPositionLeftRight(14, 0, .3, .3, sleepTime, tolerance);
////        }
////
////
////        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
////            turn(90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
////        } else{
////            turn(-90, powerTurnHigh, powerTurnLow, turnTolerance, targetReachedCountThreshold, failSafeCountThreshold);
////        }
////
//
//
//        if (Spot(RED,AUDIENCE) || Spot(BLUE,JUDGE)) {
//            runToPositionLeftRight(0, 6, .3, .3, sleepTime, tolerance);
//        } else{
//            runToPositionLeftRight(6, 0, .3, .3, sleepTime, tolerance);
//        }
//        turn(0,.3,.15,1,4,3);
//
//
//        motorArm.setTargetPosition(0);
//        motorSlideL.setTargetPosition(0);
//        motorSlideR.setTargetPosition(0);
//
//        // if we didn't detect a parking spot, pick a good default
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
//
//
//
//    }
}
