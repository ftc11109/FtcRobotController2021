//private void strafeToPosition(double targetInches, double power, int sleepTime, int tolerance) {
//    targetPosition = targetInches * 31.25;
//    tolerance = tolerance * 31.25;
//    ((DcMotorEx) driveLF).setTargetPositionTolerance(tolerance);
//    ((DcMotorEx) driveRF).setTargetPositionTolerance(tolerance);
//    ((DcMotorEx) driveLB).setTargetPositionTolerance(tolerance);
//    ((DcMotorEx) driveRB).setTargetPositionTolerance(tolerance);
//          driveLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//          driveLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//          driveRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//          driveRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//          driveLF.setPower(power);
//          driveRF.setPower(power);
//          driveLB.setPower(power);
//          driveRB.setPower(power);
//          driveLF.setTargetPosition((int) targetPosition);
//          driveRF.setTargetPosition((int) -targetPosition);
//          driveLB.setTargetPosition((int) -targetPosition);
//          driveRB.setTargetPosition((int) targetPosition);
//          driveLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//          driveRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//          driveLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//          driveRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    while (opModeIsActive()) {
//      positionLF = driveLF.getCurrentPosition();
//      positionRF = driveRF.getCurrentPosition();
//      positionLB = driveLB.getCurrentPosition();
//      positionRB = driveRB.getCurrentPosition();
//
//
//      break;
//    }
//    sleep(sleepTime);
//  }
//
//// if RDPad = pressed {
////
////    driveLF = -driveLF;
////    driveRB = -driveRB;
////
////       }
////  /      }
////
////        if RDPad = notPressed {
////
////
////        if(any drive is negative) {
////
////            change negative drive variables to absolute values;
////
////        }
////     /  /////////////////////////////////////////////////////////////// CAUTION
////        }