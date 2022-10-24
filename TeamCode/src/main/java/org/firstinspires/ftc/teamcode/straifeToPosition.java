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

// private void runToPositionLeftRight2(double targetInchesL, double targetInchesR, double powerL, double powerR, int sleep2, int tolerance, boolean freight) {
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
//
//      }
//      break;
//    }
//    sleep(sleep2);
//  }

// if RDPad = pressed {
//
  //  driveLF = -driveLF;
  //  driveRB = -driveRB;
 //
   //    }
  ///      }
  //
  //      if RDPad = notPressed {

   //
      //  if(any drive is negative) {
//
//            change negative drive variables to absolute values;

  //      }
     ///  /////////////////////////////////////////////////////////////// CAUTION
    //    }