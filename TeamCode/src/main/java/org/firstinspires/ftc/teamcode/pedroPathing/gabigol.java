package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class gabigol extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleftdrive = null;
    private DcMotor backleftdrive = null;
    private DcMotor frontrightdrive = null;
    private DcMotor backrightdrive = null;


    @Override
    public void runOpMode() throws InterruptedException {


    frontleftdrive = hardwareMap.get(DcMotor.class, "roda0");
    backrightdrive = hardwareMap.get(DcMotor.class,"roda1");
    frontleftdrive = hardwareMap.get(DcMotor.class,"roda2");
    frontrightdrive = hardwareMap.get(DcMotor.class,"roda3");

    frontleftdrive.setDirection(DcMotor.Direction.REVERSE);
    frontrightdrive.setDirection(DcMotor.Direction.REVERSE);
    backleftdrive.setDirection(DcMotor.Direction.FORWARD);
    backrightdrive.setDirection(DcMotor.Direction.FORWARD);

    telemetry. addData("Status", "Initialized");
    telemetry.update();

    waitForStart();
    runtime.reset();
     while (opModeIsActive()) {
         double max;

         double axial = -gamepad1.left_stick_y;
         double lateral = gamepad1.left_stick_x;
         double yaw = gamepad1.right_stick_x;

         double frontleftpower = axial + lateral + yaw;
         double frontrightpower = axial - lateral - yaw;
         double backleftpower = axial - lateral + yaw;
         double backrightpower = axial + lateral - yaw;

         max = Math.max(Math.abs(frontleftpower), Math.abs(frontrightpower));
         max = Math.max(max, Math.abs(backleftpower));
         max = Math.max(max, Math.abs(backrightpower));

          if (max > 1.0) {
              frontleftpower /= max;
              frontrightpower /= max;
              backleftpower /= max;
              backrightpower /= max;
          }

          frontleftdrive.setPower(frontleftpower);
          frontrightdrive.setPower(frontrightpower);
          backleftdrive.setPower(backleftpower);
          backrightdrive.setPower(backrightpower);


          telemetry.addData("Status", "Run Time: " + runtime.toString());
          telemetry.addData("Front left/right","%4. 2f,%4. 2f", frontleftpower, frontrightpower);
          telemetry.addData(" Back left/right","%4. 2f, %4. 2f", backleftpower, backrightpower);
          telemetry.update();
     }
    }
}