package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TELEOP FINAL - CORTEX VERMELHO", group = "CORTEX")
public class tele extends LinearOpMode {

    // ===== DRIVE =====
    DcMotor FL0, FR1, BL2, BR3;

    // ===== CTVL =====
    DcMotorEx ctvl;
    DcMotorEx ctvl2;

    boolean ctvlToggle = false;
    boolean rtLast = false;

    // ===== SHOOTERS =====
    DcMotorEx shooterL, shooterR;

    // ===== RPM PRESETS =====
    final double RPM_A = 1470;
    final double RPM_B = 1355;
    final double RPM_X = 1475;
    final double RPM_Y = 1865;

    // ===== CTVL2 TIMER (GAMEPAD2 LT) =====
    ElapsedTime ctvl2Timer = new ElapsedTime();
    boolean ctvl2Ativo = false;
    boolean lt2Last = false;


    boolean shooterOn = false;
    double targetRPM = 0;

    // ===== PID =====
    double powerL = 0, powerR = 0;
    double iL = 0, iR = 0;
    double lastErrL = 0, lastErrR = 0;
    long lastTime;

    final double kP = 0.0006;
    final double kI = 0.0000003;
    final double kD = 0.0002;

    final double POWER_MAX = 1.0;
    final double POWER_MIN = 0.0;

    boolean aLast = false, bLast = false, xLast = false, yLast = false;

    // ===== DPAD GAMEPAD 2 =====
    boolean dpadUpLast = false;
    boolean dpadDownLast = false;
    boolean dpadLeftLast = false;
    boolean dpadRightLast = false;

    @Override
    public void runOpMode() {

        // ===== DRIVE =====
        FL0 = hardwareMap.get(DcMotor.class, "roda0");
        FR1 = hardwareMap.get(DcMotor.class, "roda1");
        BL2 = hardwareMap.get(DcMotor.class, "roda2");
        BR3 = hardwareMap.get(DcMotor.class, "roda3");

        FL0.setDirection(DcMotorSimple.Direction.FORWARD);
        BL2.setDirection(DcMotorSimple.Direction.FORWARD);
        FR1.setDirection(DcMotorSimple.Direction.REVERSE);
        BR3.setDirection(DcMotorSimple.Direction.REVERSE);

        // ===== CTVL =====
        ctvl = hardwareMap.get(DcMotorEx.class, "CTVL");
        ctvl.setDirection(DcMotorSimple.Direction.FORWARD);
        ctvl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ctvl2 = hardwareMap.get(DcMotorEx.class, "CTVL2");
        ctvl2.setDirection(DcMotorSimple.Direction.REVERSE);
        ctvl2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctvl2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== SHOOTERS =====
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {

            // ===== DRIVE =====
            double drive  = -gamepad1.left_stick_y;
            double turn   =  gamepad1.right_stick_x;
            double strafe = 0;

            // ===== STRAFE =====
            if (gamepad1.left_bumper)       strafe = -1.0;
            else if (gamepad1.right_bumper) strafe =  1.0;

            // ===== GIRO PELO DPAD (GAMEPAD1) =====
            if (gamepad1.dpad_left)  turn -= 0.3;
            if (gamepad1.dpad_right) turn += 0.3;

            FL0.setPower(drive + strafe + turn);
            FR1.setPower(drive - strafe - turn);
            BL2.setPower(drive - strafe + turn);
            BR3.setPower(drive + strafe - turn);

            // ===== CTVL (TOGGLE) =====
            boolean rt = gamepad1.right_trigger > 0.5;
            if (rt && !rtLast) ctvlToggle = !ctvlToggle;
            rtLast = rt;

            if (gamepad1.left_trigger > 0.3)
                ctvl.setPower(-1.0);
            else if (ctvlToggle)
                ctvl.setPower(1.0);
            else
                ctvl.setPower(0.0);

            // ===== CTVL2 CONTROLE ÚNICO (GAMEPAD2 RT + LT) =====

            boolean lt2 = gamepad2.left_trigger > 0.5;

// --- Inicia ciclo automático (LT) ---
            if (lt2 && !lt2Last && !ctvl2Ativo) {
                ctvl2Timer.reset();
                ctvl2Ativo = true;
            }

// --- Prioridade TOTAL: ciclo automático ---
            if (ctvl2Ativo) {
                ctvl2.setPower(-1.0);

                if (ctvl2Timer.milliseconds() >= 500) {
                    ctvl2.setPower(0.0);
                    ctvl2Ativo = false;
                }
            }
// --- Manual (RT) só se LT NÃO estiver ativo ---
            else if (gamepad2.right_trigger > 0.3) {
                ctvl2.setPower(-1.0);
            }
            else {
                ctvl2.setPower(0.0);
            }

            lt2Last = lt2;



            // ===== SHOOTER PRESETS =====
            if (gamepad2.a && !aLast) toggleShooter(RPM_A);
            if (gamepad2.b && !bLast) toggleShooter(RPM_B);
            if (gamepad2.x && !xLast) toggleShooter(RPM_X);
            if (gamepad2.y && !yLast) toggleShooter(RPM_Y);

            aLast = gamepad2.a;
            bLast = gamepad2.b;
            xLast = gamepad2.x;
            yLast = gamepad2.y;

            // ===== AJUSTE FINO RPM - DPAD GAMEPAD2 =====
            if (shooterOn) {
                if (gamepad2.dpad_up && !dpadUpLast)    targetRPM += 100;
                if (gamepad2.dpad_down && !dpadDownLast) targetRPM -= 100;
                if (gamepad2.dpad_right && !dpadRightLast) targetRPM += 10;
                if (gamepad2.dpad_left && !dpadLeftLast)  targetRPM -= 10;
                if (targetRPM < 0) targetRPM = 0;
            }

            dpadUpLast = gamepad2.dpad_up;
            dpadDownLast = gamepad2.dpad_down;
            dpadLeftLast = gamepad2.dpad_left;
            dpadRightLast = gamepad2.dpad_right;

            // ===== PID SHOOTER =====
            if (shooterOn) {
                pidShooter();
                shooterL.setPower(powerL);
                shooterR.setPower(powerR);
            } else {
                shooterL.setPower(0);
                shooterR.setPower(0);
            }

            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("RPM L", shooterL.getVelocity());
            telemetry.addData("RPM R", shooterR.getVelocity());
            telemetry.update();
        }
    }

    // ===== PID =====
    void pidShooter() {
        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        if (dt <= 0) dt = 0.01;
        lastTime = now;

        double errL = targetRPM - Math.abs(shooterL.getVelocity());
        double errR = targetRPM - Math.abs(shooterR.getVelocity());

        iL += errL * dt;
        iR += errR * dt;

        double dL = (errL - lastErrL) / dt;
        double dR = (errR - lastErrR) / dt;

        lastErrL = errL;
        lastErrR = errR;

        powerL = Math.max(POWER_MIN,
                Math.min(POWER_MAX, powerL + kP * errL + kI * iL + kD * dL));

        powerR = Math.max(POWER_MIN,
                Math.min(POWER_MAX, powerR + kP * errR + kI * iR + kD * dR));
    }

    // ===== TOGGLE SHOOTER =====
    void toggleShooter(double rpm) {
        shooterOn = !(shooterOn && targetRPM == rpm);
        targetRPM = shooterOn ? rpm : 0;

        powerL = powerR = 0.35;
        iL = iR = 0;
        lastErrL = lastErrR = 0;
        lastTime = System.currentTimeMillis();
    }
}