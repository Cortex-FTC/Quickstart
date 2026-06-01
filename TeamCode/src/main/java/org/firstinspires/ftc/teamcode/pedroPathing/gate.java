package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Azul gate")//si
public class gate extends OpMode {

    double powerL = 0.35;

    double powerR = 0.35;

    double iL = 0;
    double iR = 0;

    double lastErrL = 0;
    double lastErrR = 0;

    long lastTime;

    double targetRPM = 1350;

    final double kP = 0.0006;
    final double kI = 0.0000003;
    final double kD = 0.0002;

    final double POWER_MAX = 1.0;
    final double POWER_MIN = 0.0;

    private void ctvl2meio(double inicio, double fim){
        double t = pathTimer.getElapsedTimeSeconds();

        if(t > inicio && t < fim){
            ctvl2.setPower(-1);
        } else {
            ctvl2.setPower(0);
        }
    }

    private void desligarCtvl2Tempo(double tempoMs){
        if(pathTimer.getElapsedTimeSeconds() > tempoMs){
            ctvl2.setPower(-1);
        } else {
            ctvl2.setPower(0);
        }
    }
    private void ligarCtvl2Tempo(double tempoMs){
        if(pathTimer.getElapsedTimeSeconds() < tempoMs){
            ctvl2.setPower(-1);
        }else{
            ctvl2.setPower(0);
        }
    }

    private void ligactvl(double tempo){
        if(pathTimer.getElapsedTimeSeconds() < tempo){
            ctvl.setPower(1);
        }else{
            ctvl.setPower(0);
        }
    }
    private DcMotorEx shooterR;
    private DcMotorEx shooterL;
    private DcMotorEx ctvl;
    private DcMotorEx ctvl2;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_START_SHOOT1,
        ATIVASHOOT,
        ESPERA1,
        DRIVE_SHOOT1_ARTEFATO1,
        ESPERA2,
        DRIVE_ARTEFATO1_PEGA1,
        DRIVE_PEGA1_SHOOT2,
        ESPERA3,
        DRIVE_SHOOT2_PEGA2,
        DRIVE_PEGA2_Shoot3,
        ESPERA4,
        DRIVE_Shoot3_Pega3,
        ESPERA6,
        DRIVE_Pega3_Shoot4,
        DRIVE_Shoot4_Artefato1,
        ESPERA7,
        DRIVE_Artefato1_pega4,
        ESPERA8,
        DRIVE_pega4_Shoot5,
        ESPERA5,
        ESPERA9,
        ESPERA10,
        CTVL2,
        CTVL3,
        CTVL4,
        PARA
    }

    PathState pathState;

    private final Pose startPose = new Pose(27.265, 131.294, Math.toRadians(143));
    private final Pose shootpose1 = new Pose(49.500, 103.500, Math.toRadians(130));
    private final Pose artefato2 = new Pose(46.706, 56.353, Math.toRadians(180));
    private final Pose pega1 = new Pose(13.471, 56.412, Math.toRadians(180));
    private final Pose shoot2 =     new Pose(57.971, 78.088, Math.toRadians(135));
    private final Pose pega2 = new Pose(12.706, 62.471, Math.toRadians(147));
    private final Pose shoot3 = new Pose(56.971, 86.029, Math.toRadians(135));
    private final Pose pega3 = new Pose(16.147, 64.059, Math.toRadians(143));
    private final Pose shoot4 = new Pose(56.824, 85.647, Math.toRadians(133));
    private final Pose artefato1 = new Pose(44.471, 85.765, Math.toRadians(180));
    private final Pose pega4 = new Pose(20.647, 85.941, Math.toRadians(180));
    private final Pose shoot5 = new Pose(43.412, 93.441, Math.toRadians(134));


    private PathChain startShoot1;
    private PathChain shoot1Artefato1;
    private PathChain artefato2Pega1;
    private PathChain pega1Shoot2;
    private PathChain shoot2pega2;
    private PathChain pega2shoot3;
    private PathChain shoot3pega3;
    private PathChain pega3shoot4;
    private PathChain shoot4artefato1;
    private PathChain artefato1pega4;
    private PathChain pega4shoot5;


    public void buildPaths() {

        startShoot1 = follower.pathBuilder() //1
                .addPath(new BezierLine(startPose, shootpose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootpose1.getHeading())
                .build();

        shoot1Artefato1 = follower.pathBuilder() //2
                .addPath(new BezierLine(shootpose1, artefato2))
                .setLinearHeadingInterpolation(shootpose1.getHeading(), artefato2.getHeading())
                .build();

        artefato2Pega1 = follower.pathBuilder() //3
                .addPath(new BezierLine(artefato2, pega1))
                .setLinearHeadingInterpolation(artefato2.getHeading(), pega1.getHeading())
                .build();

        pega1Shoot2 = follower.pathBuilder() //4
                .addPath(new BezierLine(pega1, shoot2))
                .setLinearHeadingInterpolation(pega1.getHeading(), shoot2.getHeading())
                .build();

        shoot2pega2 = follower.pathBuilder() //4
                .addPath(new BezierLine(shoot2, pega2))
                .setLinearHeadingInterpolation(shoot2.getHeading(), pega2.getHeading())
                .build();

        pega2shoot3 = follower.pathBuilder() //5
                .addPath(new BezierLine(pega2, shoot3))
                .setLinearHeadingInterpolation(artefato2.getHeading(), pega2.getHeading())
                .build();

        shoot3pega3 = follower.pathBuilder() //6
                .addPath(new BezierLine(shoot3, pega3))
                .setLinearHeadingInterpolation(shoot3.getHeading(), pega3.getHeading())
                .build();


        pega3shoot4 = follower.pathBuilder() //9
                .addPath(new BezierLine(pega3, shoot4))
                .setLinearHeadingInterpolation(pega3.getHeading(), shoot4.getHeading())
                .build();

        shoot4artefato1 = follower.pathBuilder() //10
                .addPath(new BezierLine(shoot4,artefato1))
                .setLinearHeadingInterpolation(shoot4.getHeading(), artefato1.getHeading())
                .build();


        artefato1pega4 = follower.pathBuilder() //11
                .addPath(new BezierLine(artefato1,pega4))
                .setLinearHeadingInterpolation(artefato1.getHeading(), pega4.getHeading())
                .build();

        pega4shoot5 = follower.pathBuilder() //12
                .addPath(new BezierLine(pega4,shoot5))
                .setLinearHeadingInterpolation(pega4.getHeading(), shoot5.getHeading())
                .build();


    }

    public void pidShooter(){

        long now = System.currentTimeMillis();

        double dt = (now - lastTime)/1000.0;
        if(dt <= 0) dt = 0.01;

        lastTime = now;

        double errL = targetRPM - Math.abs(shooterL.getVelocity());
        double errR = targetRPM - Math.abs(shooterR.getVelocity());

        iL += errL * dt;
        iR += errR * dt;

        final double I_MAX = 5000;

        iL = Math.max(-I_MAX, Math.min(I_MAX, iL));
        iR = Math.max(-I_MAX, Math.min(I_MAX, iR));

        double dL = (errL - lastErrL)/dt;
        double dR = (errR - lastErrR)/dt;

        lastErrL = errL;
        lastErrR = errR;

        powerL += kP*errL + kI*iL + kD*dL;
        powerR += kP*errR + kI*iR + kD*dR;

        powerL = Math.max(POWER_MIN, Math.min(POWER_MAX, powerL));
        powerR = Math.max(POWER_MIN, Math.min(POWER_MAX, powerR));

        shooterL.setPower(powerL);
        shooterR.setPower(powerR);
    }

    public void statePathUpdate() {

        switch (pathState) {

            case DRIVE_START_SHOOT1:
                follower.followPath(startShoot1, true);
                setPathState(PathState.ESPERA1);
                break;

            case ESPERA1:
                desligarCtvl2Tempo(2);
                ligactvl(4);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(shoot1Artefato1, true);
                    setPathState(PathState.DRIVE_SHOOT1_ARTEFATO1);
                }
                break;



            case DRIVE_SHOOT1_ARTEFATO1:
                ctvl2.setPower(0);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.200) {
                    follower.followPath(shoot1Artefato1, true);
                    setPathState(PathState.DRIVE_ARTEFATO1_PEGA1);
                }
                break;

            case DRIVE_ARTEFATO1_PEGA1:
                ligactvl(3);
                if (!follower.isBusy()) {
                    follower.followPath(artefato2Pega1, true);
                    setPathState(PathState.ESPERA2);
                }
                break;

            case ESPERA2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.200) {
                    follower.followPath(pega1Shoot2, true);
                    setPathState(PathState.DRIVE_PEGA1_SHOOT2);
                }
                break;

            case DRIVE_PEGA1_SHOOT2:
                if (!follower.isBusy()) {
                    follower.followPath(pega1Shoot2, true);
                    setPathState(PathState.ESPERA3);
                }
                break;

            case ESPERA3:
                ligactvl(1.400);
                ligarCtvl2Tempo(1.4);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.400) {
                    ctvl2.setPower(0);
                    follower.followPath(shoot2pega2, true);
                    setPathState(PathState.DRIVE_SHOOT2_PEGA2);
                }
                break;

            case DRIVE_SHOOT2_PEGA2:
                ctvl2.setPower(0);
                ligactvl(3);
                if (!follower.isBusy()) {
                    follower.followPath(shoot2pega2, true);
                    setPathState(PathState.ESPERA9);
                }
                break;

            case ESPERA9:
                ligactvl(3);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(pega2shoot3, true);
                    setPathState(PathState.DRIVE_Shoot3_Pega3);
                }
                break;

            case DRIVE_PEGA2_Shoot3:
                if (!follower.isBusy()) {
                    follower.followPath(pega2shoot3, true);
                    setPathState(PathState.ESPERA4);
                }
                break;

            case ESPERA4:
                ligactvl(1.400);
                ligarCtvl2Tempo(1.4);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 10) {
                    follower.followPath(shoot3pega3, true);
                    setPathState(PathState.DRIVE_Shoot3_Pega3);
                }
                break;


            case DRIVE_Shoot3_Pega3:
                ligactvl(3);
                if (!follower.isBusy()) {
                    follower.followPath(shoot3pega3, true);
                    setPathState(PathState.ESPERA6);
                }
                break;

            case ESPERA6:
                ligactvl(3);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(pega3shoot4, true);
                    setPathState(PathState.DRIVE_Pega3_Shoot4);
                }
                break;


            case DRIVE_Pega3_Shoot4:
                if (!follower.isBusy()) {
                    follower.followPath(pega3shoot4, true);
                    setPathState(PathState.ESPERA5);
                }
                break;

            case ESPERA5:
                ligactvl(1.400);
                ligarCtvl2Tempo(1.4);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.400) {
                    follower.followPath(shoot4artefato1, true);
                    setPathState(PathState.DRIVE_Shoot4_Artefato1);
                }
                break;

            case DRIVE_Shoot4_Artefato1:
                if (!follower.isBusy()) {
                    follower.followPath(shoot4artefato1, true);
                    setPathState(PathState.ESPERA7);
                }
                break;

            case ESPERA7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.200) {
                    follower.followPath(artefato1pega4, true);
                    setPathState(PathState.DRIVE_Artefato1_pega4);
                }
                break;

            case DRIVE_Artefato1_pega4:
                ligactvl(3);
                ctvl2.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(artefato1pega4, true);
                    setPathState(PathState.DRIVE_pega4_Shoot5);
                }
                break;


            case DRIVE_pega4_Shoot5:
                ctvl2.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(pega4shoot5, true);
                    setPathState(PathState.ESPERA8);
                }
                break;

            case ESPERA8:
                ligactvl(1.400);
                ligarCtvl2Tempo(1.400);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 10) {
                    follower.followPath(pega4shoot5, true);
                    setPathState(PathState.PARA);
                }
                break;

            case PARA:
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        ctvl = hardwareMap.get(DcMotorEx.class,"CTVL");
        ctvl2 = hardwareMap.get(DcMotorEx.class,"CTVL2");

        ctvl.setDirection(DcMotorSimple.Direction.FORWARD);
        ctvl2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);

        ctvl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctvl2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pathState = PathState.DRIVE_START_SHOOT1;

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setPose(startPose);

        pathState = PathState.DRIVE_START_SHOOT1;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        pidShooter();
        statePathUpdate();
        // Se não estiver em um estado de espera, desliga o shooter
        if (pathState != PathState.ESPERA1 &&
                pathState != PathState.ESPERA3 &&
                pathState != PathState.ESPERA6 &&
                pathState != PathState.ESPERA8) {

            ctvl2.setPower(0);
        }

        if (pathState != PathState.ESPERA1 &&
                pathState != PathState.ESPERA3 &&
                pathState != PathState.ESPERA6 &&
                pathState != PathState.ESPERA8) {

            shooterR.setPower(0);
            shooterL.setPower(0);
        }

        telemetry.addData("Target RPM",targetRPM);
        telemetry.addData("RPM L",shooterL.getVelocity());
        telemetry.addData("RPM R",shooterR.getVelocity());
        telemetry.addData("Power L",powerL);
        telemetry.addData("Power R",powerR);
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}