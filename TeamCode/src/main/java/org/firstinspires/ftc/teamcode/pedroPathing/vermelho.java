package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Autonomous(name = "vermelho frente1")//si
public class vermelho extends OpMode {

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
        DRIVE_SHOOT2_ARTEFATO2,
        DRIVE_ARTEFATO2_PEGA2,
        ESPERA4,
        DRIVE_PEGA2_Shoot2,
        DRIVE_GATE_GATE2,
        ESPERA5,
        DRIVE_GATE2_SHOOT3,
        ESPERA6,
        DRIVE_SHOOT3_ARTEFATO3,
        DRIVE_ARTEFATO3_PEGA3,
        ESPERA7,
        DRIVE_PEGA3_SHOOT4,
        ESPERA8,
        DRIVE_SHOOT4_GATE4,
        ESPERA9,
        DRIVE_GATE4_GATE5,
        DRIVE_GATE1_GATE2,

        CTVL1,
        CTVL2,
        CTVL3,
        CTVL4,
        PARA
    }

    PathState pathState;

    private final Pose startPose = new Pose(117.000, 130.395, Math.toRadians(37));
    private final Pose shootpose1 = new Pose(88.535, 101.093, Math.toRadians(36));
    private final Pose artefato1 = new Pose(98.047, 80.628, Math.toRadians(0));
    private final Pose pega1 = new Pose(124.221, 80.637, Math.toRadians(0));
    private final Pose shoot2 = new Pose(98.674, 102.767, Math.toRadians(44));
    private final Pose artefato2 = new Pose(95.442, 56.721, Math.toRadians(0));
    private final Pose pega2 = new Pose(127.918, 56.767, Math.toRadians(0));

    private final Pose gate1 = new Pose(107.261, 62.257, Math.toRadians(90));
    private final Pose gate2 = new Pose(127.853, 72.265, Math.toRadians(90));

    private final Pose shoot3 = new Pose(103.605, 101.767, Math.toRadians(46));
    private final Pose artefato3 = new Pose(98.953, 32.209, Math.toRadians(0));
    private final Pose pega3 = new Pose(133.596, 32.349, Math.toRadians(0));
    private final Pose shoot4 = new Pose(101.326, 101.605, Math.toRadians(44));
    private final Pose gate4 = new Pose(126.930, 66.442, Math.toRadians(90));


    private PathChain startShoot1;
    private PathChain shoot1Artefato1;
    private PathChain artefato1Pega1;
    private PathChain pega1Shoot2;
    private PathChain shoot2Artefato2;
    private PathChain artefato2Pega2;
    private PathChain pega2shoot2;

    private PathChain gate1gate2;
    private PathChain shoot3Artefato3;
    private PathChain Artefato3pega3;
    private PathChain pega3shoot4;
    private PathChain shoot4gate4;
    private PathChain gate4gate5;

    public void buildPaths() {

        startShoot1 = follower.pathBuilder() //1
                .addPath(new BezierLine(startPose, shootpose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootpose1.getHeading())
                .build();

        shoot1Artefato1 = follower.pathBuilder() //2
                .addPath(new BezierLine(shootpose1, artefato1))
                .setLinearHeadingInterpolation(shootpose1.getHeading(), artefato1.getHeading())
                .build();

        artefato1Pega1 = follower.pathBuilder() //3
                .addPath(new BezierLine(artefato1, pega1))
                .setLinearHeadingInterpolation(artefato1.getHeading(), pega1.getHeading())
                .build();

        pega1Shoot2 = follower.pathBuilder() //4
                .addPath(new BezierLine(pega1, shoot2))
                .setLinearHeadingInterpolation(pega1.getHeading(), shoot2.getHeading())
                .build();

        shoot2Artefato2 = follower.pathBuilder() //4
                .addPath(new BezierLine(shoot2, artefato2))
                .setLinearHeadingInterpolation(shoot2.getHeading(), artefato2.getHeading())
                .build();

        artefato2Pega2 = follower.pathBuilder() //5
                .addPath(new BezierLine(artefato2, pega2))
                .setLinearHeadingInterpolation(artefato2.getHeading(), pega2.getHeading())
                .build();

        gate1gate2= follower.pathBuilder() // 5
                .addPath(new BezierCurve(
                        pega2,gate1,gate2
                ))
                .setLinearHeadingInterpolation(pega2.getHeading(), gate1.getHeading(), gate2.getHeading())
                .build();

        pega2shoot2 = follower.pathBuilder() //6
                .addPath(new BezierLine(pega2, shoot3))
                .setLinearHeadingInterpolation(pega2.getHeading(), shoot3.getHeading())
                .build();


        shoot3Artefato3 = follower.pathBuilder() //9
                .addPath(new BezierLine(shoot3, artefato3))
                .setLinearHeadingInterpolation(shoot3.getHeading(), artefato3.getHeading())
                .build();

        Artefato3pega3 = follower.pathBuilder() //10
                .addPath(new BezierLine(artefato3,pega3))
                .setLinearHeadingInterpolation(artefato3.getHeading(), pega3.getHeading())
                .build();


        pega3shoot4 = follower.pathBuilder() //11
                .addPath(new BezierLine(pega3,shoot4))
                .setLinearHeadingInterpolation(pega3.getHeading(), shoot4.getHeading())
                .build();

        shoot4gate4 = follower.pathBuilder() //12
                .addPath(new BezierLine(shoot4,gate4))
                .setLinearHeadingInterpolation(shoot4.getHeading(), gate4.getHeading())
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
                    follower.followPath(artefato1Pega1, true);
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
                    follower.followPath(shoot2Artefato2, true);
                    setPathState(PathState.DRIVE_SHOOT2_ARTEFATO2);
                }
                break;

            case DRIVE_SHOOT2_ARTEFATO2:
                ctvl2.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(shoot2Artefato2, true);
                    setPathState(PathState.DRIVE_ARTEFATO2_PEGA2);
                }
                break;

            case DRIVE_ARTEFATO2_PEGA2:
                ctvl2.setPower(0);
                ligactvl(3);
                if (!follower.isBusy()) {
                    follower.followPath(artefato2Pega2, true);
                    setPathState(PathState.DRIVE_GATE1_GATE2);
                }
                break;

            case DRIVE_GATE1_GATE2:
                if (!follower.isBusy()) {
                    follower.followPath(gate1gate2, true);
                    setPathState(PathState.ESPERA4);
                }
                break;

            case ESPERA4:
                ctvl2.setPower(0);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(pega2shoot2, true);
                    setPathState(PathState.DRIVE_PEGA2_Shoot2);
                }
                break;




            case DRIVE_PEGA2_Shoot2:
                if (!follower.isBusy()) {
                    follower.followPath(pega2shoot2, true);
                    setPathState(PathState.ESPERA6);
                }
                break;
//

            case ESPERA6:
                ligactvl(1.4);
                ligarCtvl2Tempo(1.4);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.400) {
                    follower.followPath(shoot3Artefato3, true);
                    setPathState(PathState.DRIVE_SHOOT3_ARTEFATO3);
                }
                break;


            case DRIVE_SHOOT3_ARTEFATO3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.200) {
                    follower.followPath(shoot3Artefato3, true);
                    setPathState(PathState.DRIVE_ARTEFATO3_PEGA3);
                }
                break;

            case DRIVE_ARTEFATO3_PEGA3:
                ligactvl(3);
                if (!follower.isBusy()) {
                    follower.followPath(Artefato3pega3, true);
                    setPathState(PathState.ESPERA7);
                }
                break;

            case ESPERA7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.200) {
                    follower.followPath(pega3shoot4, true);
                    setPathState(PathState.DRIVE_PEGA3_SHOOT4);
                }
                break;

            case DRIVE_PEGA3_SHOOT4:
                if (!follower.isBusy()) {
                    follower.followPath(pega3shoot4, true);
                    setPathState(PathState.ESPERA8);
                }
                break;

            case ESPERA8:
                ligactvl(2);
                ligarCtvl2Tempo(2);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(shoot4gate4, true);
                    setPathState(PathState.DRIVE_SHOOT4_GATE4);
                }
                break;

            case DRIVE_SHOOT4_GATE4:
                if (!follower.isBusy()) {
                    follower.followPath(shoot4gate4, true);
                    setPathState(PathState.ESPERA9);
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