package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Autonomous(name = "Azul frente")
public class teste extends OpMode {

//w
    double powerL = 0.35;

    double powerR = 0.35;

    double iL = 0;
    double iR = 0;

    double lastErrL = 0;
    double lastErrR = 0;

    long lastTime;

    double targetRPM = 1290;

    final double kP = 0.0006;
    final double kI = 0.0000003;
    final double kD = 0.0002;

    final double POWER_MAX = 1.0;
    final double POWER_MIN = 0.0;

    private void Ctvl2Entre(double inicio, double fim){
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
        DRIVE_PEGA2_GATE,
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
        CTVL1,
        CTVL2,
        CTVL3,
        CTVL4,
        PARA
    }

    PathState pathState;

    private final Pose startPose = new Pose(26.100, 129.400, Math.toRadians(142));
    private final Pose shootpose1 = new Pose(46.544, 107.692, Math.toRadians(146));
    private final Pose artefato1 = new Pose(46.047, 86.233, Math.toRadians(180));
    private final Pose pega1 = new Pose(29.428, 86.284, Math.toRadians(180));
    private final Pose shoot2 = new Pose(42.698, 91.047, Math.toRadians(128));
    private final Pose artefato2 = new Pose(22.047, 56.823, Math.toRadians(180));
    private final Pose pega2 = new Pose(27.144, 56.879, Math.toRadians(180));
    private final Pose gate = new Pose(47.848, 67.638, Math.toRadians(0));
    private final Pose gate2 = new Pose(19.690, 69.687, Math.toRadians(0));
    private final Pose shoot3 = new Pose(53.700, 81.991, Math.toRadians(129));
    private final Pose artefato3 = new Pose(44.294, 37.782, Math.toRadians(180));
    private final Pose pega3 = new Pose(21.216, 37.926, Math.toRadians(180));
    private final Pose shoot4 = new Pose(54.837, 80.372, Math.toRadians(129));
    private final Pose gate4 = new Pose(16.628, 70.302, Math.toRadians(0));
    private final Pose gate5 = new Pose(31.337, 67.291, Math.toRadians(145));

    private PathChain startShoot1;
    private PathChain shoot1Artefato1;
    private PathChain artefato1Pega1;
    private PathChain pega1Shoot2;
    private PathChain shoot2Artefato2;
    private PathChain artefato2Pega2;
    private PathChain pega2Gate;
    private PathChain gateGate2;
    private PathChain gate2Shoot3;
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

        pega2Gate = follower.pathBuilder() //6
                .addPath(new BezierLine(pega2, gate))
                .setLinearHeadingInterpolation(pega2.getHeading(), gate.getHeading())
                .build();

        gateGate2 = follower.pathBuilder() // path7
                .addPath(new BezierCurve(
                        pega2,gate,gate2
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        gate2Shoot3 = follower.pathBuilder() //8
                .addPath(new BezierLine(gate2, shoot3))
                .setLinearHeadingInterpolation(gate2.getHeading(), shoot3.getHeading())
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

        gate4gate5 = follower.pathBuilder() //12
                .addPath(new BezierLine(gate4,gate5))
                .setLinearHeadingInterpolation(gate4.getHeading(), gate5.getHeading())
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
                ligactvl(5);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(shoot1Artefato1, true);
                    setPathState(PathState.DRIVE_SHOOT1_ARTEFATO1);
                }
                break;

            case ATIVASHOOT:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
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
                Ctvl2Entre(0,0.700);

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
                ligactvl(2);
                ligarCtvl2Tempo(1.400);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.400) {
                    follower.followPath(shoot2Artefato2, true);
                    setPathState(PathState.DRIVE_SHOOT2_ARTEFATO2);
                }
                break;

            case DRIVE_SHOOT2_ARTEFATO2:
                if (!follower.isBusy()) {
                    follower.followPath(shoot2Artefato2, true);
                    setPathState(PathState.DRIVE_ARTEFATO2_PEGA2);
                }
                break;

            case DRIVE_ARTEFATO2_PEGA2:
                ligactvl(3);
                Ctvl2Entre(0,0.700);

                if (!follower.isBusy()) {
                    follower.followPath(artefato2Pega2, true);
                    setPathState(PathState.ESPERA4);
                }
                break;

            case ESPERA4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.200) {
                    follower.followPath(pega2Gate, true);
                    setPathState(PathState.DRIVE_PEGA2_GATE);
                }
                break;

            case DRIVE_PEGA2_GATE:
                if (!follower.isBusy()) {
                    follower.followPath(pega2Gate, true);
                    setPathState(PathState.DRIVE_GATE_GATE2);
                }
                break;

            case DRIVE_GATE_GATE2:
                if (!follower.isBusy()) {
                    follower.followPath(gateGate2, true);
                    setPathState(PathState.ESPERA5);
                }
                break;

            case ESPERA5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.200) {
                    follower.followPath(gate2Shoot3, true);
                    setPathState(PathState.DRIVE_GATE2_SHOOT3);
                }
                break;

            case DRIVE_GATE2_SHOOT3:
                if (!follower.isBusy()) {
                    follower.followPath(gate2Shoot3, true);
                    setPathState(PathState.ESPERA6);
                }
                break;

            case ESPERA6:
                ligactvl(2);
                ligarCtvl2Tempo(1.400);

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
                Ctvl2Entre(0,0.700);


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
                ligarCtvl2Tempo(1.400);

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.400) {
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

            case ESPERA9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.400) {
                    follower.followPath(gate4gate5, true);
                    setPathState(PathState.DRIVE_GATE4_GATE5);
                }
                break;

            case DRIVE_GATE4_GATE5:
                if (!follower.isBusy()) {
                    follower.followPath(gate4gate5, true);
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
        ctvl.setPower(1.0);
        statePathUpdate();
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