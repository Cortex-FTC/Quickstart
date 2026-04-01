package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.sleep;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Autonomous(name = "vermelho")
public class teste2 extends OpMode {

    private void ligarCtvl2Tempo(double tempo){
        if(pathTimer.getElapsedTimeSeconds() < tempo){
            ctvl2.setPower(-1);
        }else{
            ctvl2.setPower(0);
        }
    }

    final double RPM_X = 1475;

    private DcMotor shooterR;
    private DcMotor ctvl;
    private DcMotor ctvl2;


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

    private final Pose startPose = new Pose(117.000, 130.395, Math.toRadians(37));
    private final Pose shootpose1 = new Pose(88.535, 101.093, Math.toRadians(28));
    private final Pose artefato1 = new Pose(98.047, 80.628, Math.toRadians(0));
    private final Pose pega1 = new Pose(110.721, 80.372, Math.toRadians(0));
    private final Pose shoot2 = new Pose(98.674, 102.767, Math.toRadians(40));
    private final Pose artefato2 = new Pose(95.442, 56.721, Math.toRadians(0));
    private final Pose pega2 = new Pose(110.977, 56.767, Math.toRadians(0));
    private final Pose gate = new Pose(113.349, 69.140, Math.toRadians(90));
    private final Pose gate2 = new Pose(124.953, 70.535, Math.toRadians(90));
    private final Pose shoot3 = new Pose(103.605, 101.767, Math.toRadians(42));
    private final Pose artefato3 = new Pose(98.953, 32.209, Math.toRadians(0));
    private final Pose pega3 = new Pose(110.302, 32.349, Math.toRadians(0));
    private final Pose shoot4 = new Pose(101.326, 101.605, Math.toRadians(41));
    private final Pose gate4 = new Pose(126.930, 66.442, Math.toRadians(90));

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
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
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


    }

    public void statePathUpdate() {

        switch (pathState) {

            case DRIVE_START_SHOOT1:

                follower.followPath(startShoot1, true);
                setPathState(PathState.ESPERA1);
                break;



            case ESPERA1:
                ctvl.setPower(1);
                ctvl2.setPower(-1);
                ligarCtvl2Tempo(1.400);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(shoot1Artefato1, true);
                    setPathState(PathState.DRIVE_SHOOT1_ARTEFATO1);
                }
                break;

            case DRIVE_SHOOT1_ARTEFATO1:
                follower.followPath(shoot1Artefato1, true);
                setPathState(PathState.DRIVE_ARTEFATO1_PEGA1);
                break;

            case DRIVE_ARTEFATO1_PEGA1:
                ctvl.setPower(1);
                ctvl2.setPower(-1);
                ligarCtvl2Tempo(0.4);

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
                ctvl.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(pega1Shoot2, true);
                    setPathState(PathState.ESPERA3);
                }
                break;

            case ESPERA3:
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
                ctvl2.setPower(-1);
                ligarCtvl2Tempo(0.4);
                ctvl.setPower(1);


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
                ctvl.setPower(0);

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
                ctvl2.setPower(-1);
                ligarCtvl2Tempo(1.400);

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.400) {
                    follower.followPath(shoot3Artefato3, true);
                    setPathState(PathState.DRIVE_SHOOT3_ARTEFATO3);
                }
                break;


            case DRIVE_SHOOT3_ARTEFATO3:
                if (!follower.isBusy()) {
                    follower.followPath(shoot3Artefato3, true);
                    setPathState(PathState.DRIVE_ARTEFATO3_PEGA3);
                }
                break;

            case DRIVE_ARTEFATO3_PEGA3:
                ctvl2.setPower(-1);
                ligarCtvl2Tempo(0.4);
                ctvl.setPower(1);


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
                ctvl.setPower(0);

                if (!follower.isBusy()) {
                    follower.followPath(pega3shoot4, true);
                    setPathState(PathState.ESPERA8);
                }
                break;

            case ESPERA8:
                ctvl2.setPower(-1);
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
                ctvl2.setPower(-1);
                ligarCtvl2Tempo(1.400);
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

        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        ctvl = hardwareMap.get(DcMotor.class, "CTVL");
        ctvl2 = hardwareMap.get(DcMotor.class, "CTVL2");

        ctvl.setDirection(DcMotorSimple.Direction.FORWARD);
        ctvl2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterR.setPower(RPM_X);

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
        statePathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}