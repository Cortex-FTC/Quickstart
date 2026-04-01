package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import com.pedropathing.util.Timer;

@Disabled
@Autonomous(name = "Azul frente")
public class auto extends OpMode {




    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public enum PathState {
        DRIVE_START_SHOOT1,
        SHOOT_PRELOAD,
        DRIVE_SHOOT1_ARTEFATO,
        DRIVE_ARTEFATO_PEGA1,
        DRIVE_PEGA1_SHOOT2,
        DRIVE_SHOOT2POS_ARTEFATO2POS,
        DRIVE_ARTEFATO2POS_PEGA2,

        DRIVE_ARTEFATO2POS_GATEPOS,

        DRIVE_ARTEFATO2POS_GATE3POS,
        DRIVE_GATE3POS_SHOOT3POS,
        DRIVE_SHOOT3POS_ARTEFATO3,
        DRIVEARTEFATO3_PEGA3,
        DRIVEPEGA3_SHOOT4POS,
        DRIVE_PARA;

    }

    PathState pathState;

    private final Pose startPose = new Pose(26.1, 129.400,Math.toRadians(142));
    private final Pose shootpose1 = new Pose(46.544, 107.692,Math.toRadians(150));
    private final Pose artefato1 = new Pose(41.860,86.860,Math.toRadians(180));
    private final Pose pega1 = new Pose(19.800,86.702,Math.toRadians(180));
    private final Pose shoot2 = new Pose(42.698,91.047,Math.toRadians(128));
    private final Pose artefato2 = new Pose(43.940,62.574,Math.toRadians(180));
    private final Pose pega2 = new Pose(19.535,62.893);
    private final Pose gate = new Pose(27.589,71.215,Math.toRadians(90));
    private final Pose gate2 = new Pose(16.341,69.268,Math.toRadians(90));
    private final Pose shoot3 = new Pose(53.700,81.991,Math.toRadians(129));
    private final Pose artefato3 = new Pose(41.364,39.247,Math.toRadians(180));
    private final Pose pega3 = new Pose(20.169,38.973);
    private final Pose shoot4 = new Pose(54.837,80.372,Math.toRadians(129));
    private final Pose gated = new Pose(16.628,70.302,Math.toRadians(90));
    private final Pose gated2 = new Pose(31.337,67.291,Math.toRadians(90));
    private final Pose coleta = new Pose(13.047,61.070,Math.toRadians(145));



    private PathChain driveStartPosShootPos, driveShootPosArtefato1Pos, driveArtefato1PosPega1Pos,
            driveartefato2Pospega2Pos,
            drivePega1PosShoot2Pos, driveShoot2PosArtefato2Pos, driveArtefato2PosGatePos,driveArtefato2PosGate3Pos,
            driveGate3PosShoot3Pos, driveShoot3PosArtefato3,driveShoot4PosArtefato3Pos,driveArtefato3Pega3,
            drivePega3Shoot4,driveShoot4Gated,driveGatedColeta;

    public void buildPaths() {

        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootpose1))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootpose1.getHeading())
                .build();
        driveShootPosArtefato1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootpose1, artefato1))
                .setLinearHeadingInterpolation(shootpose1.getHeading(),artefato1.getHeading())
                .build();
        driveArtefato1PosPega1Pos = follower.pathBuilder()
                .addPath(new BezierLine(artefato1, pega1))
                .setTangentHeadingInterpolation()
                .build();
        drivePega1PosShoot2Pos = follower.pathBuilder()
                .addPath(new BezierLine(pega1, shoot2))
                .setLinearHeadingInterpolation(pega1.getHeading(),shoot2.getHeading())
                .build();
        driveShoot2PosArtefato2Pos = follower.pathBuilder()
                .addPath(new BezierLine(shoot2, artefato2))
                .setLinearHeadingInterpolation(shoot2.getHeading(),artefato2.getHeading())
                .build();
        driveartefato2Pospega2Pos = follower.pathBuilder()
                .addPath(new BezierLine(artefato2,pega2))
                .setLinearHeadingInterpolation(shoot2.getHeading(),pega2.getHeading())
                .build();
        driveArtefato2PosGatePos = follower.pathBuilder()
                .addPath(new BezierLine( pega2,gate))
                .setLinearHeadingInterpolation(pega2.getHeading(),gate.getHeading())
                .build();
        driveArtefato2PosGatePos = follower.pathBuilder()
                .addPath(new BezierCurve(gate, gate2))
                .setLinearHeadingInterpolation(gate.getHeading(),gate2.getHeading())
                .build();
        driveGate3PosShoot3Pos = follower.pathBuilder()
                .addPath(new BezierLine(gate2, shoot3))
                .setLinearHeadingInterpolation(gate2.getHeading(),shoot3.getHeading())
                .build();
        driveShoot3PosArtefato3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3, artefato3))
                .setLinearHeadingInterpolation(shoot3.getHeading(),artefato3.getHeading())
                .build();
        driveShoot4PosArtefato3Pos = follower.pathBuilder()
                .addPath(new BezierLine( artefato3,pega3))
                .setTangentHeadingInterpolation()
                .build();
        driveArtefato3Pega3 = follower.pathBuilder()
                .addPath(new BezierLine(artefato3, pega3))
                .setLinearHeadingInterpolation(artefato3.getHeading(), shoot4.getHeading())
                .build();
        drivePega3Shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(pega3,shoot4))
                .setLinearHeadingInterpolation(pega3.getHeading(), shoot4.getHeading())
                .build();


    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_SHOOT1:
                if (!follower.isBusy()) {
                    follower.followPath(driveStartPosShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;



            case SHOOT_PRELOAD:
                follower.breakFollowing();
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosArtefato1Pos, true);
                    setPathState(PathState.DRIVE_ARTEFATO_PEGA1);
                }
                break;

            case DRIVE_SHOOT1_ARTEFATO:
                if (!follower.isBusy()) {
                    follower.followPath(driveArtefato1PosPega1Pos, true);
                    setPathState(PathState.DRIVE_ARTEFATO_PEGA1);
                }
                break;

            case DRIVE_ARTEFATO_PEGA1:
                if (!follower.isBusy()) {
                    follower.followPath(drivePega1PosShoot2Pos, true);
                    setPathState(PathState.DRIVE_PEGA1_SHOOT2);
                }
                break;

            case DRIVE_PEGA1_SHOOT2:
                if (!follower.isBusy()) {
                    follower.followPath(driveShoot2PosArtefato2Pos, true);
                    setPathState(PathState.DRIVE_ARTEFATO2POS_PEGA2);
                }
                break;

            case DRIVE_ARTEFATO2POS_PEGA2:
                if (!follower.isBusy()) {
                    follower.followPath(driveShoot2PosArtefato2Pos, true);
                    setPathState(PathState.DRIVE_SHOOT2POS_ARTEFATO2POS);
                }
                break;

            case DRIVE_SHOOT2POS_ARTEFATO2POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveArtefato2PosGate3Pos, true);
                    setPathState(PathState.DRIVE_ARTEFATO2POS_GATE3POS);
                }
                break;

            case DRIVE_ARTEFATO2POS_GATE3POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveGate3PosShoot3Pos, true);
                    setPathState(PathState.DRIVE_GATE3POS_SHOOT3POS);
                }
                break;

            case DRIVE_GATE3POS_SHOOT3POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveShoot3PosArtefato3, true);
                    setPathState(PathState.DRIVE_SHOOT3POS_ARTEFATO3);
                }
                break;

            case DRIVE_SHOOT3POS_ARTEFATO3:
                if (!follower.isBusy()) {
                    follower.followPath(driveShoot4PosArtefato3Pos, true);
                    setPathState(PathState.DRIVEARTEFATO3_PEGA3);
                }
                break;


            case DRIVEARTEFATO3_PEGA3:
                if (!follower.isBusy()) {
                    follower.followPath(driveArtefato3Pega3, true);
                    setPathState(PathState.DRIVEPEGA3_SHOOT4POS);
                }
                break;

            case DRIVEPEGA3_SHOOT4POS:
                if (!follower.isBusy()) {
                    follower.followPath(drivePega3Shoot4, true);
                    setPathState(PathState.DRIVE_PARA);
                }
                break;

        }
    }
    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_START_SHOOT1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);


        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state",pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("Path time",pathTimer.getElapsedTimeSeconds());



    }
}