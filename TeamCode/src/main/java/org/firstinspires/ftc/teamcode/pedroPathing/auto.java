package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
@Disabled
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class auto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(27.265, 131.294, Math.toRadians(143))
        );
        paths = new Paths(follower); // Build paths
        follower = Constants.createFollower(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(27.265, 131.294),
                                    new Pose(49.500, 103.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(142))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(49.500, 103.500),
                                    new Pose(46.706, 56.353)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(46.706, 56.353),
                                    new Pose(21.147, 56.412)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(21.147, 56.412),
                                    new Pose(57.971, 78.088)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(57.971, 78.088),
                                    new Pose(21.110, 59.704)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(165))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(21.110, 59.704),
                                    new Pose(56.971, 86.029)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(165), Math.toRadians(135))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.971, 86.029),
                                    new Pose(19.283, 58.051)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(159))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(19.283, 58.051),
                                    new Pose(57.029, 86.673)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(164))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(57.029, 86.673),
                                    new Pose(20.647, 85.941)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(164), Math.toRadians(180))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(20.647, 85.941),
                                    new Pose(43.412, 93.441)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
            switch (pathState) {

                case 0:
                    follower.followPath(paths.Path1);
                    return 1;

                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path2);
                        return 2;
                    }
                    break;

                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path3);
                        return 3;
                    }
                    break;

                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path4);
                        return 4;
                    }
                    break;

                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path5);
                        return 5;
                    }
                    break;

                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path6);
                        return 6;
                    }
                    break;

                case 6:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path7);
                        return 7;
                    }
                    break;

                case 7:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path8);
                        return 8;
                    }
                    break;

                case 8:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path9);
                        return 9;
                    }
                    break;

                case 9:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path10);
                        return 10;
                    }
                    break;
            }

            return pathState;
        }
    }
