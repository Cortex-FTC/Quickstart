package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .centripetalScaling(0.001)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.009, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, 0.04))
            .forwardZeroPowerAcceleration(-42.370342249757684)
            .lateralZeroPowerAcceleration(-65.7899283960854)
            .mass(9.5);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(59.0829754134761676)
            .yVelocity(36.4913052286924566)
            .leftFrontMotorName("roda0")
            .rightFrontMotorName("roda1")
            .leftRearMotorName("roda2")
            .rightRearMotorName("roda3")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardPodY(3.54331)
            .strafePodX(-4.92126)
            .forwardTicksToInches(0.0030150324493385911)
            .strafeTicksToInches(0.0030152474360586539)
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardEncoder_HardwareMapName("roda0")
            .strafeEncoder_HardwareMapName("roda2")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                    )
            );
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}