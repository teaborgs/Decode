package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.zyxOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.messages.PoseMessage;

/**
 * Pinpoint-based drive localization for RR quickstart 2025+.
 */
@Config
public class PinpointDrive extends MecanumDrive {
    public static class Params {
        public String pinpointDeviceName = "pinpoint";

        // Pinpoint driver expects offsets in mm
        public double xOffset = 80;
        public double yOffset = -100;

        // ticks per millimeter
        public double encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        public GoBildaPinpointDriver.EncoderDirection xDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public GoBildaPinpointDriver.EncoderDirection yDirection =
                GoBildaPinpointDriver.EncoderDirection.REVERSED;

        public boolean usePinpointIMUForTuning = false;

        // If right turn shows negative heading and you want the opposite, set true
        public boolean invertHeading = false;
    }

    public static Params PARAMS = new Params();

    public GoBildaPinpointDriverRR pinpoint;

    // Sync marker to detect external pose resets
    private Pose2d lastPinpointPose = pose;

    public PinpointDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);

        FlightRecorder.write("PINPOINT_PARAMS", PARAMS);

        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, PARAMS.pinpointDeviceName);

        if (PARAMS.usePinpointIMUForTuning) {
            lazyImu = new LazyImu(
                    hardwareMap,
                    PARAMS.pinpointDeviceName,
                    new RevHubOrientationOnRobot(zyxOrientation(0, 0, 0))
            );
        }

        pinpoint.setOffsets(PARAMS.xOffset, PARAMS.yOffset);
        pinpoint.setEncoderResolution(PARAMS.encoderResolution);
        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);

        pinpoint.resetPosAndIMU();

        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        pinpoint.setPosition(pose);
    }

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        // If pose was externally reset, sync Pinpoint
        if (lastPinpointPose != pose) {
            pinpoint.setPosition(pose);
        }

        pinpoint.update();

        Pose2d ppPose = pinpoint.getPositionRR();

        Rotation2d h = ppPose.heading;
        if (PARAMS.invertHeading) {
            h = h.inverse(); // inversare rotație robustă, fără glitch-uri de wrap
        }

        pose = new Pose2d(ppPose.position, h);


        lastPinpointPose = pose;

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
        FlightRecorder.write("PINPOINT_STATUS", pinpoint.getDeviceStatus());

        PoseVelocity2d v = pinpoint.getVelocityRR();
        if (PARAMS.invertHeading) {
            return new PoseVelocity2d(v.linearVel, -v.angVel);
        } else {
            return v;
        }
    }

}
