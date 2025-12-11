package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

            waitForStart();
            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            // small forward + slight left curve
                            .splineTo(new Vector2d(12, 4), Math.PI / 4)
                            .splineTo(new Vector2d(18, 8), Math.PI / 2)
                            .build()
            );

        } else if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)) {
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

            waitForStart();
            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(12, 4), Math.PI / 4)
                            .splineTo(new Vector2d(18, 8), Math.PI / 2)
                            .build()
            );

        } else if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();
            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(12, 4), Math.PI / 4)
                            .splineTo(new Vector2d(18, 8), Math.PI / 2)
                            .build()
            );

        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();
            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(12, 4), Math.PI / 4)
                            .splineTo(new Vector2d(18, 8), Math.PI / 2)
                            .build()
            );
        } else {
            throw new RuntimeException();
        }
    }
}