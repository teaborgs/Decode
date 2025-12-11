package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous(name = "SplineTestTuner", group = "tuning")
public final class SplineTestTuner extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		Pose2d beginPose = new Pose2d(0, 0, 0);

		if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
			// --------- PINPOINT / MECANUM TUNING PARAMS ----------
			// Speeds/accels
			MecanumDrive.PARAMS.maxWheelVel     = 30.0;           // in/s
			MecanumDrive.PARAMS.maxProfileAccel = 25.0;           // in/s^2

			// Angular limits
			MecanumDrive.PARAMS.maxAngVel   = Math.PI / 3.0;      // ~60 deg/s
			MecanumDrive.PARAMS.maxAngAccel = Math.PI / 3.0;

			// Controller gains
			MecanumDrive.PARAMS.axialGain   = 2.0;
			MecanumDrive.PARAMS.lateralGain = 1.2;
			MecanumDrive.PARAMS.headingGain = 2.0;                // a bit stronger

			// Velocity gains off for now
			MecanumDrive.PARAMS.axialVelGain   = 0.0;
			MecanumDrive.PARAMS.lateralVelGain = 0.0;
			MecanumDrive.PARAMS.headingVelGain = 0.1;

			PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

			waitForStart();
			if (isStopRequested()) return;

			Actions.runBlocking(
					drive.actionBuilder(beginPose)
							// Go forward to (24, 0)
							.splineTo(new Vector2d(24, 0), 0.0)
							// Then curve up to (24, 24), facing up
							.splineTo(new Vector2d(24, 24), Math.PI / 2.0)
							// Final small move to (18, 30), still facing up
							.splineTo(new Vector2d(18, 30), Math.PI / 2.0)
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
