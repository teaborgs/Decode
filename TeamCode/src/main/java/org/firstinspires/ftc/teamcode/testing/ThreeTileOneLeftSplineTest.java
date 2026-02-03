package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "Three Tile + One Left Spline", group = "Testing")
public class ThreeTileOneLeftSplineTest extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {
		Pose2d startPose = new Pose2d(0, 0, 0);

		// Use your tuned params (these should also be in MecanumDrive.PARAMS normally)
		MecanumDrive.PARAMS.maxWheelVel     = 30.0;          // in/s
		MecanumDrive.PARAMS.maxProfileAccel = 25.0;          // in/s^2
		MecanumDrive.PARAMS.maxAngVel       = Math.PI / 3.0; // rad/s
		MecanumDrive.PARAMS.maxAngAccel     = Math.PI / 3.0;

		MecanumDrive.PARAMS.axialGain       = 3.0;
		MecanumDrive.PARAMS.lateralGain     = 2.0;
		MecanumDrive.PARAMS.headingGain     = 2.0;

		MecanumDrive.PARAMS.axialVelGain    = 0.0;
		MecanumDrive.PARAMS.lateralVelGain  = 0.0;
		MecanumDrive.PARAMS.headingVelGain  = 0.0;

		PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

		waitForStart();
		if (isStopRequested()) return;

		final double TILE = 24.0; // approx field tile size (adjust if needed)

		Actions.runBlocking(
				drive.actionBuilder(startPose)
						// 3 tiles straight forward, keep heading 0
						.lineToX(3 * TILE)

						// then SPLINE into 1 tile left and rotate 90° left (to +π/2)
						// end pose ≈ (72, 24, 90°)
						.splineTo(
								new Vector2d(3 * TILE, 1 * TILE),
								Math.PI / 2.0
						)
						.build()
		);
	}
}
