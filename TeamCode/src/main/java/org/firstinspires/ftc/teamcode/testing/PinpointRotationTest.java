package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "Pinpoint Rotation Test (Controlled)", group = "Tuning")
public class PinpointRotationTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		Pose2d startPose = new Pose2d(0, 0, 0);
		PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

		// HARD RESET: zero Pinpoint pose
		if (drive.pinpoint != null) {
			drive.pinpoint.resetPosAndIMU();
			sleep(300);
			drive.pinpoint.setPosition(startPose);
		}

		// IMPORTANT: zero the HUB IMU yaw too, since we use it for heading
		drive.lazyImu.get().resetYaw();

		drive.pose = startPose;

		telemetry.addLine("Ready. Robot will spin slowly in place.");
		telemetry.update();

		waitForStart();
		if (isStopRequested()) return;

		double testDurationSec = 4.0;   // how long to spin
		double rotationPower   = 0.2;   // spin power

		long startTime = System.nanoTime();

		while (opModeIsActive()) {
			double elapsedSec = (System.nanoTime() - startTime) / 1e9;
			if (elapsedSec > testDurationSec) break;

			// Spin in place at low power
			drive.leftFront.setPower( rotationPower);
			drive.leftBack.setPower(  rotationPower);
			drive.rightFront.setPower(-rotationPower);
			drive.rightBack.setPower(-rotationPower);

			// UPDATE ODOMETRY FROM PINPOINT + IMU
			PoseVelocity2d vel = drive.updatePoseEstimate();
			Pose2d p = drive.pose;

			double x = p.position.x;
			double y = p.position.y;
			double hDeg = Math.toDegrees(p.heading.toDouble());

			telemetry.addData("Time (s)", "%.1f", elapsedSec);
			telemetry.addData("ODO X (in)", "%.3f", x);
			telemetry.addData("ODO Y (in)", "%.3f", y);
			telemetry.addData("ODO H (deg)", "%.1f", hDeg);
			telemetry.addData("Rot Power", rotationPower);
			telemetry.update();
		}

		// stop
		drive.leftFront.setPower(0);
		drive.leftBack.setPower(0);
		drive.rightFront.setPower(0);
		drive.rightBack.setPower(0);

		drive.updatePoseEstimate();
		Pose2d endPose = drive.pose;

		telemetry.addLine("=== Rotation Test Done ===");
		telemetry.addData("Final ODO X (in)", "%.3f", endPose.position.x);
		telemetry.addData("Final ODO Y (in)", "%.3f", endPose.position.y);
		telemetry.addData("Final ODO H (deg)", "%.1f", Math.toDegrees(endPose.heading.toDouble()));
		telemetry.update();

		sleep(5000);
	}
}
