package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "Pinpoint Rotation Test", group = "Tuning")
public class PinpointRotationTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		Pose2d startPose = new Pose2d(0, 0, 0);
		PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

		// HARD RESET: zero Pinpoint + RR pose
		if (drive.pinpoint != null) {
			drive.pinpoint.resetPosAndIMU();
			// tiny delay to let it calibrate
			Thread.sleep(300);
			drive.pinpoint.setPosition(startPose);
		}
		drive.pose = startPose;

		telemetry.addLine("Ready. Robot will spin in place.");
		telemetry.update();

		waitForStart();
		if (isStopRequested()) return;

		double testDurationSec = 30.0;      // 30s is enough for now
		double omega = Math.PI / 4.0;       // ~45°/s
		long startTime = System.nanoTime();

		while (opModeIsActive()) {
			double elapsedSec = (System.nanoTime() - startTime) / 1e9;
			if (elapsedSec > testDurationSec) break;

			// spin in place
			drive.setDrivePowers(
					new PoseVelocity2d(new Vector2d(0, 0), omega)
			);

			// UPDATE ODOMETRY
			drive.updatePoseEstimate();
			Pose2d p = drive.pose;

			double x = p.position.x;
			double y = p.position.y;
			double hDeg = Math.toDegrees(p.heading.toDouble());

			telemetry.addData("Time (s)", "%.1f", elapsedSec);
			telemetry.addData("ODO X (in)", "%.3f", x);
			telemetry.addData("ODO Y (in)", "%.3f", y);
			telemetry.addData("ODO H (deg)", "%.1f", hDeg);
			telemetry.update();
		}

		// stop
		drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
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
