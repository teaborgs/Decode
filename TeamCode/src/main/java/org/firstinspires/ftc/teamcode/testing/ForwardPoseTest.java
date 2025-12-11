package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "Forward Pose Test", group = "Testing")
public class ForwardPoseTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

		waitForStart();
		if (isStopRequested()) return;

		// drive forward slowly for ~1.5s
		long start = System.nanoTime();
		while (opModeIsActive() && (System.nanoTime() - start)/1e9 < 1.5) {
			// pure forward command
			drive.setDrivePowers(
					new PoseVelocity2d(new Vector2d(0.4, 0.0), 0.0)
			);

			drive.updatePoseEstimate();
			Pose2d p = drive.pose;

			telemetry.addData("X (in)", "%.2f", p.position.x);
			telemetry.addData("Y (in)", "%.2f", p.position.y);
			telemetry.addData("H (deg)", "%.1f", Math.toDegrees(p.heading.toDouble()));
			telemetry.update();
		}

		drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
	}
}
