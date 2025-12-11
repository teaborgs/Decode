package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "RR Turn 90 Test", group = "Tuning")
public class RRTurn90Test extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		Pose2d startPose = new Pose2d(0, 0, 0);
		PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

		// make sure RR + Pinpoint start aligned
		drive.pose = startPose;
		if (drive.pinpoint != null) {
			drive.pinpoint.resetPosAndIMU();
			Thread.sleep(300);
			drive.pinpoint.setPosition(startPose);
		}

		telemetry.addLine("Ready to turn 90°");
		telemetry.update();

		waitForStart();
		if (isStopRequested()) return;

		// ask RR to turn +90° (counterclockwise)
		Actions.runBlocking(
				drive.actionBuilder(drive.pose)
						.turn(Math.PI / 2)   // 90 degrees
						.build()
		);

		// update final pose from Pinpoint
		drive.updatePoseEstimate();
		Pose2d endPose = drive.pose;

		double finalHeadingDeg = Math.toDegrees(endPose.heading.toDouble());

		telemetry.addLine("=== Turn Test Done ===");
		telemetry.addData("Final heading (deg)", "%.1f", finalHeadingDeg);
		telemetry.update();

		sleep(5000);
	}
}
