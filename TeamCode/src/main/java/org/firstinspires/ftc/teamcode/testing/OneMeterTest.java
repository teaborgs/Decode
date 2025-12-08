package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "One Meter Test", group = "Testing")
public final class OneMeterTest extends BaseOpMode
{
	private PinpointDrive drivetrain;

	@Override
	protected void OnInitialize()
	{
		drivetrain = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
	}

	@Override
	protected void jOnInitialize()
	{

	}

	@Override
	protected void OnRun()
	{
		Actions.runBlocking(drivetrain.actionBuilder(new Pose2d(0, 0, 0))
				.lineToXConstantHeading(centimetersToInches(100))
				.build());
	}
}