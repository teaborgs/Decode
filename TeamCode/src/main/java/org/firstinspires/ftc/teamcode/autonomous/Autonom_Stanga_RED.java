package org.firstinspires.ftc.teamcode.autonomous;
/*
import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.RobotHardwareNEW;
import org.firstinspires.ftc.teamcode.systems.ExtendoMotorSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoServoSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem.IntakeDirection;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name = "\uD83D\uDD34Autonom_Basket\uD83D\uDD34", group = "Auto")
public class Autonom_Stanga_RED extends BaseOpMode
{
	private RobotHardwareNEW robot;
	ElapsedTime timercaca= new ElapsedTime();
	@Override
	protected void OnInitialize()
	{
		robot = new RobotHardwareNEW(hardwareMap);
		robot.init();
		robot.lift.setLiftPower(1);
	}

	@Override
	protected void jOnInitialize()
	{
	}

	@Override
	protected void OnRun()
	{
		// Deliver preload specimen
		Actions.runBlocking(RunSequentially(
				robot.scoreClaw.closeAction(),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(9, 16, Math.toRadians(-16)), Math.toRadians(-16))
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.BASKET_HIGH),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER),
						robot.intakeTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
						robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.HALF)
				),
				robot.scoreClaw.openAction(),
				WaitFor(0.6),
				robot.intake.setDirectionAction(IntakeDirection.FORWARD),
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.EXTENDED),
				robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.RETRACTED),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE)
		));
		//sample #1
		timercaca.reset();
		while(!seesYellow() && opModeIsActive() && timercaca.milliseconds()<2000) {
		}
		robot.intake.setIntakeDirection(IntakeDirection.STOP);
		Actions.runBlocking(RunSequentially(
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.RETRACTED),
				robot.intakeTumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
				WaitFor(0.9),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.EXTENDED),
				WaitFor(0.2),
				robot.scoreClaw.closeAction(),
				WaitFor(0.5),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.RETRACTED),
				RunInParallel(
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.BASKET_HIGH),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER)
				),
				robot.scoreClaw.openAction(),
				WaitFor(0.6),
				robot.drivetrain.actionBuilder(robot.drivetrain.pose)
						.setTangent(0)
						.splineToLinearHeading(new Pose2d(9, 16, Math.toRadians(0)), Math.toRadians(0))
						.build(),
				robot.intakeTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
				robot.intake.setDirectionAction(IntakeDirection.FORWARD),
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.AUTO),
				WaitFor(0.2),
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.EXTENDED),
				robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.RETRACTED),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE)

		));
		//sample #2
		timercaca.reset();
		while(!seesYellow() && opModeIsActive() && timercaca.milliseconds()<2000) {
		}
		robot.intake.setIntakeDirection(IntakeDirection.STOP);
		Actions.runBlocking(RunSequentially(
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.RETRACTED),
				robot.intakeTumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
				WaitFor(0.7),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.EXTENDED),
				WaitFor(0.2),
				robot.scoreClaw.closeAction(),
				WaitFor(0.5),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.RETRACTED),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(9, 18, Math.toRadians(-16)), Math.toRadians(-16))
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.BASKET_HIGH),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER)
				),
				robot.scoreClaw.openAction(),
				WaitFor(0.6),
				robot.drivetrain.actionBuilder(robot.drivetrain.pose)
						.setTangent(0)
						.splineToLinearHeading(new Pose2d(9, 16, Math.toRadians(28)), Math.toRadians(28))
						.build(),
				robot.intakeTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY),
				robot.intake.setDirectionAction(IntakeDirection.FORWARD),
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.AUTO),
				WaitFor(0.2),
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.EXTENDED),
				robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.RETRACTED),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE)
		));
		//sample #3
		timercaca.reset();
		while(!seesYellow() && opModeIsActive() && timercaca.milliseconds()<2000) {
		}
		robot.intake.setIntakeDirection(IntakeDirection.STOP);
		Actions.runBlocking(RunSequentially(
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.RETRACTED),
				robot.intakeTumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
				WaitFor(0.9),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.EXTENDED),
				WaitFor(0.2),
				robot.scoreClaw.closeAction(),
				WaitFor(0.5),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.RETRACTED),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(9, 19, Math.toRadians(-16)), Math.toRadians(-16))
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.BASKET_HIGH),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER)
				),
				robot.scoreClaw.openAction(),
				WaitFor(0.6)
				));
		Actions.runBlocking(RunSequentially(
				robot.scoreClaw.closeAction(),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(55, -30, Math.toRadians(-90)), Math.toRadians(-90))
								.build(),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
						robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.RETRACTED),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE)
				),
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.SUBMERSIBIL),
				WaitFor(0.4),
				robot.intake.setDirectionAction(IntakeDirection.FORWARD),
				robot.intakeTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY)
		));
		//submersible
		timercaca.reset();
		robot.intake.setIntakeDirection(IntakeDirection.STOP);
		robot.extendo.setSlowMode(true);
		while (opModeIsActive() && !seesTargetSample() && timercaca.milliseconds()<1500) {
			if (seeNoGood()) {
				Actions.runBlocking(RunSequentially(
						robot.intake.setDirectionAction(IntakeDirection.REVERSE),
						WaitFor(0.4),
						robot.intake.setDirectionAction(IntakeDirection.FORWARD)
				));
			} else
			{
				Actions.runBlocking(RunSequentially(
						robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.EXTENDED),
						WaitFor(0.3),
						new InstantAction(() -> robot.extendo.setSlowMode(false)),
						robot.intake.setDirectionAction(IntakeDirection.FORWARD)
				));
			}
		}
		robot.extendo.setSlowMode(false);
		robot.intake.setIntakeDirection(IntakeDirection.STOP);
		Actions.runBlocking(RunSequentially(
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.RETRACTED),
				robot.intakeTumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
				WaitFor(0.5),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.EXTENDED),
				WaitFor(0.2),
				robot.scoreClaw.closeAction(),
				WaitFor(0.5),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.RETRACTED),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(9, 19, Math.toRadians(-16)), Math.toRadians(-16))
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.BASKET_HIGH),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER)
				),
				robot.scoreClaw.openAction(),
				WaitFor(0.6)
		));
//PICKUP 2 SUBMERSIBLE
		Actions.runBlocking(RunSequentially(
				robot.scoreClaw.closeAction(),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(55, -30, Math.toRadians(-90)), Math.toRadians(-90))
								.build(),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
						robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.RETRACTED),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.IDLE)
				),
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.SUBMERSIBIL),
				WaitFor(0.4),
				robot.intake.setDirectionAction(IntakeDirection.FORWARD),
				robot.intakeTumbler.setDestinationAction(TumblerSystem.TumblerDestination.BUSY)
		));
		timercaca.reset();
		robot.intake.setIntakeDirection(IntakeDirection.STOP);
		robot.extendo.setSlowMode(true);
		while (opModeIsActive() && !seesTargetSample() && timercaca.milliseconds()<1500) {
			if (seeNoGood()) {
				Actions.runBlocking(RunSequentially(
						robot.intake.setDirectionAction(IntakeDirection.REVERSE),
						WaitFor(0.4),
						robot.intake.setDirectionAction(IntakeDirection.FORWARD)
				));
			} else
			{
				Actions.runBlocking(RunSequentially(
						robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.EXTENDED),
						WaitFor(0.3),
						new InstantAction(() -> robot.extendo.setSlowMode(false)),
						robot.intake.setDirectionAction(IntakeDirection.FORWARD)
				));
			}
		}
		robot.extendo.setSlowMode(false);
		robot.intake.setIntakeDirection(IntakeDirection.STOP);
		Actions.runBlocking(RunSequentially(
				robot.extendo.extendoAction(ExtendoMotorSystem.ExtendoLevel.RETRACTED),
				robot.intakeTumbler.setDestinationAction(TumblerSystem.TumblerDestination.IDLE),
				WaitFor(0.5),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.EXTENDED),
				WaitFor(0.2),
				robot.scoreClaw.closeAction(),
				WaitFor(0.5),
				robot.scoreExtendo.extendAction(ExtendoServoSystem.ExtendoLevel.RETRACTED),
				RunInParallel(
						robot.drivetrain.actionBuilder(robot.drivetrain.pose)
								.setTangent(0)
								.splineToLinearHeading(new Pose2d(9, 19, Math.toRadians(-16)), Math.toRadians(-16))
								.build(),
						robot.lift.moveLiftTo(LiftSystem.LiftLevel.BASKET_HIGH),
						robot.scoreTumbler.setDestinationAction(TumblerSystem.TumblerDestination.HOVER)
				),
				robot.scoreClaw.openAction(),
				WaitFor(0.8)
		));
	}
	private boolean seesYellow() {
		int red = robot.intakeColorSensor.red();
		int green = robot.intakeColorSensor.green();
		int blue = robot.intakeColorSensor.blue();

		return red > 6000 && green > 8000 && blue < 4000;
	}
	private boolean seesTargetSample() {
		int red = robot.intakeColorSensor.red();
		int green = robot.intakeColorSensor.green();
		int blue = robot.intakeColorSensor.blue();

		boolean isYellow = red > 6000 && green > 8000 && blue < 4000;
		boolean isRed = red > 3000 && red < 6000 && green < 5000 && blue < 3000;
		//red < 3000 && green < 3000 && blue > 4000;

		return isYellow || isRed;
	}
	private boolean seeNoGood() {
		int red = robot.intakeColorSensor.red();
		int green = robot.intakeColorSensor.green();
		int blue = robot.intakeColorSensor.blue();

		return red < 3000 && green < 3000 && blue > 4000;
		//red > 3000 && red < 6000 && green < 5000 && blue < 3000;
	}
}
*/