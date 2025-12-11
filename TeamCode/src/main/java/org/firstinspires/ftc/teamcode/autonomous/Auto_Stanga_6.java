package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.waypoints;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name = "Autonom_Stanga", group = "Auto")
public class Auto_Stanga_6 extends BaseOpMode {
	private RobotHardware robot;

	@Override
	protected void OnInitialize() {
		robot = new RobotHardware(hardwareMap);
		robot.init();

		// Zero turret encoder at known starting angle
		DcMotorEx turretMotor = robot.turret.getMotor();
		turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

	@Override
	protected void jOnInitialize() {}

	private static final int AUTON_TURRET_TICKS = -57;  // tune this
	private static final double AUTON_SHOOTER_POS = 0.35; // tune this

	@Override
	protected void OnRun() {

		robot.drivetrain.updatePoseEstimate();

		/// === PATH ACTIONS ===

		// START -> SHOOT_BACK
		Action goToShootBack = robot.drivetrain.actionBuilder(waypoints.START)
				.splineTo(
						new Vector2d(waypoints.SHOOT_BACK.position.x, waypoints.SHOOT_BACK.position.y),
						waypoints.SHOOT_BACK.heading.toDouble()
				)
				.build();

		// SHOOT_BACK -> PICKUP1
		Action goToPickup = robot.drivetrain.actionBuilder(waypoints.SHOOT_BACK)
				.splineTo(
						new Vector2d(waypoints.PICKUP1.position.x, waypoints.PICKUP1.position.y),
						waypoints.PICKUP1.heading.toDouble()
				)
				.build();



		// Pickup movement
		Action intakeLine = robot.drivetrain.actionBuilder(waypoints.PICKUP1)
				.lineToY(waypoints.FPICKUP1.position.y)
				.build();

		// returning to shooting pos (first move)
		Action BackIntakeLine = robot.drivetrain.actionBuilder(waypoints.FPICKUP1)
				.lineToY(waypoints.PICKUP1.position.y)
				.build();

		//  returning to shooting pos (second move)
		Action goToShootBack2 = robot.drivetrain.actionBuilder(waypoints.PICKUP1)
				.splineTo(
						new Vector2d(waypoints.SHOOT_BACK.position.x, waypoints.SHOOT_BACK.position.y),
						waypoints.SHOOT_BACK.heading.toDouble()
				)
				.build();

		//ending location
		Action finishLine = robot.drivetrain.actionBuilder((waypoints.SHOOT_BACK)) // :)
				.splineTo(
						new Vector2d(waypoints.FINISH.position.x, waypoints.FINISH.position.y),
						waypoints.FINISH.heading.toDouble()
				)
				.build();

		/// === SHOOTER AND INTAKE ACTIONS ===

		// start shooter and open stopper
		Action shooter_on = packet-> {
			robot.outtake1.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.outtake2.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
			return false;
		};

		//start shooter and close stopper
		Action shooter_off = packet-> {
			robot.outtake1.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.outtake2.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
			return false;
		};

		// feed artifacts to shooter
		Action shootArtifact = packet-> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.transfer.setPower(1);
			return false;
		};

		// stop feeding artifacts to shooter
		Action stopShooting = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.transfer.setPower(0);
			return false;
		};

		// turret preset angle
		Action moveTurretToAutonAngle = packet -> {
			DcMotorEx turretMotor = robot.turret.getMotor();
			turretMotor.setTargetPosition(AUTON_TURRET_TICKS);
			turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			turretMotor.setPower(0.3);
			return false;
		};

		// shooter platform servo angle
		Action setAutonShooterAngle = packet -> {
			robot.turretTumbler.setPosition(AUTON_SHOOTER_POS);
			return false;
		};

		// floor intake (for pickup path)
		Action startIntake = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			return false;
		};

		Action stopIntake = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			return false;
		};


		// === FULL AUTON SEQUENCE ===

		Actions.runBlocking(
				RunSequentially(

						// Drive to shooting spot
						goToShootBack,
						WaitFor(0.2),

						/// START OF FIRST SCORE SEQUENCE


						// Aim turret + tilt shooter (ONE TIME)
						moveTurretToAutonAngle,
						setAutonShooterAngle,
						WaitFor(0.3),

						// Spin shooter wheels + intakeStopper open
						shooter_on,
						WaitFor(1.0),  // let everything get to full power

						// -------------- VOLLEY: 3 OPEN/CLOSE WINDOWS --------------

						// BALL 1
						shootArtifact,
						WaitFor(1.0),   // how long gate is open -> how many artifacts leave
						stopShooting,
						WaitFor(1.0),   // cooldown (wheels recover)

						// BALL 2
						shootArtifact,
						WaitFor(1.0),
						stopShooting,
						moveTurretToAutonAngle,  //set turret and angle positions again in case of errors
						setAutonShooterAngle,
						WaitFor(1.0),

						// BALL 3
						shootArtifact,
						WaitFor(1.2),
						stopShooting,
						WaitFor(0.40),

						// stop shooter system
						stopShooting,
						shooter_off,

						/// END OF FIRST SCORE SEQUENCE

						// Go pick up new artifacts
						goToPickup,
						WaitFor(0.2),

						startIntake,
						WaitFor(0.1),

						intakeLine,
						WaitFor(0.6),

						stopIntake,
						WaitFor(0.6),

						// return to shooting pos

						BackIntakeLine,
						WaitFor(0.1),
						goToShootBack2,
						WaitFor(0.3),

						///  START OF SECOND SCORE SEQUENCE

						// Aim turret + tilt shooter (ONE TIME)
						moveTurretToAutonAngle,
						setAutonShooterAngle,
						WaitFor(0.3),

						// Spin shooter wheels + intakeStopper open
						shooter_on,
						WaitFor(1.0),


						// BALL 1
						shootArtifact,
						WaitFor(1.0),
						stopShooting,
						WaitFor(1.0),

						// BALL 2
						shootArtifact,
						WaitFor(1.0),
						stopShooting,
						moveTurretToAutonAngle,
						setAutonShooterAngle,
						WaitFor(1.0),

						// BALL 3
						shootArtifact,
						WaitFor(1.2),
						stopShooting,
						WaitFor(0.40),

						// stop shooter system
						stopShooting,
						shooter_off,

						///  END OF SECOND SCORE SEQUENCE

						//leave triangle
						finishLine




				)
		);
	}
}
