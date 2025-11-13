package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.RobotHardwareDECODE;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem.IntakeDirection;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@TeleOp(name = "decodeOP", group = "TeleOp")
public final class decode extends BaseOpMode
{
	private InputSystem driveInput, armInput;

	private RobotHardwareDECODE robot;

	private static class Keybindings
	{
		//ramane de vazut cum facem cu driver2 :)
		/*public static class Arm
		{
			public static final InputSystem.Key BASKET_LOW_KEY = new InputSystem.Key("dpad_down");
			public static final InputSystem.Key BASKET_HIGH_KEY = new InputSystem.Key("dpad_up");
			public static final InputSystem.Key CHAMBER_HIGH_KEY = new InputSystem.Key("dpad_left");
			public static final InputSystem.Key PRIMARY_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key SPECIMEN_KEY = new InputSystem.Key("b");
			public static final InputSystem.Key SUSPEND_KEY = new InputSystem.Key("y");
			public static final InputSystem.Key CANCEL_SUSPEND_KEY = new InputSystem.Key("x");
		}*/

		public static class Drive
		{
			public static final InputSystem.Key SHOOTER_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key INTAKE_KEY = new InputSystem.Key("right_bumper");
			public static final InputSystem.Axis TURRET_ANGLE = new InputSystem.Axis("right_stick_x");
			public static final InputSystem.Axis SHOOTER_ANGLE = new InputSystem.Axis("right_stick_y");

			public static final InputSystem.Key SUPPRESS_KEY = new InputSystem.Key("left_bumper");
			public static final InputSystem.Axis DRIVE_X = new InputSystem.Axis("left_stick_x");
			public static final InputSystem.Axis DRIVE_Y = new InputSystem.Axis("left_stick_y");
			public static final InputSystem.Axis DRIVE_ROT_L = new InputSystem.Axis("left_trigger");
			public static final InputSystem.Axis DRIVE_ROT_R = new InputSystem.Axis("right_trigger");
			public static final InputSystem.Key EXTENDO_FULL_KEY = new InputSystem.Key("dpad_up");
			public static final InputSystem.Key EXTENDO_HALF_KEY = new InputSystem.Key("dpad_left");
			public static final InputSystem.Key EXTENDO_ZERO_KEY = new InputSystem.Key("dpad_down");
			public static final InputSystem.Key GRAB_TRANSFER_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key GRAB_HOLD_KEY = new InputSystem.Key("b");
			public static final InputSystem.Key GRAB_WALL_KEY = new InputSystem.Key("x");
			public static final InputSystem.Key GRAB_RESET_KEY = new InputSystem.Key("y");
		}
	}

	@Override

	protected void OnInitialize()
	{
		driveInput = input1;
		armInput = input2;
	}

	@Override
	protected void jOnInitialize() {
		driveInput = input1;
		armInput = input2;
	}

	@Override
	protected void OnStart()
	{
		robot = new RobotHardwareDECODE(hardwareMap);
		robot.init();
		shooterPosition = robot.turretTumbler.getPosition();
	}

	@Override
	protected void OnRun()
	{
		Drive();
		Shooter();
		Turret();
		ShooterAngle();
		Intake();

	}

	private void Drive()
	{
		float speed = 1f;
        if (driveInput.isPressed(Keybindings.Drive.SUPPRESS_KEY)) speed = 0.4f;
		robot.drivetrain.setDrivePowers(
				new PoseVelocity2d(new Vector2d(driveInput.getValue(Keybindings.Drive.DRIVE_Y),
						driveInput.getValue(Keybindings.Drive.DRIVE_X)).times(-speed),
						(driveInput.getValue(Keybindings.Drive.DRIVE_ROT_L) - driveInput.getValue(Keybindings.Drive.DRIVE_ROT_R)) * speed
				)
		);
	}

	private static final double TURRET_DEADZONE = 0.12;
	private static final double SHOOTER_DEADZONE = 0.15;
	private double shooterPosition = 0.5; // trb tunat

	private void Shooter() {
		if (driveInput.isPressed(Keybindings.Drive.SHOOTER_KEY)) {
			robot.outtake.setIntakeDirection(IntakeDirection.FORWARD);
		} else {
			robot.outtake.setIntakeDirection(IntakeDirection.STOP);
		}
	}

	private void ShooterAngle() {
		double input = -driveInput.getValue(Keybindings.Drive.SHOOTER_ANGLE); // invert?

		if (Math.abs(input) < SHOOTER_DEADZONE) return;

		shooterPosition += input * 0.02;              // sensitivity
		shooterPosition = Math.max(0, Math.min(1, shooterPosition));

		robot.turretTumbler.setPosition(shooterPosition);

		telemetry.addData("Shooter Angle Pos", shooterPosition); //sterge dupa teste
		telemetry.update();
	}

	private void Turret(){
		double x = driveInput.getValue(Keybindings.Drive.TURRET_ANGLE);

		if (Math.abs(x) < TURRET_DEADZONE) {
			robot.turret.setIntakeDirection(IntakeDirection.STOP);
			return;
		}
		if (x > 0) {
			robot.turret.setIntakeDirection(IntakeDirection.FORWARD);
		} else {
			robot.turret.setIntakeDirection(IntakeDirection.REVERSE);
		}
	}

	private void Intake() {
		if (driveInput.isPressed(Keybindings.Drive.INTAKE_KEY)) {
			robot.intake.setIntakeDirection(IntakeDirection.FORWARD);
			robot.intakeTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY); //trb facute pozitiile
		} else {
			robot.intake.setIntakeDirection(IntakeDirection.STOP);
			robot.intakeTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE); //cred ca il pastrez?
		}
	}
}