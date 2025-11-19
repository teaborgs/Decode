package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.DistanceSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

public final class RobotHardware {
	public final MecanumDrive drivetrain;
	public TumblerSystem intakeStopper, turretTumbler;
	public DistanceSystem distanceSystem;
	public IntakeSystem intake, outtake, turret;

	public RobotHardware(HardwareMap hardwareMap) {
		drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

		intake = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"));
		outtake = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "outtake"));
		turret = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "turret"));


		intakeStopper = new TumblerSystem(hardwareMap.get(Servo.class, "intakeStopper"), 0.2f, 0.36f, 0.15f, 0.3f, 0.38f);
		turretTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "turretTumbler"), 0.50f, 0.85f, 0.25f, 0.4f, 0.68f);

	}

	public void init() {
		intakeStopper.init();
		intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
		turretTumbler.init();


		turret.init();
		outtake.init();
		intake.init();

	}
}