package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.DistanceSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoMotorSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoServoSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.OpenCloseSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

public final class RobotHardwareDECODE {
	public final PinpointDrive drivetrain;
	public TumblerSystem intakeTumbler, turretTumbler;
	public DistanceSystem distanceSystem;
	public IntakeSystem intake, outtake, turret;
	public ColorSensor intakeColorSensor;


	public RobotHardwareDECODE(HardwareMap hardwareMap) {
		intakeColorSensor = hardwareMap.get(ColorSensor.class, "intakeColorSensor");
		drivetrain = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

		intake = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"));
		outtake = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "outtake"));
		turret = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "turret"));


		intakeTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "intakeTumbler"), 0.11f, 0.24f, 0.15f, 0.3f, 0.38f);
		turretTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "scoreTumbler"), 0.50f, 0.85f, 0.25f, 0.4f, 0.68f);

	}

	public void init() {
		intakeTumbler.init();
		intakeTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
		turretTumbler.init();


		turret.init();
		outtake.init();
		intake.init();

	}
}