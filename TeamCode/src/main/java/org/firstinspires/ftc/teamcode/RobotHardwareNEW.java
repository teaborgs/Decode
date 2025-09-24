package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.systems.DistanceSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoMotorSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoServoSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.OpenCloseSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

public final class RobotHardwareNEW {
	public final PinpointDrive drivetrain;
	public TumblerSystem intakeTumbler, scoreTumbler;
	public ExtendoMotorSystem extendo;
	public ExtendoServoSystem scoreExtendo;
	public LiftSystem lift;
	public OpenCloseSystem scoreClaw;
	public DistanceSystem distanceSystem;
	public IntakeSystem intake;
	public ColorSensor intakeColorSensor;
	public Servo axle;

	public RobotHardwareNEW(HardwareMap hardwareMap) {
		intakeColorSensor = hardwareMap.get(ColorSensor.class, "intakeColorSensor");
		drivetrain = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
		intake = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"));
		extendo = new ExtendoMotorSystem(hardwareMap.get(DcMotorEx.class, "extendo"));
		lift = new LiftSystem(hardwareMap.get(DcMotorEx.class, "lift2"));
		scoreClaw = new OpenCloseSystem(hardwareMap.get(Servo.class, "scoreClaw"), 0.31f, 0.001f);
		intakeTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "intakeTumbler"), 0.11f, 0.24f, 0.15f, 0.3f, 0.38f);
		scoreTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "scoreTumbler"), 0.50f, 0.85f, 0.25f, 0.4f, 0.68f);
		scoreExtendo = new ExtendoServoSystem(hardwareMap.get(Servo.class, "scoreExtendo"));
		//
		/*
		intakeTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "intakeTumbler"), 0.14f, 0.14f, 0.14f, 0.14f, 0.23f);
		scoreTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "scoreTumbler"), 0.99f, 0.3f, 0.5f, 0.32f, 0.41f);
		lift = new LiftSystem(hardwareMap.get(DcMotorEx.c	lass, "lift1"), hardwareMap.get(DcMotorEx.class, "lift2"));
		distanceSystem = new DistanceSystem(hardwareMap.get(Rev2mDistanceSensor.class, "leftDist"),
											hardwareMap.get(Rev2mDistanceSensor.class, "rightDist"));
		*/
	}

	public void init() {

		intake.init();
		extendo.init();
		lift.init();
		intakeTumbler.init();
		intakeTumbler.setDestination(TumblerSystem.TumblerDestination.IDLE);
		scoreClaw.init();
		scoreTumbler.init();
		scoreExtendo.init();;


		/*
		intakeTumbler.init();
		scoreTumbler.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
		lift.init();
		scoreTumbler.init();
		scoreClaw.init();
		scoreTumbler.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
		scoreClaw.close();

		*/
	}
}