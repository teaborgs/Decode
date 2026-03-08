package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.systems.OuttakeSystem;

@Config
@TeleOp(name = "Shooter PIDFF Tuning", group = "Tuning")
public class ShooterPidffTuningTeleOp extends OpMode {

	public static String MOTOR_1_NAME = "outtake1";
	public static String MOTOR_2_NAME = "outtake2";

	public static DcMotorSimple.Direction MOTOR_1_DIR = DcMotorSimple.Direction.FORWARD;
	public static DcMotorSimple.Direction MOTOR_2_DIR = DcMotorSimple.Direction.REVERSE;

	public static boolean ENABLE_SHOOTER = false;
	public static double TARGET_RPM = 3400.0;

	public static double TICKS_PER_REV = 28.0;

	public static double kV = 1.0 / 6000.0;
	public static double kS = 0.008;

	public static double kP = 0.00045;
	public static double kI = 0.0;
	public static double kD = 0.0;

	public static double RPM_SLEW = 10000.0;

	public static double MAX_POWER = 1.0;
	public static double MIN_POWER = 0.0;

	public static double KICK_PWR = 0.28;
	public static double KICK_TIME_SEC = 0.28;
	public static double KICK_ERR_RPM = 90.0;
	public static double KICK_DROP_RPM_PER_S = 1400.0;
	public static double KICK_MIN_CMD_RPM = 1800.0;

	public static double OVERSHOOT_START_RPM = 45.0;
	public static double OVERSHOOT_FULL_RPM = 140.0;
	public static double OVERSHOOT_MIN_FF_SCALE = 0.72;

	public static double INTEGRAL_MAX = 8000.0;

	public static double FILTER_OLD_WEIGHT = 0.72;
	public static double FILTER_NEW_WEIGHT = 0.28;

	private OuttakeSystem outtake1;
	private OuttakeSystem outtake2;
	private VoltageSensor voltageSensor;

	@Override
	public void init() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, MOTOR_1_NAME);
		DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, MOTOR_2_NAME);

		outtake1 = new OuttakeSystem(motor1, MOTOR_1_DIR);
		outtake2 = new OuttakeSystem(motor2, MOTOR_2_DIR);

		voltageSensor = hardwareMap.voltageSensor.iterator().next();

		outtake1.setVoltageSensor(voltageSensor);
		outtake2.setVoltageSensor(voltageSensor);

		pushConstants();

		outtake1.init();
		outtake2.init();

		telemetry.addLine("Shooter tuning ready");
		telemetry.addData("Motor 1", MOTOR_1_NAME);
		telemetry.addData("Motor 2", MOTOR_2_NAME);
		telemetry.update();
	}

	@Override
	public void loop() {
		pushConstants();

		double target = ENABLE_SHOOTER ? TARGET_RPM : 0.0;

		outtake1.setRpm(target);
		outtake2.setRpm(target);

		double rpm1 = outtake1.getRpm();
		double rpm2 = outtake2.getRpm();
		double avgAbs = (Math.abs(rpm1) + Math.abs(rpm2)) * 0.5;
		double avgSigned = (rpm1 + rpm2) * 0.5;
		double errAbs = target - avgAbs;

		if (gamepad1.a) {
			outtake1.triggerKick();
			outtake2.triggerKick();
		}

		if (gamepad1.dpad_up) {
			TARGET_RPM += 10.0;
		} else if (gamepad1.dpad_down) {
			TARGET_RPM -= 10.0;
		}

		if (TARGET_RPM < 0) TARGET_RPM = 0;

		telemetry.addData("Enabled", ENABLE_SHOOTER);
		telemetry.addData("Target RPM", TARGET_RPM);
		telemetry.addData("Voltage", voltageSensor.getVoltage());

		telemetry.addLine();

		telemetry.addData("OT1 RPM", rpm1);
		telemetry.addData("OT2 RPM", rpm2);
		telemetry.addData("AVG RPM ABS", avgAbs);
		telemetry.addData("AVG RPM SIGNED", avgSigned);
		telemetry.addData("AVG ERR ABS", errAbs);

		telemetry.addLine();

		telemetry.addData("OT1 Power", outtake1.raw().getPower());
		telemetry.addData("OT2 Power", outtake2.raw().getPower());

		telemetry.addLine();

		telemetry.addData("kV", kV);
		telemetry.addData("kS", kS);
		telemetry.addData("kP", kP);
		telemetry.addData("kI", kI);
		telemetry.addData("kD", kD);
		telemetry.addData("RPM_SLEW", RPM_SLEW);

		telemetry.addLine();

		telemetry.addData("OVERSHOOT_START_RPM", OVERSHOOT_START_RPM);
		telemetry.addData("OVERSHOOT_FULL_RPM", OVERSHOOT_FULL_RPM);
		telemetry.addData("OVERSHOOT_MIN_FF_SCALE", OVERSHOOT_MIN_FF_SCALE);

		telemetry.addLine();

		telemetry.addData("KICK_PWR", KICK_PWR);
		telemetry.addData("KICK_TIME_SEC", KICK_TIME_SEC);
		telemetry.addData("KICK_ERR_RPM", KICK_ERR_RPM);
		telemetry.addData("KICK_DROP_RPM_PER_S", KICK_DROP_RPM_PER_S);
		telemetry.addData("KICK_MIN_CMD_RPM", KICK_MIN_CMD_RPM);

		telemetry.addLine();

		telemetry.addData("FILTER_OLD_WEIGHT", FILTER_OLD_WEIGHT);
		telemetry.addData("FILTER_NEW_WEIGHT", FILTER_NEW_WEIGHT);

		telemetry.update();
	}

	@Override
	public void stop() {
		if (outtake1 != null) outtake1.stop();
		if (outtake2 != null) outtake2.stop();
	}

	private void pushConstants() {
		double sum = FILTER_OLD_WEIGHT + FILTER_NEW_WEIGHT;
		if (sum <= 0) {
			FILTER_OLD_WEIGHT = 0.72;
			FILTER_NEW_WEIGHT = 0.28;
			sum = 1.0;
		}

		FILTER_OLD_WEIGHT /= sum;
		FILTER_NEW_WEIGHT /= sum;

		OuttakeSystem.TICKS_PER_REV = TICKS_PER_REV;

		OuttakeSystem.kV = kV;
		OuttakeSystem.kS = kS;

		OuttakeSystem.kP = kP;
		OuttakeSystem.kI = kI;
		OuttakeSystem.kD = kD;

		OuttakeSystem.RPM_SLEW = RPM_SLEW;

		OuttakeSystem.MAX_POWER = MAX_POWER;
		OuttakeSystem.MIN_POWER = MIN_POWER;

		OuttakeSystem.KICK_PWR = KICK_PWR;
		OuttakeSystem.KICK_TIME_SEC = KICK_TIME_SEC;
		OuttakeSystem.KICK_ERR_RPM = KICK_ERR_RPM;
		OuttakeSystem.KICK_DROP_RPM_PER_S = KICK_DROP_RPM_PER_S;
		OuttakeSystem.KICK_MIN_CMD_RPM = KICK_MIN_CMD_RPM;

		OuttakeSystem.OVERSHOOT_START_RPM = OVERSHOOT_START_RPM;
		OuttakeSystem.OVERSHOOT_FULL_RPM = OVERSHOOT_FULL_RPM;
		OuttakeSystem.OVERSHOOT_MIN_FF_SCALE = OVERSHOOT_MIN_FF_SCALE;

		OuttakeSystem.INTEGRAL_MAX = INTEGRAL_MAX;
		OuttakeSystem.FILTER_OLD_WEIGHT = FILTER_OLD_WEIGHT;
		OuttakeSystem.FILTER_NEW_WEIGHT = FILTER_NEW_WEIGHT;
	}
}