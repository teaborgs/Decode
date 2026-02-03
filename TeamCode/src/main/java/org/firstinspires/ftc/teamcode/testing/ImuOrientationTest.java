package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "IMU Orientation Test", group = "Testing")
public class ImuOrientationTest extends OpMode {

	RobotHardware robot;

	@Override
	public void init() {
		robot = new RobotHardware(hardwareMap);
		robot.drivetrain.lazyImu.get().resetYaw();
	}

	@Override
	public void loop() {
		YawPitchRollAngles angles =
				robot.drivetrain.lazyImu.get().getRobotYawPitchRollAngles();

		telemetry.addData("Yaw (deg)", angles.getYaw(AngleUnit.DEGREES));
		telemetry.addData("Pitch (deg)", angles.getPitch(AngleUnit.DEGREES));
		telemetry.addData("Roll (deg)", angles.getRoll(AngleUnit.DEGREES));

		telemetry.update();
	}
}
