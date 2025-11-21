package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.CRServo;

public class TransferSystem extends AbstractSystem
{
	private final CRServo transfer;

	public TransferSystem(CRServo transfer)
	{
		this.transfer = transfer;
	}

	@Override
	public void init()
	{

		transfer.setPower(0);
	}

	public void setPower(double power)
	{
		transfer.setPower(power);
	}


	public void start()
	{
		transfer.setPower(-1.0);
	}


	public void stop()
	{
		transfer.setPower(0.0);
	}


		public void update()
	{

	}
}
