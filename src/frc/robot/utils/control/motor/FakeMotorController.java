package frc.robot.utils.control.motor;

import frc.robot.utils.control.MotionConfig;
import frc.robot.utils.control.controltype.ControlType;
import frc.robot.utils.control.encoder.QuadratureEncoder;
import frc.robot.utils.control.encoder.QuadratureEncoder.EncoderType;
import frc.robot.utils.control.motionprofile.motionmagic.MotionMagic;
import frc.robot.utils.control.pidf.PID;
import frc.robot.utils.control.pidf.PIDF;
import frc.robot.utils.math.units.BaseUnit;
import frc.robot.utils.math.units.Quantity;
import frc.robot.utils.math.units.Units;

public class FakeMotorController extends BBMotorController {
	public FakeMotorController() {
	}
	
	private int motionSlot = 0;
	
	private double percentOutput = 0;
	private double position = 8192; // start at 1 revolution
	private double velocity = 0;
	private double voltage = 0;
	
	private final int PID_SLOTS = 2;
	
	private double[] kP = new double[PID_SLOTS];
	private double[] kI = new double[PID_SLOTS];
	private double[] kD = new double[PID_SLOTS];
	private double[] kF = new double[PID_SLOTS];
	private double[] iZone = new double[PID_SLOTS];
	
	private double[] mmAcc = new double[PID_SLOTS];
	private double[] mmVel = new double[PID_SLOTS];
	
	private boolean hasEncoder = false;
	private int encoderTPR = 0;
	
	
	@Override
	protected int getMaxMotionSlots() {
		return PID_SLOTS;
	}

	@Override
	public void selectMotionConfigSlot(int slot) {
		motionSlot = slot;
		System.out.println("Selected slot: " + motionSlot);
		
	}

	@Override
	protected BaseUnit getThetaUnit_nu() {
		return new BaseUnit(Units.REV, encoderTPR, "ticks");
	}

	@Override
	protected BaseUnit getTimeUnit_nu() {
		return Units.MIN;
	}

	@Override
	protected BaseUnit getSecondTimeUnit_nu() {
		return Units.S;
	}

	@Override
	protected void cmdPosition_native(double val_nu, ControlType controlMethod) {
		if (hasEncoder) {
			position = val_nu;
			System.out.println("Setting pos to " + val_nu + " using " + controlMethod);
		}
		
	}

	@Override
	protected void cmdPercent_native(double perc) {
		percentOutput = perc;
		voltage = 12.3 * percentOutput;
		System.out.println("Setting percent output to " + perc);
		
	}

	@Override
	protected void addQuadratureEncoder(QuadratureEncoder sensor) {
		hasEncoder = true;
		encoderTPR = sensor.getTicksPerRev();
	}

	@Override
	public double getPosition_nu() {
		if (!hasEncoder) {
			return 0;
		}
		
		return position;
	}

	@Override
	protected void loadPID(PID constants, int slot) {
		System.out.println("Loading PID into " + slot);
		kP[slot] = constants.getKP();
		kI[slot] = constants.getKI();
		kD[slot] = constants.getKD();
		iZone[slot] = constants.getIZone();
	}
	
	@Override
	protected void loadPIDF(PIDF constants, int slot) {
		// load PID
		loadPID(constants, slot);
		System.out.println("Loading F into " + slot);
		// load F
		kF[slot] = constants.getKF();
	}

	@Override
	protected void loadMotionMagic(double acc, double vel, int slot) {
		System.out.println("Loading MotionMagic into " + slot);
		mmAcc[slot] = acc;
		mmVel[slot] = vel;
	}

	@Override
	public double getVelocity_nu() {
		return velocity;
	}

	@Override
	public double getVoltage() {
		return voltage;
	}

	@Override
	public double getPercentVoltage() {
		return percentOutput;
	}

	@Override
	public void follow(BBMotorController motorController) {
		System.out.println("following...");
		
	}

	@Override
	protected void clearPIDF(int slot) {
		System.out.println("Clearing slot " + slot);
		kP[slot] = 0;
		kI[slot] = 0;
		kD[slot] = 0;
		kF[slot] = 0;
		iZone[slot] = 0;
	}

	@Override
	protected void clearMotionMagic(int slot) {
		mmAcc[slot] = 0;
		mmVel[slot] = 0;
	}
	
	
	
	public static void main(String[] args) {
		FakeMotorController fmc = new FakeMotorController();

		System.out.println(fmc.getPosition());
		fmc.addEncoder(new QuadratureEncoder(EncoderType.AMT));
		System.out.println(fmc.getPosition());
		fmc.setThetaUnit(Units.REV);
		System.out.println(fmc.getPosition());
		fmc.setMeasurementToDistance(new Quantity(3, Units.IN));
		System.out.println(fmc.getPosition());
		
		System.out.println("------------------------------");
		PIDF pidf = new PIDF(1, 2, 3, 4);
		ControlType controlType = ControlType.MotionMagic;
		MotionMagic mm = new MotionMagic(
			new Quantity(1, Units.G_ACC),
			new Quantity(7, Units.FT_PER_S)
		);
		MotionConfig mc = (new MotionConfig()).pid(pidf).controller(controlType).motionMagic(mm);
		fmc.addMotionConfiguration(mc, "MotionMagic");
		fmc.setMotionConfig(0);
		fmc.setMotionConfig("MotionMagic");
		fmc.setMotionConfig("SmartMotion");
		fmc.setMotionConfig(ControlType.MotionMagic);
		
		System.out.println("------------------------------");
		PID pid = new PID(1, 2, 3);
		controlType = ControlType.Position;
		mc = (new MotionConfig()).pid(pid).controller(controlType);
		fmc.addMotionConfiguration(mc, "Position");
		fmc.setMotionConfig(1);
		fmc.setMotionConfig("Position");
		fmc.setMotionConfig(ControlType.Position);
		
		System.out.println("------------------------------");
		pidf = new PIDF(1, 2, 3, 4);
		controlType = ControlType.Velocity;
		mc = (new MotionConfig()).pid(pidf).controller(controlType);
		fmc.addMotionConfiguration(mc, "Velocity");
		// shouldn't output anything, hasn't been loaded in, aka internally stored not in wrapped motor controller
		fmc.setMotionConfig(2);
		// at this point, replaced 1 in motor controller
		fmc.setMotionConfig(1);
		// load 2 back in
		fmc.setMotionConfig("Velocity");
		// replaced 0
		fmc.setMotionConfig("MotionMagic");
		// replaced 1
		fmc.setMotionConfig(ControlType.Velocity);
	}
}
