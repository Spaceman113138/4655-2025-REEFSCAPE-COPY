package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.ctre.phoenix6.sim.CANrangeSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class OutakeRollersIOSim extends OutakeRollersIOTallonFX {
    private static final double kGearRatio = 10.0;

    private TalonFXSimState rightMotorSim;
    private final DCMotorSim rightSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, kGearRatio), DCMotor.getKrakenX60(1));

    private TalonFXSimState leftMotorSim;
    private final DCMotorSim leftSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, kGearRatio), DCMotor.getKrakenX60(1));

    private CANrangeSimState manipulatorSimState;
    private CANrangeSimState elevatorSimState;

    public OutakeRollersIOSim() {
        super();
        rightMotorSim = rightMotor.getSimState();
        leftMotorSim = leftMotor.getSimState();
        manipulatorSimState = manipulatorLaser.getSimState();
        elevatorSimState = elevatorLaser.getSimState();
        manipulatorLaser.getConfigurator().apply(createCANrangeConfig());
        elevatorLaser.getConfigurator().apply(createCANrangeConfig());
    }

    private CANrangeConfiguration createCANrangeConfig() {
        CANrangeConfiguration config = new CANrangeConfiguration();
        config.ToFParams.withUpdateMode(UpdateModeValue.ShortRange100Hz);
        config.ProximityParams.withMinSignalStrengthForValidMeasurement(2500)
                .withProximityHysteresis(0.025)
                .withProximityThreshold(0.1);
        return config;
    }

    public void updateInputs(OutakeRollersIOInputs inputs) {
        rightMotorSim.setSupplyVoltage(12);
        leftMotorSim.setSupplyVoltage(12);

        double rightMotorVoltage = rightMotorSim.getMotorVoltage();
        double leftMotorVoltage = leftMotorSim.getMotorVoltage();

        rightSimModel.setInputVoltage(rightMotorVoltage);
        rightSimModel.update(0.02);
        leftSimModel.setInputVoltage(leftMotorVoltage);
        leftSimModel.update(0.02);

        rightMotorSim.setRawRotorPosition(rightSimModel.getAngularPosition().times(kGearRatio));
        rightMotorSim.setRotorVelocity(rightSimModel.getAngularVelocity().times(kGearRatio));
        leftMotorSim.setRawRotorPosition(leftSimModel.getAngularPosition().times(kGearRatio));
        leftMotorSim.setRotorVelocity(leftSimModel.getAngularVelocity().times(kGearRatio));

        super.updateInputs(inputs);
    }
}
