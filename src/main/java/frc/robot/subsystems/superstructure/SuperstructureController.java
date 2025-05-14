package frc.robot.subsystems.superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.util.FieldConstants.PieceType.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.FieldConstants.PieceType;
import frc.robot.util.SubsystemGroup;
import java.util.function.BooleanSupplier;

public class SuperstructureController extends SubsystemGroup {
    public Elevator elevator;
    public Wrist wrist;
    public OutakeRollers outake;

    public static enum ScorePositions {
        hold(0.25, -45, 6.0),
        level1(0.05, -45, 6.0),
        level2(0.235, 30.0, 6.0),
        level3(0.370, 30.0, 6.0),
        level4(0.58, 35.0, 6.0),
        barge(0.588, -35, 12.0),
        processor(0.05, 0.0, 6.0);

        public final double elevator;
        public final double wrist;
        public final double outake;

        private ScorePositions(double elevator, double wrist, double outake) {
            this.elevator = elevator;
            this.wrist = wrist;
            this.outake = outake;
        }
    }

    public static enum IntakePositions {
        hopperCoral(0, -2.0, coral),
        algaeGround(0, 20.0, algae),
        algaeL2(0.185, 0, algae),
        algaeL3(.312, 0, algae);

        public final double elevator;
        public final double wrist;
        public final PieceType type;

        private IntakePositions(double elevator, double wrist, PieceType type) {
            this.elevator = elevator;
            this.wrist = wrist;
            this.type = type;
        }
    }

    public static enum StorePositions {
        storeCoral(0, 0),
        storeAlgae(.05, 0);

        public final double elevator;
        public final double wrist;

        private StorePositions(double elevator, double wrist) {
            this.elevator = elevator;
            this.wrist = wrist;
        }
    }

    public SuperstructureController(ElevatorIO elevator, WristIO wrist, OutakeRollersIO outake) {
        this.elevator = new Elevator(elevator);
        this.wrist = new Wrist(wrist);
        this.outake = new OutakeRollers(outake);

        configureDefaultCommands();
    }

    private void configureDefaultCommands() {
        elevator.setDefaultCommand(elevator.moveToStore(outake.hasAlgae));
        wrist.setDefaultCommand(wrist.moveToSetpoint(0.0));
        outake.setDefaultCommand(outake.holdPiece());
    }

    private Command score(ScorePositions position) {
        return outake.score(position.outake).withName("score").asProxy();
    }

    private Command intake(IntakePositions postion) {
        return sequence(
                        wrist.moveToSetpoint(postion.wrist).asProxy(),
                        elevator.moveToSetpoint(postion.elevator).asProxy(),
                        outake.intake(postion.type).asProxy())
                .withName("intake " + postion.toString());
    }

    private Command prepareToScore(ScorePositions positions) {
        return wrist.moveToSetpoint(positions.elevator)
                .asProxy()
                .alongWith(elevator.moveToSetpoint(positions.wrist).asProxy())
                .withName("score " + positions.toString());
    }

    public Command scoreL4(BooleanSupplier delayCondition, BooleanSupplier canScore) {
        return expose(waitUntil(delayCondition)
                .andThen(prepareToScore(ScorePositions.level4)
                        .withDeadline(waitUntil(() -> canScore.getAsBoolean()
                                        && elevator.atSetpoint.getAsBoolean()
                                        && wrist.atSetpoint.getAsBoolean())
                                .andThen(score(ScorePositions.level4))))
                .withName("Level 4"));
    }

    public Command scoreL3(BooleanSupplier delayCondition, BooleanSupplier canScore) {
        return expose(waitUntil(delayCondition)
                .andThen(prepareToScore(ScorePositions.level3)
                        .withDeadline(waitUntil(() -> canScore.getAsBoolean()
                                        && elevator.atSetpoint.getAsBoolean()
                                        && wrist.atSetpoint.getAsBoolean())
                                .andThen(score(ScorePositions.level3))))
                .withName("Level 3"));
    }

    public Command scoreL2(BooleanSupplier delayCondition, BooleanSupplier canScore) {
        return expose(waitUntil(delayCondition)
                .andThen(prepareToScore(ScorePositions.level2)
                        .withDeadline(waitUntil(() -> canScore.getAsBoolean()
                                        && elevator.atSetpoint.getAsBoolean()
                                        && wrist.atSetpoint.getAsBoolean())
                                .andThen(score(ScorePositions.level2)))
                        .withName("Level 2")));
    }

    public Command scoreL1(BooleanSupplier delayCondition, BooleanSupplier canScore) {
        return expose(waitUntil(delayCondition)
                .andThen(prepareToScore(ScorePositions.level1)
                        .withDeadline(waitUntil(() -> canScore.getAsBoolean()
                                        && elevator.atSetpoint.getAsBoolean()
                                        && wrist.atSetpoint.getAsBoolean())
                                .andThen(score(ScorePositions.level1)))
                        .withName("Level 1")));
    }

    public Command holdHigh() {
        return expose(prepareToScore(ScorePositions.hold));
    }

    public Command idle() {
        return parallel(elevator.idleCommand(), wrist.idleCommand(), outake.idleCommand())
                .withName("subsystem idle");
    }

    public Command scoreBarge(BooleanSupplier delayCondition, BooleanSupplier canScore) {
        return expose(waitUntil(delayCondition)
                .andThen(prepareToScore(ScorePositions.barge)
                        .withDeadline(waitUntil(() -> canScore.getAsBoolean()
                                        && elevator.atSetpoint.getAsBoolean()
                                        && wrist.atSetpoint.getAsBoolean())
                                .andThen(score(ScorePositions.barge)))
                        .withName("Barge")));
    }

    public Command scoreProcessor(BooleanSupplier canScore) {
        return expose(prepareToScore(ScorePositions.processor)
                .withDeadline(waitUntil(() -> canScore.getAsBoolean()
                                && elevator.atSetpoint.getAsBoolean()
                                && wrist.atSetpoint.getAsBoolean())
                        .andThen(score(ScorePositions.processor)))
                .withName("Processor"));
    }

    public Command intakeCoral() {
        return expose(intake(IntakePositions.hopperCoral).withName("Intake Coral"));
    }

    public Command intakeAlgaeGround() {
        return expose(intake(IntakePositions.algaeGround).withName("Ground Algae"));
    }

    public Command intakeAlgaeL2() {
        return expose(intake(IntakePositions.algaeL2).withName("Algae L2"));
    }

    public Command intakeAlgaeL3() {
        return expose(intake(IntakePositions.algaeL3).withName("Algae L3"));
    }
}
