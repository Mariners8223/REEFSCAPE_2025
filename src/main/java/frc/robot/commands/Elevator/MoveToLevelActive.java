package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;

import java.util.function.Supplier;


public class MoveToLevelActive extends Command {
    private final Elevator elevator;
    private final Supplier<ElevatorConstants.ElevatorLevel> targetLevelSupplier;

    public MoveToLevelActive(Elevator elevator, Supplier<ElevatorConstants.ElevatorLevel> targetLevelSupplier) {
        this.elevator = elevator;
        this.targetLevelSupplier = targetLevelSupplier;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevator);
    }

    @Override
    public void execute() {
        if(targetLevelSupplier.get() != null && targetLevelSupplier.get() != elevator.getDesiredLevel())
            elevator.moveMotorByPosition(targetLevelSupplier.get());        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.moveMotorByPosition(ElevatorConstants.ElevatorLevel.Bottom);
    }
}
