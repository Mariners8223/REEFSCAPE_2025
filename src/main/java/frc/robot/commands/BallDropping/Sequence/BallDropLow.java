package frc.robot.commands.BallDropping.Sequence;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BallDropping.BallDropOff;
import frc.robot.commands.BallDropping.BallDropOnForLow;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.BallDropping.BallDroppingConstants;

public class BallDropLow extends SequentialCommandGroup {
    public BallDropLow(BallDropping ballDropping) {
        addCommands(
                 new ParallelDeadlineGroup(
                    new WaitCommand(BallDroppingConstants.LOW_BALL_DROP_TIME),
                    new BallDropOnForLow(ballDropping)
                ),
                 new BallDropOff(ballDropping)
        );
    }
}