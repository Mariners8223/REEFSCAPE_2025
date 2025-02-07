package frc.robot.commands.BallDropping.Sequence;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BallDropping.BallDropOff;
import frc.robot.commands.BallDropping.BallDropOnForHigh;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.BallDropping.BallDroppingConstants;

public class BallDropHigh extends SequentialCommandGroup {
    public BallDropHigh(BallDropping ballDropping) {
       addCommands(
               new BallDropOnForHigh(ballDropping),
               new WaitCommand(BallDroppingConstants.HIGH_BALL_DROP_TIME),
               new BallDropOff(ballDropping)
       );
    }
}