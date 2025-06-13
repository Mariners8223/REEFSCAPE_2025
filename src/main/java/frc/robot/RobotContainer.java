// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

//#region Imports
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.FeederLocation;
import frc.robot.Constants.FeederSide;
import frc.robot.Constants.ReefLocation;
import frc.robot.Constants.RobotType;
import frc.robot.commands.BallDropping.BallDropOff;
import frc.robot.commands.BallDropping.BallDropOnForHigh;
import frc.robot.commands.BallDropping.BallDropOnForLow;
import frc.robot.commands.BallDropping.Sequence.BallDropHigh;
import frc.robot.commands.BallDropping.Sequence.BallDropLow;
import frc.robot.commands.Climb.ClimbCommand;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.commands.Elevator.MoveToLevel;
import frc.robot.commands.Elevator.MoveToLevelActive;
import frc.robot.commands.EndEffector.EjectSequance;
import frc.robot.commands.EndEffector.MiniEject;
import frc.robot.commands.EndEffector.Funnel.ToggleFunnel;
import frc.robot.commands.EndEffector.Intake.Intake;
import frc.robot.commands.EndEffector.Intake.IntakeStep2;
import frc.robot.commands.MasterCommand.*;
import frc.robot.subsystems.BallDropping.BallDropping;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.MotorPower;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LED.StripControl;
import frc.robot.subsystems.RobotAuto.RobotAuto;

import frc.robot.subsystems.Vision.Vision;
import frc.util.Elastic;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.DriveTrain.DriveBase;

import static frc.robot.Blackbox.*;
//#endregion

public class RobotContainer {
    // public static DriveBase driveBase;
    public static Elevator elevator;
    public static EndEffector endEffector;
    public static BallDropping ballDropping;
    public static RobotAuto robotAuto;
    public static Vision vision;
    public static Climb climb;
    // public static LED led;

    public static LoggedDashboardChooser<Command> autoChooser;

    public static LoggedDashboardChooser<FeederSide> feederSideChooser;


    public static CommandXboxController driveController;
    public static CommandGenericHID operatorController;

    public RobotContainer() {
        //#region Initialisation
        
        driveController = new CommandXboxController(0);
        operatorController = new CommandGenericHID(1);

        // driveBase = new DriveBase();
        elevator = new Elevator();
        endEffector = new EndEffector();
        ballDropping = new BallDropping();
        climb = new Climb();
        // led = new LED();
        // robotAuto = new RobotAuto();
        // vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose, HomeToReef::isRunning);

        // endEffector.setDefaultCommand(new Intake(endEffector));

        if(Constants.ROBOT_TYPE == Constants.RobotType.COMPETITION){
            new Trigger(DriverStation::isDSAttached).onTrue(
                new InstantCommand(() -> {
                    Elastic.selectTab(0);
                    Logger.recordOutput("Elastic Tab", "auto");
                }).ignoringDisable(true)
            );
        }
        //#endregion
        controllerCircle.onTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    endEffectorMotorLeftDutyCycle((1/3)),
                     endEffectorMotorRightDutyCycle((-1/3))
                ),
               new WaitCommand(3),
               new ParallelCommandGroup(
                endEffectorMotorLeftStop(),
                endEffectorMotorRightStop()
               
               )

            )
        );

        //#region Logs
        if(Constants.ROBOT_TYPE == RobotType.DEVELOPMENT) HomeToReef.pidTune();

        Trigger robotReadyClimb = new Trigger(() -> Timer.getMatchTime() < 30 && endEffector.isFunnelInClimb());

        robotReadyClimb.onTrue(new InstantCommand(() -> {
            Logger.recordOutput("Elastic Tab", "EndGame");
            Elastic.selectTab(2);
        }));

        new Trigger(() -> !endEffector.isFunnelInClimb()).onTrue(new InstantCommand(() -> {
            DriveCommand.normalSpeed();
            Logger.recordOutput("Elastic Tab", "Telop");
            Elastic.selectTab(1);
        }));
        //#endregion
    }

    //#region Configuration
    public static void configFeederChooser(){
        feederSideChooser = new LoggedDashboardChooser<>("Feeder side chooser");

        feederSideChooser.addDefaultOption("Close", FeederSide.CLOSE);

        feederSideChooser.addOption("Away", FeederSide.AWAY);
    }

    public static void configureOperatorBinding() {
        //port 2-12 + A0 are reefs in order (2 to A0) (2 is 1, 3 is 2 and so on) (A0 is 12)
        //port A1 is for ball dropping
        //ports A2-A5 are elevator levels (A2 is L1, A3 is L2, A4 is L3, A5 is L4)

        for (int i = 0; i < 12; i++) {
            ReefLocation location = ReefLocation.values()[i];

            operatorController.button(i + 1).onTrue(new InstantCommand(() -> robotAuto.setSelectedReef(location)));
        }

        for (int i = 0; i < 4; i++) {
            ElevatorLevel level = ElevatorLevel.values()[i + 1];

            operatorController.pov(i * 90).onTrue(new InstantCommand(() -> robotAuto.setSelectedLevel(level)));
        }

        BooleanSupplier isBallDropUp = () -> {
            ReefLocation selectedReef = robotAuto.getSelectedReef();

            if(selectedReef == null) return true;

            return selectedReef.isBallInUpPosition();
        };

        operatorController.button(13).whileTrue(new SequentialCommandGroup(
            new BallDropOnForLow(ballDropping).onlyIf(() -> !isBallDropUp.getAsBoolean()),
            new BallDropOnForHigh(ballDropping).onlyIf(isBallDropUp)
        ));
        operatorController.button(13).onFalse(new BallDropOff(ballDropping));

        //funnel flipping
        operatorController.axisLessThan(0, -0.5).and(() -> !endEffector.isGpLoaded()).onTrue(new ToggleFunnel(endEffector));

        //climb
        operatorController.axisLessThan(2, -0.5).and(() ->
                Timer.getMatchTime() <= 30 && endEffector.isFunnelInClimb()).whileTrue(new ClimbCommand(climb));

        //manual intake
        operatorController.axisLessThan(2, -0.5).and(() ->
                !endEffector.isFunnelInClimb()).whileTrue(new MiniEject(endEffector, elevator::getCurrentLevel, robotAuto::getSelectedReef));
    }

    public static ReefLocation configureTargetReefSupplier() {
        int reef = (int) SmartDashboard.getNumber("target Reef", 1);

        reef = MathUtil.clamp(reef, 1, 12);

        return ReefLocation.values()[reef - 1];
    }

    public static ElevatorLevel configureLevelSupplier() {
        double shit = SmartDashboard.getNumber("target Level", 1);
        System.out.println("woop woop " + shit);
        int level = (int) shit;

        level = MathUtil.clamp(level, 1, 4);

        System.out.println("new selcted level: " + level);

        return ElevatorLevel.values()[level];
    }

    public static Command getAutoCommand() {
        return autoChooser.get();
    }

    private static final BooleanSupplier checkForPathChoiceUpdate = new BooleanSupplier() {
        private String lastAutoName = "InstantCommand";

        @Override
        public boolean getAsBoolean() {
            if (autoChooser.get() == null) return false;

            String currentAutoName = autoChooser.get().getName();

            try {
                return !Objects.equals(lastAutoName, currentAutoName);
            } finally {
                lastAutoName = currentAutoName;
            }

        }
    };

    private void configChooser() {
        List<String> namesOfAutos = AutoBuilder.getAllAutoNames();
        List<PathPlannerAuto> autosOfAutos = new ArrayList<>();

        autoChooser = new LoggedDashboardChooser<>("chooser");
        for (String autoName : namesOfAutos) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);
            autosOfAutos.add(auto);
        }

        autosOfAutos.forEach(auto -> autoChooser.addOption(auto.getName(), auto));

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        SmartDashboard.putData("chooser", autoChooser.getSendableChooser());

        new Trigger(RobotState::isEnabled).and(RobotState::isTeleop).onTrue(new InstantCommand(() -> Robot.clearObjectPoseField("AutoPath")).ignoringDisable(true));
        new Trigger(RobotState::isDisabled).and(checkForPathChoiceUpdate).onTrue(new InstantCommand(() -> updateFieldFromAuto(autoChooser.get().getName())).ignoringDisable(true));
    }

    private static void updateFieldFromAuto(String autoName) {
        List<Pose2d> poses = new ArrayList<>();

        try {
            // boolean invert =
            //         DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

            PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(path -> {
//                path = invert ? path.flipPath() : path;
                //no need cause the field inverts the path

                poses.addAll(path.getPathPoses());
            });
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Error loading auto path", e.getStackTrace());
        }

        Robot.setTrajectoryField("AutoPath", poses);
    }
    //#endregion
}
