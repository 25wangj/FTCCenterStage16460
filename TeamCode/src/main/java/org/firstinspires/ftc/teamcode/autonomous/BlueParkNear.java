package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous.Case.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import android.util.Pair;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.SwitchCommand;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage.Side;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
@Autonomous(name = "BlueParkNear")
public class BlueParkNear extends AbstractAutonomous {
    private AsymConstraints boardConstraints = new AsymConstraints(60, 80, 40);
    private Pose start = new Pose(17, 62, PI / 2);
    private Pose dropLeft = new Pose(31, 34, 0);
    private Pose dropCenter = new Pose(24, 26, 0.2);
    private Pose dropRight = new Pose(9, 34, 0);
    private Pose board = new Pose(56, 36, 0);
    private Pose park = new Pose(46, 60, 0);
    @Override
    public void initAutonomous() {
        side = Side.BLUE;
        endPose = park;
        detector = new PropDetector(this, false, side);
        robot.drive.setPose(start);
        Command traj1Left = new TrajCommandBuilder(robot.drive, start)
                .lineTo(dropLeft)
                .pause(0.5)
                .marker(FnCommand.once(t -> robot.intake.setPower(-0.3)))
                .setMoveConstraints(boardConstraints)
                .splineTo(board, 0)
                .marker(FnCommand.once(t -> robot.stateMachine.transition(DEPOSIT, 240, armLeft)))
                .build(scheduler);
        Command traj1Center = new TrajCommandBuilder(robot.drive, start)
                .lineTo(dropCenter)
                .pause(0.5)
                .marker(FnCommand.once(t -> robot.intake.setPower(-0.3)))
                .setMoveConstraints(boardConstraints)
                .splineTo(board, 0)
                .marker(FnCommand.once(t -> robot.stateMachine.transition(DEPOSIT, 240, 0)))
                .build(scheduler);
        Command traj1Right = new TrajCommandBuilder(robot.drive, start)
                .lineTo(dropRight)
                .pause(0.5)
                .marker(FnCommand.once(t -> robot.intake.setPower(-0.3)))
                .setMoveConstraints(boardConstraints)
                .splineTo(board, 0)
                .marker(FnCommand.once(t -> robot.stateMachine.transition(DEPOSIT, 240, armRight)))
                .build(scheduler);
        Command traj2 = new TrajCommandBuilder(robot.drive, board)
                .pause(0.75)
                .marker(FnCommand.once(t -> robot.stateMachine.transition(RETRACT)))
                .lineTo(park)
                .marker(robot.lift.goBack())
                .build(scheduler);
        scheduler.schedule(new SeqCommand(new SwitchCommand<>(() -> runCase,
                new Pair<>(LEFT, traj1Left), new Pair<>(CENTER, traj1Center), new Pair<>(RIGHT, traj1Right)),
                traj2, FnCommand.once(t -> end())));
    }
}
