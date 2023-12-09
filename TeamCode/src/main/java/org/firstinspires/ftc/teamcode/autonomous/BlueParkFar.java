package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous.Case.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Intake.*;
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
import org.firstinspires.ftc.teamcode.movement.Vec;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
@Autonomous(name = "BlueParkFar")
public class BlueParkFar extends AbstractAutonomous {
    private AsymConstraints boardConstraints = new AsymConstraints(20, 30, 30);
    private Pose start = new Pose(-40, 62, PI / 2);
    private Pose dropLeft = new Pose(-30, 28, 3.8);
    private Pose dropCenter = new Pose(-36, 16, -1);
    private Pose dropRight = new Pose(-43, 28, -1);
    private Pose mid1 = new Pose(-24, 10, 0);
    private Pose mid2 = new Pose(12, 10, 0);
    private Pose board = new Pose(52, 36, 0);
    private Pose park = new Pose(44, 12, 0);
    @Override
    public void initAutonomous() {
        side = Side.BLUE;
        detector = new PropDetector(this, true, side);
        robot.drive.setPose(start);
        robot.drive.setMoveConstraints(new AsymConstraints(20, 30, 30));
        robot.drive.setTurnConstraints(new AsymConstraints(1.5, 3, 3));
        Command traj1Left = new TrajCommandBuilder(robot.drive, start)
                .lineTo(dropLeft)
                .pause(0.5)
                .marker(0, 0.25, FnCommand.once(t -> robot.intake.setRoller(rollerUp)))
                .setVf(20)
                .splineTo(mid1.vec(), 0)
                .build(scheduler);
        Command traj1Center = new TrajCommandBuilder(robot.drive, start)
                .lineTo(dropCenter)
                .pause(0.5)
                .marker(0, 0.25, FnCommand.once(t -> robot.intake.setRoller(rollerUp)))
                .setVf(20)
                .splineTo(mid1.vec(), 0)
                .build(scheduler);
        Command traj1Right = new TrajCommandBuilder(robot.drive, start)
                .lineTo(dropRight)
                .pause(0.5)
                .marker(0, 0.25, FnCommand.once(t -> robot.intake.setRoller(rollerUp)))
                .setVf(20)
                .splineTo(mid1.vec(), 0)
                .build(scheduler);
        Command traj2 = new TrajCommandBuilder(robot.drive, mid1, new Vec(20, 0))
                .lineTo(mid2.vec())
                .pause(15)
                .setMoveConstraints(boardConstraints)
                .splineTo(board, 0.3)
                .marker(FnCommand.once(t -> robot.stateMachine.transition(DEPOSIT, 700, 0)))
                .pause(1)
                .marker(FnCommand.once(t -> robot.stateMachine.transition(RETRACT)))
                .lineTo(park)
                .marker(robot.lift.goBack())
                .build(scheduler);
        scheduler.schedule(new SeqCommand(new SwitchCommand<>(() -> runCase,
                new Pair<>(LEFT, traj1Left), new Pair<>(CENTER, traj1Center), new Pair<>(RIGHT, traj1Right)),
                traj2, FnCommand.once(t -> end())));
    }
}
