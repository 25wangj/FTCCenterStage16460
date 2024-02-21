package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Double.NaN;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous.Case.*;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import android.util.Pair;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.SwitchCommand;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage.Side;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.vision.Vision;
@Photon
@Autonomous(name = "RedNear")
public class RedNear extends AbstractAutonomous {
    private AsymConstraints decel1 = new AsymConstraints(60, 70, 25);
    private AsymConstraints decel2 = new AsymConstraints(60, 70, 40);
    private AsymConstraints slow = new AsymConstraints(15, 40, 40);
    private Pose start = new Pose(17, -63, -PI / 2);
    private Pose left = new Pose(10, -36, 0);
    private Pose center = new Pose(24, -26, 0);
    private Pose right = new Pose(30, -36, 0);
    private Pose board = new Pose(55, -36, 0);
    private Pose midInt = new Pose(0, -12, 0);
    private Pose midStack = new Pose(-60, -16, 0);
    private Pose midBoard = new Pose(53, -25, 0);
    private int config = 0;
    @Override
    public void initAutonomous() {
        while (config == 0 && !isStopRequested()) {
            telemetry.addLine("Press a for middle lane, b for outer lane");
            telemetry.update();
            if (gamepad1.a) {
                config = 1;
            } else if (gamepad1.b) {
                config = 4;
            }
        }
        while (config % 3 == 1 && !isStopRequested()) {
            telemetry.addLine("Press x for park, y for cycle");
            telemetry.update();
            if (gamepad1.x) {
                config--;
            } else if (gamepad1.y) {
                config++;
            }
        }
        side = Side.RED;
        vision = new Vision(this, true, false, side);
        robot.drive.setPose(start);
        Command traj1Left = new TrajCommandBuilder(robot.drive, start)
                .lineTo(left)
                .pause(0.25)
                .marker(FnCommand.once(t -> robot.intake.setRoller(rollerUp)))
                .setMoveConstraints(decel2)
                .lineTo(board)
                .marker(FnCommand.once(t -> robot.stateMachine.transition(DEPOSIT, 240, armLeft)))
                .build(scheduler);
        Command traj1Center = new TrajCommandBuilder(robot.drive, start)
                .lineTo(center)
                .pause(0.25)
                .marker(FnCommand.once(t -> robot.intake.setRoller(rollerUp)))
                .setMoveConstraints(decel2)
                .lineTo(board)
                .marker(FnCommand.once(t -> robot.stateMachine.transition(DEPOSIT, 240, 0)))
                .build(scheduler);
        Command traj1Right = new TrajCommandBuilder(robot.drive, start)
                .lineTo(right)
                .pause(0.25)
                .marker(FnCommand.once(t -> robot.intake.setRoller(rollerUp)))
                .setMoveConstraints(decel2)
                .lineTo(board)
                .marker(FnCommand.once(t -> robot.stateMachine.transition(DEPOSIT, 240, armRight)))
                .build(scheduler);
        Command traj2;
        if (config == 0) {
            traj2 = new TrajCommandBuilder(robot.drive, board)
                    .pause(0.5)
                    .marker(FnCommand.once(t -> robot.stateMachine.transition(RETRACT, 1)))
                    .setTangent(PI)
                    .back(6)
                    .marker(0, 0.5, FnCommand.once(t -> robot.stateMachine.transition(IDLE)))
                    .left(24)
                    .pause(0.5)
                    .build(scheduler);
        } else if (config == 3) {
            traj2 = new TrajCommandBuilder(robot.drive, board)
                    .pause(0.5)
                    .marker(FnCommand.once(t -> robot.stateMachine.transition(RETRACT, 1)))
                    .setTangent(PI)
                    .back(6)
                    .marker(0, 0.5, FnCommand.once(t -> robot.stateMachine.transition(IDLE)))
                    .right(24)
                    .pause(0.5)
                    .build(scheduler);
        } else if (config == 2) {
            traj2 = new TrajCommandBuilder(robot.drive, board)
                    .pause(0.5)
                    .marker(FnCommand.once(t -> robot.stateMachine.transition(RETRACT, 1)))
                    .setTangent(2.3)
                    .setVel(NaN)
                    .setMoveConstraints(decel1)
                    .splineTo(midInt, PI)
                    .marker(0, 0.5, FnCommand.once(t -> robot.stateMachine.transition(IDLE)))
                    .splineTo(midStack, PI)
                    .marker(FnCommand.once(t -> robot.intake.setRoller(rollerUp)))
                    .pause(0.25)
                    .setMoveConstraints(slow)
                    .forward(12)
                    .marker(FnCommand.once(t -> robot.intake.setRoller(rollerDown)))
                    .back(8)
                    .marker(FnCommand.once(t -> robot.stateMachine.transition(INTAKE_CLOSED)))
                    .pause(0.5)
                    .resetConstraints()
                    .setVel(NaN)
                    .setMoveConstraints(decel2)
                    .splineTo(midInt, 0)
                    .marker(FnCommand.once(t -> robot.intake.setRoller(rollerUp)))
                    .marker(0, 0.5, FnCommand.once(t -> robot.stateMachine.transition(INTAKE_OPEN)))
                    .splineTo(midBoard, -0.3)
                    .marker(FnCommand.once(t -> robot.stateMachine.transition(DEPOSIT, 800, armRight)))
                    .pause(0.75)
                    .marker(FnCommand.once(t -> robot.stateMachine.transition(RETRACT, 0)))
                    .setTangent(2.85)
                    .setVel(NaN)
                    .setMoveConstraints(decel1)
                    .splineTo(midInt, PI)
                    .marker(0, 0.5, FnCommand.once(t -> robot.stateMachine.transition(INTAKE_CLOSED)))
                    .splineTo(midStack, PI)
                    .pause(0.5)
                    .resetConstraints()
                    .setVel(NaN)
                    .setMoveConstraints(decel2)
                    .splineTo(midInt, 0)
                    .marker(FnCommand.once(t -> robot.intake.setRoller(rollerUp)))
                    .marker(0, 0.5, FnCommand.once(t -> robot.stateMachine.transition(INTAKE_OPEN)))
                    .splineTo(midBoard, -0.3)
                    .marker(FnCommand.once(t -> robot.stateMachine.transition(DEPOSIT, 800, armRight)))
                    .pause(0.75)
                    .marker(FnCommand.once(t -> robot.stateMachine.transition(RETRACT, 0)))
                    .setTangent(PI)
                    .back(6)
                    .marker(0, 0.5, FnCommand.once(t -> robot.stateMachine.transition(IDLE)))
                    .left(12)
                    .pause(0.5)
                    .build(scheduler);
        } else {
            traj2 = null;
        }
        scheduler.schedule(new SeqCommand(new SwitchCommand<>(() -> runCase,
                new Pair<>(LEFT, traj1Left), new Pair<>(CENTER, traj1Center), new Pair<>(RIGHT, traj1Right)),
                traj2, FnCommand.once(t -> end())));
    }
    @Override
    public void waitAutonomous() {
        if (config == 0) {
            telemetry.addData("Config","Middle Park");
        } else if (config == 2) {
            telemetry.addData("Config", "Middle Cycle");
        } else if (config == 3) {
            telemetry.addData("Config", "Outer Park");
        } else {
            telemetry.addData("Config", "Outer Cycle");
        }
    }
}
