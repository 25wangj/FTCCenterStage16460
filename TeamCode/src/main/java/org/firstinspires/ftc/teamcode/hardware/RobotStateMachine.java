package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Climb.*;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.StateMachine;
import org.firstinspires.ftc.teamcode.command.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
public class RobotStateMachine {
    public enum robotStates {
        INTAKE_CLOSED, INTAKE_OPEN, EJECT_CLOSED, EJECT_OPEN, IDLE, DEPOSIT, RETRACT, LAUNCH, RELEASE, CLIMB
    }
    public static StateMachine<robotStates> get(CommandOpMode opMode, Robot robot, robotStates state) {
        Subsystem[] subsystems = new Subsystem[] {robot.intake, robot.lift};
        StateMachineBuilder<robotStates> builder = new StateMachineBuilder<robotStates>(opMode)
                .addState(robotStates.values())
                .addTransition(robotStates.INTAKE_CLOSED, robotStates.EJECT_CLOSED, FnCommand.once(t -> {
                robot.intake.setRoller(rollerUp);
                robot.intake.setPower(-1);}, subsystems))
                .addTransition(robotStates.EJECT_CLOSED, robotStates.INTAKE_CLOSED, FnCommand.once(t -> {
                robot.intake.setRoller(rollerDown);
                robot.intake.setPower(1);}, subsystems))
                .addTransition(robotStates.INTAKE_OPEN, robotStates.EJECT_OPEN, FnCommand.once(t -> {
                robot.intake.setRoller(rollerUp);
                robot.intake.setPower(-1);}, subsystems))
                .addTransition(robotStates.EJECT_OPEN, robotStates.INTAKE_OPEN, FnCommand.once(t -> {
                robot.intake.setRoller(rollerDown);
                robot.intake.setPower(1);}, subsystems))
                .addTransition(robotStates.INTAKE_CLOSED, robotStates.INTAKE_OPEN, new SeqCommand(
                    new WaitCommand(t -> {
                        robot.intake.setGate(gateOpen);
                        robot.intake.setPower(0);}, 0.2, subsystems),
                    new WaitCommand(t -> robot.intake.setPower(1), 0.4, subsystems),
                    new WaitCommand(t -> robot.intake.setPower(-0.5), 0.15, subsystems),
                    FnCommand.once(t -> robot.intake.setPower(1), subsystems)))
                .addTransition(robotStates.INTAKE_OPEN, robotStates.INTAKE_CLOSED, new WaitCommand(t -> {
                robot.intake.setGate(gateClosed);
                robot.intake.setPower(1);}, 0.25, subsystems))
                .addTransition(robotStates.INTAKE_OPEN, robotStates.DEPOSIT, d -> new SeqCommand(
                new WaitCommand(t -> robot.intake.setGate(gatePush), 0.15, subsystems),
                new WaitCommand(t -> robot.intake.setGate(gateUp), 0.25, subsystems),
                FnCommand.once(t -> {
                    robot.lift.setClaw(clawClosed);
                    robot.intake.setPower(-0.5);}, subsystems),
                robot.lift.goTo(d[0], d[1], 0.25),
                FnCommand.once(t -> robot.intake.setPower(0), subsystems)))
                .addTransition(robotStates.DEPOSIT, robotStates.RETRACT, d -> {
                    if (d[0] == 0) {
                        return new SeqCommand(new WaitCommand(t -> robot.lift.setClaw(clawOpen), 0.08, subsystems),
                            new WaitCommand(t -> robot.lift.setClaw(clawClosed), 0.15, subsystems),
                            new WaitCommand(t -> robot.lift.setClaw(clawOpen), 0.15, subsystems));
                    } else {
                        return new WaitCommand(t -> robot.lift.setClaw(clawOpen), 0.15, subsystems);
                    }
                })
                .addTransition(robotStates.RETRACT, robotStates.DEPOSIT, new WaitCommand(t ->
                robot.lift.setClaw(clawClosed), 0.25, subsystems))
                .addTransition(robotStates.RETRACT, robotStates.INTAKE_CLOSED, new SeqCommand(
                    new ParCommand(robot.lift.goBack(), FnCommand.once(t -> {
                        robot.intake.setGate(gateOpen);
                        robot.intake.setRoller(rollerDown);}, robot.intake)),
                    new WaitCommand(t -> robot.intake.setGate(gateClosed), 0.25, subsystems),
                    FnCommand.once(t -> robot.intake.setPower(1), subsystems)))
                .addTransition(robotStates.RETRACT, robotStates.INTAKE_OPEN, new SeqCommand(
                    new ParCommand(robot.lift.goBack(), FnCommand.once(t -> {
                        robot.intake.setGate(gateOpen);
                        robot.intake.setRoller(rollerDown);}, robot.intake)),
                    FnCommand.once(t -> robot.intake.setPower(1), subsystems)))
                .addTransition(robotStates.RETRACT, robotStates.IDLE,
                    new ParCommand(robot.lift.goBack(), FnCommand.once(t -> {
                        robot.intake.setGate(gateOpen);
                        robot.intake.setRoller(rollerDown);}, robot.intake)))
                .addTransition(robotStates.IDLE, robotStates.INTAKE_CLOSED, FnCommand.once(t -> {
                    robot.intake.setGate(gateClosed);
                    robot.intake.setPower(1);
                }))
                .addTransition(robotStates.IDLE, robotStates.INTAKE_OPEN, FnCommand.once(t -> {
                    robot.intake.setPower(1);
                }))
                .addTransition(robotStates.INTAKE_OPEN, robotStates.LAUNCH, new WaitCommand(t -> {
                    robot.climb.setPlane(planeRelease);
                    robot.intake.setPower(0);}, 0.25, subsystems))
                .addTransition(robotStates.LAUNCH, robotStates.INTAKE_OPEN, new WaitCommand(t -> {
                    robot.climb.setPlane(planeHold);
                    robot.intake.setPower(1);}, 0.25, subsystems))
                .addTransition(robotStates.LAUNCH, robotStates.RELEASE, robot.climb.release())
                .addTransition(robotStates.RELEASE, robotStates.CLIMB, FnCommand.once(
                        t -> robot.climb.climb(), subsystems));
        return builder.build(state);
    }
}