package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.clawClosed;
import static org.firstinspires.ftc.teamcode.hardware.Lift.clawOpen;
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
        INTAKE_CLOSED, INTAKE_OPEN, EJECT, DEPOSIT, RETRACT
    }
    public static StateMachine<robotStates> get(CommandOpMode opMode, Robot robot, robotStates state) {
        Subsystem[] subsystems = new Subsystem[] {robot.intake, robot.lift};
        StateMachineBuilder<robotStates> builder = new StateMachineBuilder<>(opMode);
        builder.addState(robotStates.values());
        builder.addTransition(robotStates.INTAKE_CLOSED, robotStates.EJECT, FnCommand.once(t -> {
                robot.intake.setRoller(rollerUp);
                robot.intake.setPower(intakeEject);}, subsystems));
        builder.addTransition(robotStates.EJECT, robotStates.INTAKE_CLOSED, FnCommand.once(t -> {
                robot.intake.setRoller(rollerDown);
                robot.intake.setPower(intakeClosed);}, subsystems));
        builder.addTransition(robotStates.INTAKE_OPEN, robotStates.EJECT, FnCommand.once(t -> {
                robot.intake.setRoller(rollerUp);
                robot.intake.setPower(intakeEject);}, subsystems));
        builder.addTransition(robotStates.EJECT, robotStates.INTAKE_OPEN, FnCommand.once(t -> {
                robot.intake.setRoller(rollerDown);
                robot.intake.setPower(intakeOpen);}, subsystems));
        builder.addTransition(robotStates.INTAKE_CLOSED, robotStates.INTAKE_OPEN, new SeqCommand(
                new WaitCommand(t -> {
                    robot.intake.setGate(gateOpen);
                    robot.intake.setPower(intakeSlow);}, 0.5, subsystems),
                new WaitCommand(t -> robot.intake.setPower(0), 0.1, subsystems),
                FnCommand.once(t -> robot.intake.setPower(1), subsystems)));
        builder.addTransition(robotStates.INTAKE_OPEN, robotStates.INTAKE_CLOSED, new WaitCommand(t -> {
                robot.intake.setGate(gateClosed);
                robot.intake.setPower(intakeClosed);}, 0.25, subsystems));
        builder.addTransition(robotStates.INTAKE_OPEN, robotStates.DEPOSIT, d -> new SeqCommand(
                new WaitCommand(t -> robot.intake.setGate(gatePush), 0.1, subsystems),
                new WaitCommand(t -> robot.intake.setGate(gateUp), 0.25, subsystems),
                FnCommand.once(t -> {
                    robot.lift.setClaw(clawClosed);
                    robot.intake.setPower(0);}, subsystems),
                robot.lift.goTo(d[0], d[1])));
        builder.addTransition(robotStates.DEPOSIT, robotStates.RETRACT, new WaitCommand(t ->
                robot.lift.setClaw(clawOpen), 0.25, subsystems));
        builder.addTransition(robotStates.DEPOSIT, robotStates.INTAKE_CLOSED, new SeqCommand(
                new ParCommand(robot.lift.goBack(), new WaitCommand(t -> {
                        robot.intake.setGate(gateOpen);
                        robot.intake.setRoller(rollerDown);}, 0.25, subsystems)),
                new WaitCommand(t -> robot.intake.setGate(gateClosed), 0.25, subsystems),
                FnCommand.once(t -> robot.intake.setPower(intakeClosed))));
        builder.addTransition(robotStates.DEPOSIT, robotStates.INTAKE_OPEN, new SeqCommand(
                new ParCommand(robot.lift.goBack(), new WaitCommand(t -> {
                    robot.intake.setGate(gateOpen);
                    robot.intake.setRoller(rollerDown);}, 0.25, subsystems)),
                FnCommand.once(t -> robot.intake.setPower(intakeOpen))));
        return builder.build(state);
    }
}