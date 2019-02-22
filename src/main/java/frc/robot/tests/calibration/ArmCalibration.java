package frc.robot.tests.calibration;

import frc.robot.helpers.RotationMotor;
import frc.robot.networking.Network;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Motor;
import frc.robot.tests.calibration.CalibrationStep.CalibrationMethods;

public class ArmCalibration extends CalibrationProcedure {
    ArmSubsystem armSubsystem;

    public ArmCalibration(String name, CalibrationProcedure.ProcedureDelegate delegate) {
        super(name, delegate);
        armSubsystem = ArmSubsystem.getInstance();
        Network network = Network.getInstance();
        /*this.addStep(
            "1) Zero The Arm",
            "Instructions...",
            zeroArmMethods()
        ).addStep(
            "2) Rotate the shoulder", "Instructions...", rotateMotorMethods(Motor.SHOULDER_JOINT, false)
        ).addStep(
            "3) Rotate the elbow", "Intructions...", rotateMotorMethods(Motor.ELBOW_JOINT, true)
        ).addStep("4) Rezero the arm", "Instructions...", zeroArmMethods());*/
    }

    private CalibrationStep.CalibrationMethods testJoint(Motor motorType, boolean lowering) {
        return new CalibrationMethods(){
            @Override
            public boolean run(CalibrationStep step) {
                RotationMotor motor = armSubsystem.getRotationMotor(motorType);
                return false;
            }
        
            @Override
            public void onFinish(CalibrationStep step) {
                
            }
        };
    }

    private CalibrationStep.CalibrationMethods rotateMotorMethods(Motor motorType, boolean lowering) {
        return new CalibrationStep.CalibrationMethods() {
            @Override
            public boolean run(CalibrationStep step) {
                RotationMotor motor = armSubsystem.getRotationMotor(motorType);
                int difference = motor.getPosition() - motor.homeQuadraturePosition;
                Object sumObj = step.data.get("sum");
                if (sumObj == null) step.data.put("sum", difference);
                else {
                    int sum = (int) sumObj;
                    if (Math.abs(sum) < 100000) sum += difference;
                    if (Math.abs(sum) > 1000) {
                        step.instructions = "Lower the arm back to zero";
                        step.setFinishable(true);
                    }
                }
                return false;
            }
        
            @Override
            public void onFinish(CalibrationStep step) {
                int sum = (int) step.data.get("sum");
                RotationMotor motor = armSubsystem.getRotationMotor(motorType);
                motor.setInvertedPosition(lowering ? sum > 0 : sum < 0);
            }
        };
    }
    private CalibrationStep.CalibrationMethods zeroArmMethods() {
        return new CalibrationStep.CalibrationMethods() {
            @Override
            public boolean run(CalibrationStep step) {
                step.setFinishable(true);
                return false;
            }
        
            @Override
            public void onFinish(CalibrationStep step) {
                RotationMotor shoulderMotor = armSubsystem.getRotationMotor(Motor.SHOULDER_JOINT);
                shoulderMotor.initializeHome(25);
                RotationMotor elbowMotor = armSubsystem.getRotationMotor(Motor.ELBOW_JOINT);
                elbowMotor.initializeHome(180);
            }
        };
    }
}