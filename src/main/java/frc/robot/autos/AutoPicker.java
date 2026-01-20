package frc.robot.autos;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoPicker {
    private Autos[] autoList;
    public AutoPicker(){
       autoList = new Autos[3]; //TODO Change this number when adding paths
       autoList[0] = new Autos("All Fuel Areas", allFuelPath());
       autoList[1] = new Autos("CW Three Loops", loopsCWPath());
       autoList[2] = new Autos("CCW Three Looped", loopsCCWPath());

    }

    public Command allFuelPath(){
        try{
            PathPlannerPath intake = PathPlannerPath.fromChoreoTrajectory("AllFuelAreas");
            return AutoBuilder.followPath(intake);
        } catch (Exception e){
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

        public Command loopsCWPath(){
        try{
            PathPlannerPath intake = PathPlannerPath.fromChoreoTrajectory("ThreeLoopsCW");
            return AutoBuilder.followPath(intake);
        } catch (Exception e){
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

        public Command loopsCCWPath(){
        try{
            PathPlannerPath intake = PathPlannerPath.fromChoreoTrajectory("ThreeLoopedCCW");
            return AutoBuilder.followPath(intake);
        } catch (Exception e){
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    
    public Autos[] getAutoList(){
        return autoList;
    }
}
