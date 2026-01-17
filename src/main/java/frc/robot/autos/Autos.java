package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;

public class Autos {
    public Command path;
    public String name;
    public Autos(String name, Command path){
         this.name = name;
         this.path = path;
    }
    public String getName(){
        return name;
    }
    public Command getPath(){
        return path;
    }
}
