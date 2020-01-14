package frc.paths;

public class GenPathSetup{
    //this class gives us the location of an item in the class spit out by bob trajectory
    //basically this way we don't get the numbers mixed up
    public GenPathSetup(){

    }
    // dt 1, x 2 ,y 3 ,left.pos 4 ,left.vel 5 ,left.acc 6, left.jerk 7 ,center.pos 8 ,center.vel 9 ,center.acc 10 ,center.jerk 11 ,right.pos 12 ,right.vel 13 ,right.acc 14 ,right.jerk 15,heading 16
    public static int dt(){
        return 0;
    }
    public static int x(){
        return 1;
    }
    // dt 1, x 2 ,y 3 ,left.pos 4 ,left.vel 5 ,left.acc 6, left.jerk 7 ,center.pos 8 ,center.vel 9 ,center.acc 10 ,center.jerk 11 ,right.pos 12 ,right.vel 13 ,right.acc 14 ,right.jerk 15,heading 16
   
    public static int y(){
        return 2;
    }public static int leftPos(){
        return 3;
    }public static int leftVel(){
        return 4;
    }public static int leftAcc(){
        return 5;
    }public static int leftJerk(){
        return 6;
    }public static int centerPos(){
        return 7;
    }public static int centerVel(){
        return 8;
    }public static int centerAcc(){
        return 9;
    }public static int centerJerk(){
        return 10;
    }// dt 1, x 2 ,y 3 ,left.pos 4 ,left.vel 5 ,left.acc 6, left.jerk 7 ,center.pos 8 ,center.vel 9 ,center.acc 10 ,center.jerk 11 ,right.pos 12 ,right.vel 13 ,right.acc 14 ,right.jerk 15,heading 16
   
    public static int rightPos(){
        return 11;
    }public static int rightVel(){
        return 12;
    }public static int rightAcc(){
        return 13;
    }public static int rightJerk(){
        return 14;
    }public static int heading(){
        return 15;
    }

}