package frc.robot.utils;

/** Enum to store can ids */
public class IDMap {
    public enum CAN {
        //drive motors
        leftFront(1),
        leftRear(11),
        rightFront(2),
        rightRear(21),
        
        //testing motors
        testingFalcon(3),
        testingNeo(4);

        public final int ID;
        private CAN(int id) {
            ID = id;
        }
    }
}
