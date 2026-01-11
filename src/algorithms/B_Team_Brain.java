/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * ******************************************************/
package algorithms;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

public class B_Team_Brain extends Brain {

    //---PARAMETERS---//
    private static final double HEADINGPRECISION = 0.001;
    private static final double ANGLEPRECISION = 0.1;

    private static final int ROCKY = 0x1EADDA;
    private static final int MARIO = 0x5EC0;
    private static final int UNDEFINED = 0xBADC0DE0;

    private static final int TURNSOUTHTASK = 1;
    private static final int MOVESOUTHTASK = 2;
    private static final int TURNEASTTASK = 3;
    private static final int MOVEEASTTASK = 4;
    private static final int UTURNTASK = 5;
    private static final int UTURNAGAINTASK = 6;
    private static final int SINK = 0xBADC0DE1;

    // TURN AROUND Parameters
    private int rendezvousIndex = 0;
    private double epsilon = 5.0;
    private double[] rendezvousX = {
            Parameters.STAGEWIDTH / 4.0,
            Parameters.STAGEWIDTH / 2.0,
            3.0 * Parameters.STAGEWIDTH / 4.0
    };
    private boolean[] reached = {false, false, false};

    // ROBOT MEET POINT PARAMETERS
    private double XR = Parameters.STAGEWIDTH / 2;
    private double YR = Parameters.STAGEHEIGHT / 2;
    private double X_epsilon = 10.0;

    //---VARIABLES---//
    private int state;
    private double myX, myY;
    private boolean isMoving;
    private int whoAmI;
    private double width;
    private int ballet;

    //---CONSTRUCTORS---//
    public B_Team_Brain() { super(); }

    //---ABSTRACT-METHODS-IMPLEMENTATION---//
    public void activate() {
        // ODOMETRY CODE
        whoAmI = ROCKY;
        for (IRadarResult o : detectRadar())
            if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) whoAmI = MARIO;

        if (whoAmI == ROCKY) {
            myX = Parameters.teamBSecondaryBot2InitX;
            myY = Parameters.teamBSecondaryBot2InitY;
        } else {
            myX = 0;
            myY = 0;
        }

        // INIT
        state = (whoAmI == MARIO) ? TURNSOUTHTASK : SINK;
        isMoving = false;
        ballet = 0;
    }

    public void step() {
        // ODOMETRY CODE
        if (isMoving && whoAmI == ROCKY) {
            boolean blockedByWreck = false;
            for (IRadarResult o : detectRadar()) {
                if (o.getObjectType() == IRadarResult.Types.Wreck &&
                        isSameDirection(o.getObjectDirection(), getHeading())) {
                    blockedByWreck = true;
                    break;
                }
            }

            boolean blockedByWall = (detectFront().getObjectType() == IFrontSensorResult.Types.WALL);

            if (!blockedByWreck && !blockedByWall) {
                myX += Parameters.teamBSecondaryBotSpeed * Math.cos(myGetHeading());
                myY += Parameters.teamBSecondaryBotSpeed * Math.sin(myGetHeading());
            }
            isMoving = false;
        }

        // DEBUG MESSAGE
        if (whoAmI == MARIO && state != SINK) {
            sendLogMessage("#MARIO *thinks* (x,y)= (" + (int) myX + ", " + (int) myY +
                    ") and theta= " + (int) (myGetHeading() * 180 / Math.PI) + "°.");
        }

        // Turn around
        if (whoAmI == ROCKY) {
            for (int i = 0; i < rendezvousX.length; i++) {
                if (Math.abs(myX - rendezvousX[i]) < epsilon && !reached[i]) {
                    reached[i] = true;      // mark a point as reached
                    state = UTURNTASK;
                    myMove();
                    return;
                } else if (Math.abs(myX - rendezvousX[i]) >= epsilon) {
                    reached[i] = false;     // we can redo the point
                }
            }
        }

        // AUTOMATON
        if (state == TURNSOUTHTASK && !isSameDirection(myGetHeading(), Parameters.SOUTH)) {
            stepTurn(Parameters.Direction.RIGHT);
            return;
        }
        if (state == TURNSOUTHTASK && isSameDirection(myGetHeading(), Parameters.SOUTH)) {
            state = MOVESOUTHTASK;
            myMove();
            return;
        }
        if (state == MOVESOUTHTASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL) {
            myMove();
            return;
        }
        if (state == MOVESOUTHTASK && detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
            state = TURNEASTTASK;
            stepTurn(Parameters.Direction.LEFT);
            return;
        }
        if (state == TURNEASTTASK && !isSameDirection(myGetHeading(), Parameters.EAST)) {
            stepTurn(Parameters.Direction.LEFT);
            return;
        }
        if (state == TURNEASTTASK && isSameDirection(myGetHeading(), Parameters.EAST)) {
            state = MOVEEASTTASK;
            myMove();
            return;
        }
        if (state == MOVEEASTTASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL && myX < 1000 && ballet == 0) {
            myMove();
            return;
        }
        if (state == MOVEEASTTASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL && myX >= 1000 && ballet == 0) {
            ballet = 1;
            state = UTURNTASK;
            stepTurn(Parameters.Direction.RIGHT);
            return;
        }
        if (state == MOVEEASTTASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL && myX < 1500 && ballet == 1) {
            myMove();
            return;
        }
        if (state == MOVEEASTTASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL && myX >= 1500 && ballet == 1) {
            ballet = 2;
            state = UTURNTASK;
            stepTurn(Parameters.Direction.RIGHT);
            return;
        }
        if (state == MOVEEASTTASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL && myX < 2000 && ballet == 2) {
            myMove();
            return;
        }
        if (state == MOVEEASTTASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL && myX >= 2000 && ballet == 2) {
            ballet = 3;
            state = UTURNTASK;
            stepTurn(Parameters.Direction.RIGHT);
            return;
        }
        if (state == MOVEEASTTASK && detectFront().getObjectType() != IFrontSensorResult.Types.WALL && ballet > 0 && ballet < 4) {
            myMove();
            return;
        }
        if (state == MOVEEASTTASK && detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
            state = SINK;
            return;
        }
        if (state == UTURNTASK && !isSameDirection(myGetHeading(), Parameters.WEST)) {
            stepTurn(Parameters.Direction.RIGHT);
            return;
        }
        if (state == UTURNTASK && isSameDirection(myGetHeading(), Parameters.WEST)) {
            state = UTURNAGAINTASK;
            stepTurn(Parameters.Direction.RIGHT);
            return;
        }
        if (state == UTURNAGAINTASK && !isSameDirection(myGetHeading(), Parameters.EAST)) {
            stepTurn(Parameters.Direction.RIGHT);
            return;
        }
        if (state == UTURNAGAINTASK && isSameDirection(myGetHeading(), Parameters.EAST)) {
            state = MOVEEASTTASK;
            myMove();
            return;
        }
        if (state == SINK) {
            myMove();
            return;
        }
        if (true) {
            return;
        }
    }

    private void myMove() {
        isMoving = true;
        move();
    }

    private double myGetHeading() {
        double result = getHeading();
        while (result < 0) result += 2 * Math.PI;
        while (result > 2 * Math.PI) result -= 2 * Math.PI;
        return result;
    }

    private boolean isSameDirection(double dir1, double dir2) {
        return Math.abs(dir1 - dir2) < ANGLEPRECISION;
    }

    private void goToMeetPoint() {
        // A implémenter plus tard
    }
}
