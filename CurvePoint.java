package org.firstinspires.ftc.teamcode;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double slowDownTurnRadians;
    public double getSlowDownTurnAmount;

    /**
     *
     * @param x
     * @param y
     * @param moveSpeed
     * @param turnSpeed
     * @param followDistance
     * @param slowDownTurnRadians
     * @param getSlowDownTurnAmount
     */
    public CurvePoint(double x,double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians, double getSlowDownTurnAmount){
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.getSlowDownTurnAmount = getSlowDownTurnAmount;
    }

    /**
     *
     * @param curvePoint
     */
    public CurvePoint(CurvePoint curvePoint){
        this.x = curvePoint.x;
        this.y = curvePoint.y;
        this.moveSpeed = curvePoint.moveSpeed;
        this.turnSpeed = curvePoint.turnSpeed;
        this.followDistance = curvePoint.followDistance;
        this.slowDownTurnRadians = curvePoint.slowDownTurnRadians;
        this.getSlowDownTurnAmount = curvePoint.getSlowDownTurnAmount;
    }
    public Point toPoint(){
        return new Point(x,y);
    }

    /**
     *
     * @param point
     */
    public void setPoint(Point point) {
        this.x = point.x;
        this.y = point.y;
    }
}
