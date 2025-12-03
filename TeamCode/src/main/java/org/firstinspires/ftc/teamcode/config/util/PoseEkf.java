package org.firstinspires.ftc.teamcode.config.util;

import java.util.ArrayDeque;

public class PoseEkf {

    // -------------------------
    // State vector: [x, y, theta]
    // -------------------------
    private double x;      // current filtered EKF pose x (in)
    private double y;      // current filtered EKF pose y (in)
    private double theta;  // current filtered EKF heading (rad)

    // -------------------------
    // Covariance matrix P: 3x3
    // -------------------------
    private double[][] P = new double[3][3];

    // -------------------------
    // Process & measurement noise
    // -------------------------
    private final double qPos, qTheta;   // process noise
    private final double rPos;            // measurement noise for x/y only

    // -------------------------
    // State history buffer for latency compensation
    // -------------------------
    private static class TimedState {
        double x, y, theta;
        double[][] P;
        double timestamp;
        double vForward;
        double vLateral;
        double omega;
        double odomX, odomY, odomTheta;
    }
    private final int BUFFER_SIZE = 60;
    private final ArrayDeque<TimedState> stateBuffer = new ArrayDeque<>();

    // -------------------------
    // Vision / odometry storage
    // -------------------------
    private double visionX = Double.NaN, visionY = Double.NaN;
    private double odomX = Double.NaN, odomY = Double.NaN, odomTheta = Double.NaN;
    private double predictedX = Double.NaN, predictedY = Double.NaN, predictedTheta = Double.NaN;

    // -------------------------
    // Constructor
    // -------------------------
    public PoseEkf(double initX, double initY, double initTheta,
                   double qPos, double qTheta,
                   double rPos) {
        this.x = initX; this.y = initY; this.theta = initTheta;
        this.qPos = qPos; this.qTheta = qTheta; this.rPos = rPos;

        // small initial covariance
        P[0][0] = 0.001; P[1][1] = 0.001; P[2][2] = 0.001;
    }

    // ============================================================
    // Prediction step
    // ============================================================
    public void predict(double vForward, double vLateral, double omega, double dt, double time,
                        double odomXraw, double odomYraw, double odomThetaRaw) {

        // Store raw odometry
        odomX = odomXraw; odomY = odomYraw; odomTheta = odomThetaRaw;

        // -------------------------
        // Apply holonomic motion model
        // -------------------------
        double dx = vForward * Math.cos(theta) * dt - vLateral * Math.sin(theta) * dt;
        double dy = vForward * Math.sin(theta) * dt + vLateral * Math.cos(theta) * dt;
        double dtheta = omega * dt;

        predictedX = x + dx;
        predictedY = y + dy;
        predictedTheta = wrap(theta + dtheta);

        x = predictedX;
        y = predictedY;
        theta = predictedTheta;

        // -------------------------
        // Jacobian & covariance update
        // -------------------------
        double[][] F = {
                {1, 0, -vForward * Math.sin(theta) * dt - vLateral * Math.cos(theta) * dt},
                {0, 1,  vForward * Math.cos(theta) * dt - vLateral * Math.sin(theta) * dt},
                {0, 0, 1}
        };
        double[][] Q = {{qPos,0,0},{0,qPos,0},{0,0,qTheta}};
        P = matAdd(matMul(F, matMul(P, transpose(F))), Q);

        // -------------------------
        // Save state to buffer
        // -------------------------
        TimedState ts = new TimedState();
        ts.x = x; ts.y = y; ts.theta = theta;
        ts.P = deepCopy(P); ts.timestamp = time;
        ts.vForward = vForward; ts.vLateral = vLateral; ts.omega = omega;
        ts.odomX = odomX; ts.odomY = odomY; ts.odomTheta = odomTheta;

        stateBuffer.addLast(ts);
        if (stateBuffer.size() > BUFFER_SIZE) stateBuffer.removeFirst();
    }

    // ============================================================
    // Update step (vision only x/y)
    // ============================================================
    public void updateWithVision(double zx, double zy, double timestamp) {

        TimedState closest = findClosestState(timestamp);
        if (closest == null) return;

        // Reset EKF to old state
        x = closest.x; y = closest.y; theta = closest.theta; // keep theta from odom
        P = deepCopy(closest.P);

        // Measurement update only for x/y
        double[][] H = {{1,0,0},{0,1,0}}; // 2x3
        double[][] R = {{rPos,0},{0,rPos}}; // 2x2
        double[] z = {zx, zy};
        double[] h = {x, y};
        double[] yErr = vecSub2(z, h);

        double[][] S = matAdd(matMul(H, matMul(P, transpose(H))), R);
        double[][] K = matMul(P, matMul(transpose(H), inverse2x2(S))); // 3x2 * 2x2 = 3x2
        double[] adjustment = matVecMul2x3(K, yErr);

        x += adjustment[0];
        y += adjustment[1];
        // theta unchanged
        P = matMul(matSub(identity3(), matMul(K, H)), P);

        // Store vision pose
        visionX = x; visionY = y;

        // Replay buffered predictions forward
        replayAfter(timestamp);
    }

    // ============================================================
    // Latency replay
    // ============================================================
    private void replayAfter(double timestamp) {
        for (TimedState ts : stateBuffer) {
            if (ts.timestamp > timestamp) {
                predict(ts.vForward, ts.vLateral, ts.omega,
                        0.0, ts.timestamp, ts.odomX, ts.odomY, ts.odomTheta);
            }
        }
    }

    private TimedState findClosestState(double timestamp) {
        TimedState best = null;
        double bestDiff = Double.MAX_VALUE;
        for (TimedState ts : stateBuffer) {
            double diff = Math.abs(ts.timestamp - timestamp);
            if (diff < bestDiff) { bestDiff = diff; best = ts; }
        }
        return best;
    }

    // ============================================================
    // Getter methods
    // ============================================================
    public double[] getPredictedPose() { return new double[]{predictedX, predictedY, predictedTheta}; }
    public double[] getFilteredPose() { return new double[]{x, y, theta}; }
    public double[] getVisionPose() { return new double[]{visionX, visionY, theta}; } // theta from odom
    public double[] getOdometryPose() { return new double[]{odomX, odomY, odomTheta}; }

    // ============================================================
    // Helper math functions
    // ============================================================
    private double wrap(double a) { while (a <= -Math.PI) a += 2*Math.PI; while (a>Math.PI) a -= 2*Math.PI; return a; }
    private double[][] matMul(double[][] A, double[][] B){ double[][] C=new double[3][3]; for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)C[i][j]+=A[i][k]*B[k][j];return C;}
    private double[][] matAdd(double[][] A,double[][] B){ double[][] C=new double[3][3]; for(int i=0;i<3;i++)for(int j=0;j<3;j++)C[i][j]=A[i][j]+B[i][j];return C;}
    private double[][] matSub(double[][] A,double[][] B){ double[][] C=new double[3][3]; for(int i=0;i<3;i++)for(int j=0;j<3;j++)C[i][j]=A[i][j]-B[i][j];return C;}
    private double[] matVecMul(double[][] A,double[] x){return new double[]{A[0][0]*x[0]+A[0][1]*x[1]+A[0][2]*x[2],A[1][0]*x[0]+A[1][1]*x[1]+A[1][2]*x[2],A[2][0]*x[0]+A[2][1]*x[1]+A[2][2]*x[2]};}
    private double[][] transpose(double[][] A){double[][] T=new double[3][3];for(int i=0;i<3;i++)for(int j=0;j<3;j++)T[i][j]=A[j][i];return T;}
    private double[][] identity3(){return new double[][]{{1,0,0},{0,1,0},{0,0,1}};}
    private double[][] deepCopy(double[][] A){double[][] B=new double[3][3];for(int i=0;i<3;i++)System.arraycopy(A[i],0,B[i],0,3);return B;}
    private double[] vecSub2(double[] a,double[] b){return new double[]{a[0]-b[0],a[1]-b[1]};}

    // 2x2 matrix inverse
    private double[][] inverse2x2(double[][] m){
        double a=m[0][0],b=m[0][1],c=m[1][0],d=m[1][1];
        double det = a*d-b*c;
        double invDet = 1.0/det;
        return new double[][]{
                { d*invDet, -b*invDet },
                { -c*invDet, a*invDet }
        };
    }

    // multiply 3x2 by 2x1 -> 3x1
    private double[] matVecMul2x3(double[][] A, double[] x){
        return new double[]{
                A[0][0]*x[0] + A[0][1]*x[1],
                A[1][0]*x[0] + A[1][1]*x[1],
                A[2][0]*x[0] + A[2][1]*x[1]
        };
    }

}
