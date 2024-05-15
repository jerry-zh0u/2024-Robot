package frc.robot.Vision;

public class PhotonVisionSmoother {
    private double alpha = 0.2; // Smoothing factor (should be between 0 and 1, default is 0.2)
    private double smoothedX; 
    private double smoothedY;
    private boolean isFirstUpdate = true;

    public void updateCoordinates(double currentX, double currentY) {
        if (isFirstUpdate){
            smoothedX = currentX;
            smoothedY = currentY;
            isFirstUpdate = false;
        }
        else{
            smoothedX = alpha * currentX + (1 - alpha) * smoothedX;
            smoothedY = alpha * currentY + (1 - alpha) * smoothedY;
        }
    }
    public double getSmoothedX(){
        return smoothedX;
    }
    public double getSmoothedY(){
        return smoothedY;
    }    
}
