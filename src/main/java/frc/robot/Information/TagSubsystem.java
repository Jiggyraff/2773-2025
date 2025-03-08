package frc.robot.Information;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveSubsystems.DriveSubsystem;

public class TagSubsystem extends SubsystemBase {
    //Subsystems
    OdometrySubsystem odomSub;

    // PORT
    private final int PORT = 5800;
    DatagramChannel channel;

    // ALLOCATE BYTES FOR RECEIVING
    ByteBuffer buffer = ByteBuffer.allocate(200);

    // RECEIVE PACKETS (TRUE WHEN SOCKET IS INIT)
    private String lastInput;
    
    //Subsystem state variables
    private Boolean isEnabled = true;     //Deactivates everything
    private Boolean syncTags = false;       //Stops feeding to nav
    private Boolean cautiousMode = false;  //Checks if the data *could* be reliable based off parameters
    private Boolean seesTag = false;
    private double alphaTolerance = 0.1;
    private double XCom;

    private LaserSubsystem laserSub;

    //Tag Data Stuff
    TagData lastTag;
    double[][] aprilTagPositions = 
        //   ID    X       Y       Z    Zrot Yrot   >>>>   explained below          
           {//{1, 1,  2,  43, 0, 0},
            {1, 657.37,  25.80,  58.50, 126, 0},
            {2, 657.37,  291.20, 58.50, 234, 0},
            {3, 455.15,  317.15, 51.25, 270, 0},
            {4, 365.20,  241.64, 73.54, 0,   30},
            {5, 365.20,  75.39,  73.54, 0,   30},
            {6, 530.49,  130.17, 12.13, 300, 0},
            {7, 546.87,  158.50, 12.13, 0,   0},
            {8, 530.49,  186.83, 12.13, 60,  0},
            {9, 497.77,  186.83, 12.13, 120, 0},
            {10, 481.39, 158.50, 12.13, 180, 0},
            {11, 497.77, 130.17, 12.13, 240, 0},
            {12, 33.51,  25.80,  58.50, 54,  0},
            {13, 33.51,  291.20, 58.50, 306, 0},
            {14, 325.68, 241.64, 73.54, 180, 30},
            {15, 325.68, 75.39,  73.54, 180, 30},
            {16, 235.73, -0.15,  51.25, 90,  0},
            {17, 160.39, 130.17, 12.13, 240, 0},
            {18, 144.00, 158.50, 12.13, 180, 0},
            {19, 160.39, 186.83, 12.13, 120, 0},
            {20, 193.10, 186.83, 12.13, 60,  0},
            {21, 209.49, 158.50, 12.13, 0,   0},
            {22, 193.10, 130.17, 12.13, 300, 0}};

    double[][] currentTagPolars = new double[22][2];

        //https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf
        //+X is distance along length of field from BLUE reef side
        //+Y is distance along short side of field from RED barge/processor
        //+Z is up from ground
        //Zrot is rotation around Z axis, with 0 being facing towards RED reef
        //Yrot is rotation around Y axis, with 0 being facing forward. Weird values tho, 5 and 15 should not be the same


    public static class TagData{
        public int aprilTagID;
        public double x; //How far left or right, with to the right (from the tag's POV) being negative
        public double y; // How high or low the april tag is from the cam
        public double z; // How far away, with negative being forward, in front of the tag. It should never be +
        public double alpha; //Angle from tag to robot. What direction? Don't ask. (hint: idk)
    }


    public TagSubsystem(OdometrySubsystem odomSub, LaserSubsystem laserSub) {
        this.odomSub = odomSub;
        try {
            InetSocketAddress address = new InetSocketAddress(PORT);
            this.channel = DatagramChannel.open().bind(address);
            this.channel.configureBlocking(false);
            //this.isEnabled = true;
            } catch (IOException e) {
            e.printStackTrace();
        }
        Shuffleboard.getTab("April Tag Data").addDoubleArray("ID", () -> {return data != null ? new double[] {data.aprilTagID} : new double[]{};});
        Shuffleboard.getTab("April Tag Data").addDoubleArray("X", () -> {return data != null ? new double[] {data.x} : new double[]{};});
        Shuffleboard.getTab("April Tag Data").addDoubleArray("Y", () -> {return data != null ? new double[] {data.y} : new double[]{};});
        Shuffleboard.getTab("April Tag Data").addDoubleArray("Z", () -> {return data != null ? new double[] {data.z} : new double[]{};});
        Shuffleboard.getTab("April Tag Data").addDoubleArray("Angle", () -> {return data != null ? new double[] {data.alpha} : new double[]{};});
        Shuffleboard.getTab("Odometry").addDoubleArray("Adjusted Distance", () -> {return data != null ? new double[] {laserDistance} : new double[]{};});
        Shuffleboard.getTab("April Tag Data").addBoolean("Sees Tag", () -> {return seesTag;});
        this.laserSub = laserSub;
    }

    @Override
    public void periodic() {
        // System.out.println(seesTag);
        if (isEnabled) {
            receivePacket();
        }
        graphTagPolars();
    }
        
    TagData data;
    int deadTimer = 0;
    private void receivePacket() {
        try {
            // System.out.println("Working");
            if (channel.receive(buffer) != null) {
                buffer.flip();
                String rawText = new String(buffer.array(), buffer.arrayOffset(),
                        buffer.remaining());

                this.lastInput = rawText;
                data = parseTagData(rawText);
                // System.out.println(data);
                if (data != null) {
                    lastTag = data;
                    System.out.println("Sees Tag");
                    seesTag = true;
                    // System.out.println("Tag: " + data.aprilTagID + " " + data.x + " " + data.y + " " + data.z);
                    deadTimer = 0;
                }
                buffer.clear();

            } else {
                deadTimer++;
                buffer.clear();
            } 
            // System.out.println(String.valueOf(channel.receive(buffer)));
            if (deadTimer == 10) {
                // System.out.println("No tag in sight");
                seesTag = false;
            }
        } catch (Throwable e) {
            e.printStackTrace();
        }
    }

    public TagData getLastTagData() {
        return lastTag;
    }

    public double laserDistance = 0.0;
    private void updateOdometry(TagData data) {
        if (-alphaTolerance < data.alpha && data.alpha < alphaTolerance && laserDistance != -1) {
            double angleFromHorizontal = aprilTagPositions[data.aprilTagID-1][4]*Math.PI/180;
            double x = laserDistance*Math.cos(angleFromHorizontal) + aprilTagPositions[data.aprilTagID-1][1] * 0.0254;
            double y = laserDistance*Math.sin(angleFromHorizontal) + aprilTagPositions[data.aprilTagID-1][2] * 0.0254;
            double radians = aprilTagPositions[data.aprilTagID-1][4]*Math.PI/180 - Math.PI - data.alpha;
            odomSub.setPose(x, y, radians);
        }
    }

    // private void distanceUpdateOdometry() {
    //     if (data != null) {
    //     double angleBetweenRobotAndTag = laserSub.getRelativeMotorPosition() - odomSub.getGyroAngle() + Math.PI;
    //     double dx = Math.cos(angleBetweenRobotAndTag) * data.z;
    //     double dy = Math.sin(angleBetweenRobotAndTag) * data.z;
    //     dx += aprilTagPositions[data.aprilTagID][1];
    //     dy += aprilTagPositions[data.aprilTagID][2];
    //     odomSub.setPose(dx, dy, odomSub.getGyroAngle());
    //     }
    // }

    public TagData parseTagData(String s) {
        /*TAG: 4; 0.92... 123 123 123 123 123 123 123 123;123 123 123; 123 */
        // System.out.println(s);
        String[] tokens = s.split("; ");  
        String[] ids = tokens[0].split(": ");
        if (!ids[0].equals("TAG_FOUND") || tokens.length < 4) {
            return null;
        }
        // System.out.println(";" + tokens[2].split(" ")[0] + ";");
        XCom = Double.parseDouble(tokens[1].split(" ")[6]);
        String TagMatrix = tokens[1];

        String apriltag = ids[1];
        double[] tagInfo = aprilTagPositions[Integer.parseInt(apriltag)-1];

        double yRot = (-tagInfo[4] + 0) / 180.0 * Math.PI;
        Matrix<N4,N4> tagMatrix = Matrix.eye(Nat.N4());
        tagMatrix.set(0,0, Math.sin(yRot));
        tagMatrix.set(0,2, Math.cos(yRot));
        tagMatrix.set(2,0, Math.cos(yRot));
        tagMatrix.set(2,2, -Math.sin(yRot));
        tagMatrix.set(0,3, -tagInfo[1]*0.0254);
        tagMatrix.set(1,3, -tagInfo[3]*0.0254);
        tagMatrix.set(2,3, -tagInfo[2]*0.0254);



        String Group1 = tokens[2];
        String[] Num = Group1.split(" ");
        double XNum = Double.parseDouble(Num[0]);
        double YNum = Double.parseDouble(Num[1]);
        double ZNum = Double.parseDouble(Num[2]);

        String[] MatrixNum = TagMatrix.split(" ");
        Matrix<N4,N4> rotationMatrix = Matrix.eye(Nat.N4());
        for (int i = 0; i < 3; i++ ) {
            for (int j = 0; j < 3; j++) {
                rotationMatrix.set(i, j, Double.parseDouble(MatrixNum[i*3 + j]));
            }
        }
        Matrix<N4,N4> trMatrix = Matrix.eye(Nat.N4());
        trMatrix.set(0, 3, XNum);
        trMatrix.set(1, 3, YNum);
        trMatrix.set(2, 3, ZNum);
        // System.out.println(rotationMatrix);
    
        var tagView = rotationMatrix.times(trMatrix).inv();  //A^(-1)
        rotationMatrix = /*tagMatrix.times(*/tagView/*)*/;
        
        double cosAlpha = rotationMatrix.get(0,0);
        // System.out.println(tagView);
        
        TagData data = new TagData();
        data.x = rotationMatrix.get(0, 3);
        data.y = rotationMatrix.get(1, 3);
        data.z = rotationMatrix.get(2, 3);
        data.alpha = Math.acos(cosAlpha);
        data.aprilTagID = Integer.parseInt(apriltag);

        if (syncTags == true) {
            updateOdometry(data);
        }
        // System.out.println("Tag: "+rotationMatrix.get(0, 3) + " : "  + rotationMatrix.get(1, 3) + " : "  + rotationMatrix.get(2, 3) + " : " + data.alpha);
        return data;
    }



    public void enable() {
        if (syncTags) {
            syncTags = false;
            System.out.println("Tags no longer feeding navigation");
        } else {
            syncTags = true;
            System.out.println("Tags feeding navigation");
        }
    }

    public void cautiousMode() {
        if (cautiousMode) {
            cautiousMode = false;
            System.out.println("Cautious mode off");
        } else {
            cautiousMode = true;
            System.out.println("Cautious mode on");
        }
    }

    // double[] laserSum = new double[10];
    // int laserN = 0;
    // public void setDistance(double d) {
    //     if (laserN != 10) {
    //         laserSum[laserN] = d;
    //         laserN++;
    //     } else {
    //         double laserAverage = 0.0;
    //         double laserAverage2 = 0.0;
    //         for (int i = 0; i < laserSum.length;i++) {
    //             laserAverage += laserSum[i];
    //         }
    //         laserAverage /= 10;
    //         int n = 0;
    //         for (int i = 0; i < laserSum.length; i++) {
    //             if (laserSum[i] > laserAverage) {
    //                 laserAverage2 += laserSum[i];
    //                 n++;
    //             }
    //         }
    //         laserAverage2 /= n;
    //         laserDistance = laserAverage2/1000;
    //     }
    // }

    public Boolean getSeestag() {
        // System.out.println("Sees Tag: " + seesTag);
        return seesTag;
    }

    public double getXCom() {
        return XCom;
    }

    public void reset() {
        seesTag = false;
    }

    //Assumes robot's position is in meters and is field centric
    private void graphTagPolars() {
        for (int i = 0; i < currentTagPolars.length; i++) {
            double dx = aprilTagPositions[i][1] * 0.0254 - odomSub.getY();
            double dy = aprilTagPositions[i][2] * 0.0254 - odomSub.getX();
            currentTagPolars[i][0] = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
            //From robot to tag
            currentTagPolars[i][1] = Math.atan2(dy, dx);
        }
    }

    public double[] getNearestTag() {
        double[] nearestTag = {999, 0};
        for (int i = 0; i < currentTagPolars.length; i++) {
            if (currentTagPolars[i][0] < nearestTag[0]) {
                nearestTag = currentTagPolars[i];
            }
        }
        return nearestTag;
    }
}
