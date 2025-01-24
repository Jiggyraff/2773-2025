package frc.robot.Information;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

import edu.wpi.first.math.Matrix;
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

public class TagSubsystem extends SubsystemBase {

    // PORT
    private final int PORT = 15200;
    DatagramChannel channel;

    // ALLOCATE BYTES FOR RECEIVING
    ByteBuffer buffer = ByteBuffer.allocate(200);

    // RECEIVE PACKETS (TRUE WHEN SOCKET IS INIT)
    private boolean isEnabled = false;
    private String lastInput;
    // private TagHandler tagHandler;

    TagData lastTag;

    public static double[][] aprilTagCoordinate = {
            { 652.73, 196.17, 57.13, 180 },//{ 593.68, 9.68, 53.38, 120 },
            { 637.21, 34.79, 53.38, 120 },
            { 652.73, 196.17, 57.13, 180 },
            { 652.73, 218.42, 57.13, 180 },
            { 578.77, 323.00, 53.38, 270 },
            { 72.5, 323.00, 53.38, 270 },
            { -1.50, 218.42, 57.13, 0 },
            { -1.50, 196.17, 57.13, 0 },
            { 14.02, 34.79, 53.3, 60 },
            { 57.54, 9.68, 53.3, 60 },
            { 468.69, 146.19, 52.00, 300 },
            { 468.69, 177.10, 52.0, 60 },
            { 441.74, 161.62, 52.00, 180 },
            { 209.48, 161.62, 52.00, 0 },
            { 182.73, 177.10, 52.00, 120 },
            { 182.73, 146.19, 52.00, 240 } };
    public static TagData[] lastAprilTagData = new TagData[30];

    public TagData getAprilTag(int id) {
        return lastAprilTagData[id];
    }


    public static class TagData{
        public int aprilTagID;
        public double x; // How far right or left (I think)
        public double y; // How high or low the april tag is
        public double z; // How far away (I think)
        public double alpha; //Angle from tag to robot
    }


    public TagSubsystem() {
        try {
            InetSocketAddress address = new InetSocketAddress(PORT);
            this.channel = DatagramChannel.open().bind(address);
            this.channel.configureBlocking(false);
            this.isEnabled = true;
            } catch (IOException e) {
            e.printStackTrace();
        }
        Shuffleboard.getTab("April Tag Data").addDoubleArray("ID", () -> {
            return data != null ? new double[] {data.aprilTagID} : new double[]{};
        });
        Shuffleboard.getTab("April Tag Data").addDoubleArray("X", () -> {
            return data != null ? new double[] {data.x} : new double[]{};
        });
        Shuffleboard.getTab("April Tag Data").addDoubleArray("Y", () -> {
            return data != null ? new double[] {data.y} : new double[]{};
        });
        Shuffleboard.getTab("April Tag Data").addDoubleArray("Z", () -> {
            return data != null ? new double[] {data.z} : new double[]{};
        });
        Shuffleboard.getTab("April Tag Data").addDoubleArray("Angle", () -> {
            return data != null ? new double[] {data.alpha} : new double[]{};
        });
    }

    @Override
    public void periodic() {

        if (isEnabled) {
            receivePacket();
        }
    }

    TagData data;
    private void receivePacket() {
        try {
            if (channel.receive(buffer) != null) {
                buffer.flip();
                String rawText = new String(buffer.array(), buffer.arrayOffset(),
                        buffer.remaining());

                this.lastInput = rawText;
                data = parseTagData(rawText);
                if (data != null) {
                    // updateOdometry(data);
                    updateTags(data);
                    lastTag = data;
                    // System.out.println("Tag: " + data.aprilTagID + " " + data.x + " " + data.y + " " + data.z);
                }
                buffer.clear();

            }
        } catch (Throwable e) {
            e.printStackTrace();
        }
    }

    public String getLastPacket() {
        return lastInput;
    }

//     private void updateOdometry(TagData data) {
// //  System.out.println(" Robot: " + data.aprilTagID + " , " + data.x + ", " + data.z + " " + data.alpha);
//         double distance = Math.sqrt(data.x * data.x + data.z * data.z);
//         double angle = -data.alpha + aprilTagCoordinate[data.aprilTagID][3];
//         double processedX = Math.cos(angle) * distance;
//         double processedY = Math.sin(angle) * distance;
//         double robotX = 0.0254 * aprilTagCoordinate[data.aprilTagID-1][0] + processedX;
//         double robotY = 0.0254 * aprilTagCoordinate[data.aprilTagID-1][1] + processedY;
//         odomSub.setPosition(robotX, robotY);
//         // System.out.print(": " + distance + " , " + angle + " "+data.alpha);



//         // Transform2d trans = new Transform2d(robotX, robotY, new Rotation2d());
//         // odomSub.pose.plus(trans);
//     }

    public TagData parseTagData(String s) {
        /*TAG: 4; 0.92... 123 123 123 123 123 123 123 123; 123 123 123; 123 */
        System.out.println(s);
        String[] tokens = s.split(";");  
        String[] ids = tokens[0].split(": ");
        if (!ids[0].equals("TAG") || tokens.length < 4) {
            return null;
        }

        String apriltag = ids[1];

        String Group1 = tokens[2];

        String[] Num = Group1.split(" ");

        double XNum = Double.parseDouble(Num[0]);

        double YNum = Double.parseDouble(Num[1]);

        double ZNum = Double.parseDouble(Num[2]);

        String TagMatrix = tokens[1];


        String[] MatrixNum = TagMatrix.split(" ");
        Matrix<N4,N4> rotationMatrix = new Matrix<N4, N4>(N4.instance, N4.instance);
        for (int i = 0; i < 3; i++ ) {
            for (int j = 0; j < 3; j++) {
                rotationMatrix.set(i, j, Double.parseDouble(MatrixNum[i*3 + j]));
            }
        }
        rotationMatrix.set(0, 3, XNum);
        rotationMatrix.set(1, 3, YNum);
        rotationMatrix.set(2, 3, ZNum);
        rotationMatrix.set(3, 3, 1);
    
        rotationMatrix = rotationMatrix.inv();
        
        
        double sinAlpha = rotationMatrix.get(0,0);
        double minusCosAlpha = rotationMatrix.get(0, 2);
        
        TagData data = new TagData();
        data.x = rotationMatrix.get(0, 3);
        data.y = rotationMatrix.get(1, 3);
        data.z = rotationMatrix.get(2, 3);
        data.alpha = Math.atan2(minusCosAlpha,sinAlpha);
        data.aprilTagID = Integer.parseInt(apriltag);
        // System.out.println("Tag: "+rotationMatrix.get(0, 3) + " : "  + rotationMatrix.get(1, 3) + " : "  + rotationMatrix.get(2, 3) + " : " + data.alpha);
        return data;
    }

    public void updateTags(TagData dataToUpdate)
    {
        lastAprilTagData[dataToUpdate.aprilTagID] = dataToUpdate;
    }

    public int getSpeakerTagID()
    {
        if(DriverStation.getAlliance().equals(Alliance.Red)) {
            return Constants.speakerTagIDRed;
        }
        else return Constants.speakerTagIDBlue;
    }
    public double getLastSpeakerDistance()
    {
        // TagData tempData = getAprilTag(getSpeakerTagID());
        return 4.0;//Math.sqrt(tempData.x * tempData.x + tempData.z * tempData.z);
    }
    public void printAlliance()
    {
        System.out.println(DriverStation.getAlliance());
    }
}
