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
import frc.robot.SwerveSubsystems.DriveSubsystem;

public class TagSubsystem extends SubsystemBase {

    //Subsystens
    NavigationSubsystem navSub;

    // PORT
    private final int PORT = 15200;
    DatagramChannel channel;

    // ALLOCATE BYTES FOR RECEIVING
    ByteBuffer buffer = ByteBuffer.allocate(200);

    // RECEIVE PACKETS (TRUE WHEN SOCKET IS INIT)
    private boolean isEnabled = false;
    private String lastInput;


    //Tag Data Stuff
    TagData lastTag;
    Boolean syncTags;
    double[][] aprilTagPositions = 
        //   ID    X       Y       Z    Zrot Yrot   >>>>   explained below          
           {{1, 657.37,  25.80,  58.50, 126, 0},
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

        //https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf
        //+X is distance along length of field from BLUE reef side
        //+Y is distance along short side of field from RED barge/processor
        //+Z is up from ground
        //Zrot is rotation around Z axis, with 0 being facing towards RED reef
        //Yrot is rotation around Y axis, with 0 being facing forward. Weird values tho, 5 and 15 should not be the same

    public static TagData[] lastAprilTagData = new TagData[30];

    public TagData getAprilTag(int id) {
        return lastAprilTagData[id];
    }


    public static class TagData{
        public int aprilTagID;
        public double x; //How far left or right, with to the right (from the tag's POV) being negative
        public double y; // How high or low the april tag is from the cam
        public double z; // How far away, with negative being forward, in front of the tag. It should never be +
        public double alpha; //Angle from tag to robot. What direction? Don't ask. (hint: idk)
    }


    public TagSubsystem(NavigationSubsystem navSub) {
        syncTags = false;
        this.navSub = navSub;
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

    private void updateOdometry(TagData data) {
        double angleFromHorizontal = aprilTagPositions[data.aprilTagID-1][4]*Math.PI/180;
        double x = -data.z*Math.cos(angleFromHorizontal) + aprilTagPositions[data.aprilTagID-1][1];
        double y =  data.x*Math.cos(angleFromHorizontal) + aprilTagPositions[data.aprilTagID-1][2];
        navSub.setPosition(x, y);
    }

    public TagData parseTagData(String s) {
        /*TAG: 4; 0.92... 123 123 123 123 123 123 123 123; 123 123 123; 123 */
        // System.out.println(s);
        String[] tokens = s.split(";");  
        String[] ids = tokens[0].split(": ");
        if (!ids[0].equals("TAG") || tokens.length < 4) {
            return null;
        }
        String TagMatrix = tokens[1];

        String apriltag = ids[1];
        String Group1 = tokens[2];
        String[] Num = Group1.split(" ");
        double XNum = Double.parseDouble(Num[0]);
        double YNum = Double.parseDouble(Num[1]);
        double ZNum = Double.parseDouble(Num[2]);

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

        if (syncTags) {
            updateOdometry(data);
        }
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
