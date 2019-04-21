package steiner.model;

import java.io.ByteArrayInputStream;
import java.io.ObjectInputStream;
import java.net.*;

public class DistributedServer extends Thread {
    public int getPORT() {
        return PORT;
    }

    private int PORT = 23333;
    private Vertex vertex;
    private static final int DATALENGTH = 4096;
    byte[] inBuffer = new byte[DATALENGTH];
    private DatagramPacket inPacket = new DatagramPacket(inBuffer, inBuffer.length);
    private DatagramPacket outPacket;

    public DistributedServer(Vertex vertex){
        this.vertex = vertex;
        this.PORT = vertex.getName() + PORT;
        vertex.setServerThread(new Thread(this));
        vertex.getServerThread().start();
    }

    public void run() {
        try(DatagramSocket socket = new DatagramSocket(PORT)){
            while(!this.isInterrupted()){
                socket.receive(inPacket);
                System.out.println(inBuffer == inPacket.getData());
                if(inPacket.getData()[0]==0) {  //It's an ask, send self pressure
                    byte[] sendData = vertex.getPressureLocal().PressureLocalToByte(vertex);  //tweak here, sendData should be pressureLocal
                    outPacket = new DatagramPacket(sendData, sendData.length, inPacket.getAddress(),inPacket.getPort()-10000);
                    socket.send(outPacket);
                }else{  //It's a reply, update neighbor's pressure
                    byte[] receiveData = inPacket.getData();
                    ByteArrayInputStream byteArrayInputStream = new ByteArrayInputStream(receiveData);
                    ObjectInputStream objectInputStream = new ObjectInputStream(byteArrayInputStream);
                    objectInputStream.readByte(); //dump the message type

                    int neighborName = objectInputStream.readInt();
                    Vertex neighbor = vertex.getNeighborByName(neighborName);
                    pressureLocal pressure = pressureLocal.ByteToPressureLocal(receiveData);
                    vertex.getNeighborsPressureLocal().put(neighbor,pressure);
                }
            }
        }catch (Exception e){
            e.printStackTrace();
        }
    }

    public void ask(int dest_port){
        try{
            DatagramSocket socket = new DatagramSocket(PORT+10000); //Choose a different port from the listening one
            DatagramPacket ask = new DatagramPacket(new byte[0],0,InetAddress.getByName("127.0.0.1"),dest_port);
            byte[] bytes = {0};     //type 0 means it's a pull request
            ask.setData(bytes);
            socket.send(ask);
            socket.close(); //Close the socket once the request is sent
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
