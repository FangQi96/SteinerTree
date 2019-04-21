package steiner.model;

import java.io.*;
import java.util.ArrayList;

public class pressureLocal implements Serializable{
    private ArrayList<Double> pressure = new ArrayList<>();
    private int iterationTurn;
    public pressureLocal(int turn){
        iterationTurn = turn;
    }
    public void setPressureLocal(ArrayList<Double> pressure,int turn){
        this.pressure.addAll(pressure);
        this.iterationTurn = turn;
    }

    public ArrayList<Double> getPressure() {
        return pressure;
    }

    public int getIterationTurn(){      //Used for un synchronized iteration
        return iterationTurn;
    }

    public byte[] PressureLocalToByte(Vertex owner){
        byte[] bytes = null;
        try{
            ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
            ObjectOutputStream objectOutputStream = new ObjectOutputStream(byteArrayOutputStream);

            objectOutputStream.writeByte(1);    //type 1 means it's a message contains pressureLocal
            objectOutputStream.writeInt(owner.getName());
            objectOutputStream.writeObject(this);

            bytes = byteArrayOutputStream.toByteArray();

            byteArrayOutputStream.close();
            objectOutputStream.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return bytes;
    }

    public static pressureLocal ByteToPressureLocal(byte[] bytes){
        Object object = null;
        try{
            ByteArrayInputStream byteArrayInputStream = new ByteArrayInputStream(bytes);
            ObjectInputStream objectInputStream = new ObjectInputStream(byteArrayInputStream);


            objectInputStream.readByte();    //dump the message type
            objectInputStream.readInt();       //dump out the vertex name

            object = objectInputStream.readObject();
            byteArrayInputStream.close();
            objectInputStream.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return (pressureLocal)object;
    }
}
