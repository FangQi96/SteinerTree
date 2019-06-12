package steiner.model;

import java.io.ByteArrayInputStream;
import java.io.ObjectInputStream;
import java.net.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

public class DistributedServer extends Thread {
    public int getPORT() {
        return PORT;
    }

    private int PORT = 23333;
    private Vertex vertex;
    private static final int DATALENGTH = 4096;
    private byte[] inBuffer = new byte[DATALENGTH];
    private DatagramPacket inPacket = new DatagramPacket(inBuffer, inBuffer.length);
    private DatagramPacket outPacket;
    private DistributedServer.Listener listener = null;
    private Set<Vertex> sourceSet = null;
    private int vertexNumber = 0;
    private DatagramSocket askSocket = null;


    public Double[][] getSlideWindow() {
        return slideWindow;
    }

    private Double[][] slideWindow = new Double[100][5];



    private double delta = 1.3;
    private double rho = 0.95;
    private double I_0 = 100;

    public DistributedServer(Vertex vertex) {
        this.vertex = vertex;
        this.PORT = vertex.getName() + PORT;
        new Thread(this).start();
    }

    public void run() {
        this.listen();
        try {
            askSocket = new DatagramSocket(PORT + 10000); //Choose a different port from the listening one
        } catch (SocketException e) {
            e.printStackTrace();
        }
        try {       //Wait for the vertex's initialization
            sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        if(this.vertex.getNeighborMap().size()!=0) {
            ArrayList<Double> vertexPressure = vertex.getPressureLocal().getPressure();
            for (int i = 0; i < 5; i++) {
                slideWindow[0][i] = vertexPressure.get(i);
            }
            iteration(1, 30);
        }
        else{
            this.listener.exit = true;
            try {
                this.listener.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        askSocket.close();
        //System.out.println(vertex.toString()+" "+listner.isInterrupted());
    }

    public void listen() {
        this.listener = this.new Listener();
        this.listener.start();
    }

    class Listener extends Thread{
        private volatile boolean exit = false;
        public void run() {
            try (DatagramSocket socket = new DatagramSocket(PORT)) {
                while (!exit) {
                    socket.receive(inPacket);
                    if (inPacket.getData()[0] == 0) {  //It's an ask, send self pressure
                        int iterationAsked = inPacket.getData()[1];
                        if(slideWindow[iterationAsked][0]!=null) {     //Already calculated
                            pressureLocal out = new pressureLocal(iterationAsked);
                            out.setPressureLocal(new ArrayList<>(Arrays.asList(slideWindow[iterationAsked])),iterationAsked);
                            byte[] sendData = out.PressureLocalToByte(vertex);  //tweak here, sendData should be pressureLocal
                            outPacket = new DatagramPacket(sendData, sendData.length, inPacket.getAddress(), inPacket.getPort() - 10000);
                            socket.send(outPacket);
                            //System.out.println(vertex.toString()+" received an ask and then sent back.");
                        }else{  //not calculated yet
                            ; //Do nothing, should return a not done yet message
                            System.out.println(vertex.toString()+" received an ask but pressure asked is not ready");
                        }
                    } else {  //It's a reply, update neighbor's pressure
                        byte[] receiveData = inPacket.getData();
                        ByteArrayInputStream byteArrayInputStream = new ByteArrayInputStream(receiveData);
                        ObjectInputStream objectInputStream = new ObjectInputStream(byteArrayInputStream);
                        objectInputStream.readByte(); //dump the message type

                        int neighborName = objectInputStream.readInt();
                        Vertex neighbor = vertex.getNeighborByName(neighborName);
                        pressureLocal pressure = pressureLocal.ByteToPressureLocal(receiveData);
                        vertex.getNeighborsPressureLocal().put(neighbor, pressure);
                        System.out.println(vertex.toString()+" updated Node "+neighborName+"'s pressureLocal");
                    }
                }
                System.out.println(vertex.toString()+" stops listening.");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void askNeighbors(byte iterationTime){
        for(Vertex neighbor:this.vertex.getNeighborMap().values()){
            this.ask(neighbor.getServer().getPORT(),iterationTime);
        }
    }

    public void ask(int dest_port,byte iterationTime) {
        try {
            DatagramPacket ask = new DatagramPacket(new byte[0], 0, InetAddress.getByName("127.0.0.1"), dest_port);
            byte[] bytes = {0,iterationTime};//type 0 means it's a pull request asks for [iterationTime] pressure
            byte[] time = String.valueOf(System.currentTimeMillis()).getBytes();//Time stamp
            byte[] result = Arrays.copyOf(bytes,bytes.length+time.length);
            System.arraycopy(time,0,result,bytes.length,time.length);
            ask.setData(bytes);
            askSocket.send(ask);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void iteration(int inner, int outer) {
        int neighborNum = vertex.getNeighborMap().size();
        for(int i=1;i<=outer;i++){
            while(this.vertex.getNeighborsPressureLocal().size()!=neighborNum){     //Haven't collect all pressureLocal
                //askNeighbors((byte)(i-1));
                for(Vertex neighbor:vertex.getNeighborMap().values()){
                    if(!vertex.getNeighborsPressureLocal().containsKey(neighbor)) {
                        ask(neighbor.getServer().getPORT(), (byte) (i - 1));
                        //System.out.println(vertex.toString()+" asks for turn "+(byte)(i-1)+" pressure");
                        try {
                            Thread.sleep(1);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }
            }

            for(int j=0;j<inner;j++){
                calculation(i);
            }

            this.vertex.getNeighborsPressureLocal().clear();    //clear up the neighbors map for next iteration
            //System.out.println(vertex.toString()+" has completed Turn "+i);
        }
        vertex.setCompleted(true);
    }

    public void calculation(int iterationTurn) {
        int count = 0;
        for (Vertex sink : sourceSet) {
            double sum_above = 0;
            double sum_below = 0.000001;
            for (ConcurrentHashMap.Entry<Vertex, pressureLocal> entry : vertex.getNeighborsPressureLocal().entrySet()) {
                Edge edgeLocal = this.vertex.getNeighborEdgeMap().get(entry.getKey());
                double flux = Math.abs(edgeLocal.getConductivity() * (vertex.getPressureLocal().getPressure().get(count) -
                        entry.getValue().getPressure().get(count)) / edgeLocal.getWeight());
                double new_conductivity = (Math.pow(flux, delta) / (I_0 * I_0 / vertexNumber * vertexNumber +
                        Math.pow(flux, delta))) + rho * edgeLocal.getConductivity();
                edgeLocal.setConductivity(new_conductivity);
                /********************************************************
                 *Don't cut, leave it to the graph(AKA global controller)*
                 ********************************************************/
                sum_below = sum_below + new_conductivity / edgeLocal.getWeight();
                sum_above = sum_above + new_conductivity * entry.getValue().getPressure().get(count) / edgeLocal.getWeight();

            }

            if (vertex.isSource()) {
                if (vertex.getName() == sink.getName())
                    sum_above = sum_above - (sourceSet.size() - 1) * I_0;
                else
                    sum_above = sum_above + I_0;
            } else
                ;
            vertex.getPressureLocal().getPressure().set(count, sum_above / sum_below);
            vertex.getPressureLocal().setIterationTurn(iterationTurn);
            vertex.getPressureLocal().getPressure().toArray(slideWindow[iterationTurn]);    //update self SlideWindow
            count++;
        }
    }



    public void setVertexNumber(int vertexNumber) {
        this.vertexNumber = vertexNumber;
    }

    public void setSourceSet(Set<Vertex> sourceSet) {
        this.sourceSet = sourceSet;
    }
}
