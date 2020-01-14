/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot;
/**
 *
 * @author vshah-21
 */
import java.io.IOException;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
/**
 * A simple TCP server. When a client connects, it sends the client the current
 * datetime, then closes the connection. This is arguably the simplest server
 * you can write. Beware though that a client has to be completely served its
 * date before the server will be able to handle another client.
 */
public class CSVServer {
     
    public static String addComas(String[] s){
        String news = null;
        for(int i =0; i < s.length;i++){
            news+=s[i];
            if(i<s.length-1){
                news+=",";
            }
        }
        news +=",";
        return news;
    }
    
    public void testings(ArrayList<String[]> ones)  throws IOException {
         
        try (ServerSocket listener = new ServerSocket(4000)) {
            
                try (Socket realServer = listener.accept()) {
                    PrintWriter o = new PrintWriter(realServer.getOutputStream(), true);
                    for(int i =0; i < ones.size();i++){
                        o.println(addComas(ones.get(i)));
                    }
                    realServer.close();
                    
                }
            
        }
    }

}
