// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

/** Simple HTTP Responder for a robot */
public class RobotHttpResponder extends Thread {
    // maybe we will want to disable the server when robot is actively driving, to conserve the CPU cycles
    public boolean m_disabled = false;

    // please set these in robotPeriodic(), numbers only (no strings, otherwise questions about thread safety)
    public double m_battVoltage = Double.NaN;
    public int m_numMotorsConnected = 0;

    // copied from https://www.codejava.net/java-se/networking/java-socket-server-examples-tcp-ip
    public void run() {
        try (ServerSocket serverSocket = new ServerSocket(8080, 5, InetAddress.getByAddress(new byte[] {0, 0, 0, 0}))) {
            System.out.println("HTTP responder is listening on port " + m_port);
            while (true) {
                Socket client = serverSocket.accept();
                while (m_disabled)
                    sleep(1000); // when disabled, try to not serve web pages
                BufferedReader reader = new BufferedReader(new InputStreamReader(client.getInputStream()));
                String request = reader.readLine();
                System.out.println("Request: " + request);
                String response = null;
    
                // 1. did that client request the CSS?
                if (request.contains(".css")) {
                    response = 
                        "HTTP/1.1 200 OK\n" +
                        "Content-Type: text/css; charset=utf-8\n" +
                        "\n" +
                        "body { background-color: lightblue; }\n";
                }
    
                // 2. otherwise, assume the client wanted the HTML
                else {
                    response = 
                        "HTTP/1.1 200 OK\n" +
                        "Content-Type: text/html; charset=utf-8\n" +
                        "\n" +
                        "<html><body>\n" +
                        "<li>battery voltage: <strong>" + m_battVoltage + "</strong>\n" +
                        "<li>motors connected: <strong>" + m_numMotorsConnected + "</strong>\n" +
                        "</body></html>";
                }

                // done
                if (response != null)
                    client.getOutputStream().write(response.getBytes(StandardCharsets.UTF_8));
                client.close();
            }
        } catch (IOException ex) {
            System.out.println("Server exception: " + ex.getMessage());
            ex.printStackTrace();
        }
        catch (InterruptedException e) {
            System.out.println("Server interrupted: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private final int m_port;
    public RobotHttpResponder(int port) { m_port = port; }
}
