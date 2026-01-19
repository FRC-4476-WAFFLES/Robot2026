// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.utils.obsolete;

// import java.util.ArrayList;
// import java.util.HashMap;

// import frc.robot.Robot;

// /** 
//  * Helper for keeping shufflebaord code clean.
//  * Use RegisterNetworkUser() to setup parameters how often to update networktables.
//  * RegisterNetworkUser() has a few overloads, namely letting you easily disable this tab while keeping all the relevant code around.
//  * As a rule of thumb to keep nt clean, try to disable any subsystems that aren't being actively debugged.
//  * To init this class call init() with a reference to the main robot class.
//  * To make a subsystem use this class implement the NetworkUser interface.
//  */
// public class SubsystemNetworkManager {
//     public static ArrayList<NetworkUser> m_networkUserList = new ArrayList<>();
//     public static HashMap<Double, NetworktablesUpdateRunnable> m_updateTasks = new HashMap<>();

//     private static Robot m_robot;
//     private static boolean m_setupComplete = false;

//     // Equal to the default periodic update rate
//     private static final double DefaultUpdatesPerSecond = 20;
//     private static final double DefaultUpdateOffsetSeconds = 0.01;

//     private static class NetworktablesUpdateRunnable implements Runnable {
//         ArrayList<NetworkUser> updateList = new ArrayList<>();

//         public void addUser(NetworkUser shuffleUser) {
//             updateList.add(shuffleUser);
//         }

//         public void run() {
//             for (int i = 0; i < updateList.size(); ++i) {
//                 updateList.get(i).updateNetwork();
//             }
//         }
//     }

//     public static void init(Robot robot) {
//         m_setupComplete = true;
//         m_robot = robot;
//     }

//     public static void RegisterNetworkUser(NetworkUser shuffleUser) {
//         RegisterNetworkUser(shuffleUser, true, DefaultUpdatesPerSecond);
//     }

//     public static void RegisterShuffleUser(NetworkUser shuffleUser, boolean enabled) {
//         RegisterNetworkUser(shuffleUser, enabled, DefaultUpdatesPerSecond);
//     }

//     public static void RegisterNetworkUser(NetworkUser networkUser, boolean enabled, double updatesPerSecond) {
//         if (!m_setupComplete) {
//             System.out.println("NT Manager not setup yet, init must be called first in robot initialization");
//             return;
//         }

//         if (!enabled) {
//             return;
//         }

//         // Avoid adding same class twice
//         if (m_networkUserList.contains(networkUser)) {
//             System.out.println("Cannot register same class twice. Class was: " + networkUser.getClass().getName());
//             return;
//         }

//         m_networkUserList.add(networkUser);
//         networkUser.initializeNetwork();
//         SetupUpdateTask(networkUser, updatesPerSecond);
//     }

//     private static void SetupUpdateTask(NetworkUser networkUser, double updatesPerSecond) {
//         NetworktablesUpdateRunnable runnable;
//         if (m_updateTasks.containsKey(updatesPerSecond)) {
//             runnable = m_updateTasks.get(updatesPerSecond);
//         } else {
//             runnable = new NetworktablesUpdateRunnable();
//             m_updateTasks.put(updatesPerSecond, runnable);
//             m_robot.addPeriodic(runnable, 1.0 / updatesPerSecond, DefaultUpdateOffsetSeconds);
//         }
//         runnable.addUser(networkUser);
//     }
// }
