package org.firstinspires.ftc.teamcode;

import java.io.File; // Import the File class

import java.io.FileNotFoundException; // Import this class to handle errors
import java.io.FileWriter; // Import the FileWriter class
import java.io.IOException; // Import the IOException class to handle errors

import java.util.Scanner; // Import the Scanner class to read text files

import android.os.Environment;


public class Files {


    public static Boolean saveFileString(String name, String value) {
        try {
            String logFilePath = String.format("%s/%s.txt",
                    Environment.getExternalStorageDirectory().getAbsolutePath(), name);
            //"/sdcard", name);
            System.out.println(logFilePath);

            File myObj = new File(logFilePath);
            myObj.delete();
            myObj.createNewFile();
            FileWriter myWriter = new FileWriter(logFilePath);
            myWriter.write(value);
            myWriter.close();
            return true;



        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
            return false;
        }
    }

    public static String readFileString(String name) {
        try {
            String logFilePath = String.format("%s/%s.txt",
                    Environment.getExternalStorageDirectory().getAbsolutePath(), name);
            //"/sdcard", name);


            File myObj = new File(logFilePath);
            Scanner myReader = new Scanner(myObj);
            String data = myReader.nextLine();
            System.out.println(data);
            myReader.close();
            return data;

        } catch (FileNotFoundException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
            return "";
        }
    }



    public static Boolean saveFileDouble(String name, double value) {
        return saveFileString(name, String.valueOf(value));
    }

    public static double readFileDouble(String name) {
        try {
            String logFilePath = String.format("%s/%s.txt",
                    Environment.getExternalStorageDirectory().getAbsolutePath(), name);
            //"/sdcard", name);


            File myObj = new File(logFilePath);
            Scanner myReader = new Scanner(myObj);
            String data = myReader.nextLine();
            System.out.println(data);
            myReader.close();
            return Double.parseDouble(data);

        } catch (FileNotFoundException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
            return 0.0;
        }
    }
}
