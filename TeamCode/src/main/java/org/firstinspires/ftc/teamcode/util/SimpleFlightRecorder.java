package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class SimpleFlightRecorder {
    private static final String TAG = "SimpleFlightRecorder";
    private static final String LOG_DIR = "/sdcard/FIRST/RoadRunner/logs";
    private static final Gson gson = new GsonBuilder().setPrettyPrinting().create();
    private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US);
    private static String currentLogFile = null;

    private static void ensureLogDirectory() {
        File dir = new File(LOG_DIR);
        if (!dir.exists()) {
            dir.mkdirs();
        }
    }

    private static String getLogFileName() {
        if (currentLogFile == null) {
            currentLogFile = dateFormat.format(new Date()) + ".log";
        }
        return currentLogFile;
    }

    public static void write(String key, Object value) {
        try {
            ensureLogDirectory();
            File logFile = new File(LOG_DIR, getLogFileName());
            FileWriter writer = new FileWriter(logFile, true);
            
            // Format exactly like Road Runner's FlightRecorder
            String timestamp = String.format(Locale.US, "%.3f", System.nanoTime() / 1e9);
            String json = gson.toJson(value);
            writer.write(String.format("%s %s %s\n", timestamp, key, json));
            writer.close();
        } catch (IOException e) {
            Log.e(TAG, "Error writing to log file", e);
        }
    }

    public static void reset() {
        currentLogFile = null;
    }

    // Helper class to match Road Runner's PoseMessage format exactly
    public static class PoseMessage {
        public long timestamp;
        public double x;
        public double y;
        public double heading;

        public PoseMessage(double x, double y, double heading) {
            this.timestamp = System.nanoTime();
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }
} 