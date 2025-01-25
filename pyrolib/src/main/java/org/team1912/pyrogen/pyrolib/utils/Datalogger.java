package org.team1912.pyrogen.pyrolib.utils;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

// from
// https://github.com/FIRST-Tech-Challenge/WikiSupport/blob/master/SampleOpModes/Datalogging/W_Datalogger_v05.java

public class Datalogger {

    private Writer writer;              // contains write() method to store file
    private StringBuffer lineBuffer;    // its methods build each line (row) of data
    private long timeBase;              // time of instantiation (milliseconds)
    private long nsBase;                // time of reset (nanoseconds)

    public Datalogger (String fileName) {
        String directoryPath    = "/sdcard/FIRST/java/src/Datalogs";
        String filePath         = directoryPath + "/" + fileName + ".txt";
        new File(directoryPath).mkdir();  // create Datalogs folder if needed
        try {
            writer = new FileWriter(filePath);
            lineBuffer = new StringBuffer(128);     // initial length 128
        }
        catch (IOException e) {
        }
        timeBase = System.currentTimeMillis();
        nsBase = System.nanoTime();
        addField("Time");               // first/default column label
        addField("d ms");               // second/default column label
    }

    private void flushLineBuffer(){
        try {
            lineBuffer.append('\n');                // end-of-line character
            writer.write(lineBuffer.toString());    // add line (row) to file
            lineBuffer.setLength(0);                // clear the line (row)
        }
        catch (IOException e) {
        }
    }

    private void insertTimestamps(){
        long milliTime,nanoTime;
        // Update time for first two columns (cumulative and incremental time).
        milliTime   = System.currentTimeMillis();
        nanoTime    = System.nanoTime();
        // Insert timestamps at position 0, *before* the OpMode data fields.
        lineBuffer.insert
                (0, String.format("%.3f",(milliTime - timeBase) / 1000.0) + ','
                        + String.format("%.3f",(nanoTime - nsBase) / 1.0E6) + ',');
        // Divide milliseconds by 1,000 to log seconds, in field named "Time".
        // Divide nanoseconds by 1,000,000 to log milliseconds, in "d ms".
        // The 1000.0 decimal and 1.0E6 scientific notation avoid a type error;
        // the expressions' variables are 'long'.
        nsBase      = nanoTime;         // reset for incremental time delta
    }

    public void firstLine() {
        flushLineBuffer();
    }

    // The OpMode calls this *public* method to add timestamps and complete the
    // current line (row) of data.
    public void newLine() {
        insertTimestamps();
        flushLineBuffer();
    }

    // These two (overloaded) methods add a text field to the line (row),
    // preceded by a comma.  This creates the comma-separated values (CSV).
    public void addField(String s) {
        if (lineBuffer.length()>0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(s);
    }
    public void addField(char c) {
        if (lineBuffer.length()>0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(c);
    }
    // Checking the line length (before inserting a comma) is not needed when a
    // default timestamp (and its comma) will be inserted before all data, as in
    // the current example. The check is here in case the default timestamp is removed.

    public void addField(boolean b) {
        addField(b ? '1' : '0');
    }

    public void addField(byte b) {
        addField(Byte.toString(b));
    }
    public void addField(short s) {
        addField(Short.toString(s));
    }
    public void addField(long l) {
        addField(Long.toString(l));
    }
    public void addField(float f) {
        addField(Float.toString(f));
    }
    public void addField(double d) {
        addField(Double.toString(d));
    }

    public void resetTime() {
        timeBase = System.currentTimeMillis();
        nsBase = System.nanoTime();
    }

    public void closeDataLogger() {
        try {
            writer.close();             // close the file
        }
        catch (IOException e) {
        }
    }

    @Override
    protected void finalize() throws Throwable {
        closeDataLogger();
        super.finalize();
    }
}
