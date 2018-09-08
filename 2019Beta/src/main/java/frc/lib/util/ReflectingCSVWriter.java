package frc.lib.util;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.concurrent.ConcurrentLinkedDeque;

public class ReflectingCSVWriter<T> {

    ConcurrentLinkedDeque<String> mLinesToWrite = new ConcurrentLinkedDeque<>();
    Field[] mFields;
    PrintWriter mOutput = null;

    /**
     * sets up a logger that pulls public data from a data subclass
     * @param fileName including path to file
     * @param typeClass the data subclass to read from
     */
    public ReflectingCSVWriter(String fileName, Class<T> typeClass) {
        mFields = typeClass.getFields();
        try {
            mOutput = new PrintWriter(fileName);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        // Write field names.
        StringBuffer line = new StringBuffer();
        for (Field field : mFields) {
            if (line.length() != 0) {
                line.append(", ");
            }
            line.append(field.getName());
        }
        writeLine(line.toString());
    }


    /**
     * registers an update in values to a queue
     * @param value
     */
    public void add(T value) {
        StringBuffer line = new StringBuffer();
        for (Field field : mFields) {
            if (line.length() != 0) {
                line.append(", ");
            }
            try {
                if (CSVWritable.class.isAssignableFrom(field.getType())) {
                    line.append(((CSVWritable) field.get(value)).toCSV());
                } else {
                    line.append(field.get(value).toString());
                }
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        mLinesToWrite.add(line.toString());
    }

    // Call this periodically from any thread to write to disk.
    public void write() {
        while (true) {
            String val = mLinesToWrite.pollFirst();
            if (val == null) {
                break;
            }
            writeLine(val);
        }
    }

    /**
     * call periodically to write all updates to file
     * dumps queued updates into file
     */
    public synchronized void flush() {
        if (mOutput != null) {
            write();
            mOutput.flush();
        }
    }

    private void writeLine(String toWrite){
        //System.out.println("writing: " + toWrite);
        if(mOutput !=  null){
            mOutput.println(toWrite);
        }
    }

}
