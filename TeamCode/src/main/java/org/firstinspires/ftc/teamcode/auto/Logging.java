package org.firstinspires.ftc.teamcode.auto;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import java.io.*;
import java.util.Arrays;
import java.util.stream.Collectors;

@Config
public class Logging {
    public static String PATH = "/storage/self/primary/FIRST/data.csv";
    PrintWriter pw;

    public Logging(String[] headers) throws FileNotFoundException {
        File file = new File(PATH);
        this.pw = new PrintWriter(file);
        pw.println(String.join(",", headers));
    }

    public void Write(double[] values) {
        pw.println(Arrays.stream(values)
                .mapToObj(Double::toString)
                .collect(Collectors.joining(",")));
        pw.flush();
    }

    public void Close() {
        pw.close();
    }
}
