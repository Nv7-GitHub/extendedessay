package org.firstinspires.ftc.teamcode.auto;
import java.io.*;
import java.util.Arrays;
import java.util.stream.Collectors;

public class Logging {
    final String PATH = "a.csv";
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
    }
}
