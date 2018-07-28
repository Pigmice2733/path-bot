package frc.robot;

import java.awt.BasicStroke;
import java.awt.Color;
import java.io.File;
import java.io.IOException;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import frc.robot.motion.StaticProfile;

public class ProfilePlotter {

    public static void plotVelocity(StaticProfile profile, double step, String name) {
        XYSeries data = new XYSeries("Velocity");

        for (double i = 0.0; i <= profile.profileDuration; i += step) {
            data.add(i, profile.getVelocity(i));
        }

        plot(data, "Velocity", name);
    }

    public static void plotPosition(StaticProfile profile, double step, String name) {
        XYSeries data = new XYSeries("Position");

        for (double i = 0.0; i <= profile.profileDuration; i += step) {
            data.add(i, profile.getPosition(i));
        }

        plot(data, "Position", name);
    }

    private static void plot(XYSeries series, String name, String type) {
        XYSeriesCollection data = new XYSeriesCollection();
        data.addSeries(series);

        JFreeChart chart = ChartFactory.createXYLineChart("Profile", "Time", name, data, PlotOrientation.VERTICAL,
                false, false, false);

        XYPlot plot = chart.getXYPlot();

        XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();

        renderer.setSeriesPaint(0, Color.BLUE);
        renderer.setSeriesStroke(0, new BasicStroke(3.0f));

        plot.setRenderer(renderer);

        plot.setRangeGridlinesVisible(true);
        plot.setRangeGridlinePaint(Color.BLACK);

        plot.setDomainGridlinesVisible(true);
        plot.setDomainGridlinePaint(Color.BLACK);

        // Create graph output dir if not exists
        File directory = new File("./graphs/");
        if (!directory.exists()) {
            directory.mkdir();
        }

        // Ex. "trapezoid" -> "./graphs/position-trapezoid-profile.png"
        File imageFile = new File("./graphs/" + type.toLowerCase() + "-" + name.toLowerCase() + "-profile.png");

        int width = 640;
        int height = 480;

        try {
            ChartUtilities.saveChartAsPNG(imageFile, chart, width, height);
        } catch (IOException ex) {
            System.err.println(ex);
        }
    }
}