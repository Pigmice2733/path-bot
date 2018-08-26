package frc.robot.utils;

import java.awt.BasicStroke;
import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartUtils;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class Plot {
    private XYPlot plot;
    private JFreeChart chart;
    private String title;
    private XYSeriesCollection dataset;
    private XYLineAndShapeRenderer renderer;
    private double duration;

    private final static List<Color> colors = Arrays.asList(Color.BLUE, Color.GREEN, Color.BLACK, Color.RED,
            Color.MAGENTA, Color.ORANGE);

    public interface Data {
        double get(double time);
    }

    public Plot(String title, Data data, double duration, String seriesName, double step) {
        this.title = title;
        this.duration = duration;

        XYSeries series = createSeries(data, duration, seriesName, step);
        dataset = new XYSeriesCollection();
        dataset.addSeries(series);

        chart = ChartFactory.createXYLineChart(title, "Time", "Value", dataset, PlotOrientation.VERTICAL, true, false,
                false);
        plot = chart.getXYPlot();

        renderer = new XYLineAndShapeRenderer();
        renderer.setDefaultShapesVisible(false);
        renderer.setSeriesPaint(0, Color.BLUE);
        renderer.setSeriesStroke(0, new BasicStroke(3.0f));

        plot.setRenderer(renderer);

        plot.setRangeGridlinesVisible(true);
        plot.getRangeAxis().setAxisLineVisible(true);
        plot.setRangeGridlinePaint(Color.BLACK);

        plot.setDomainGridlinesVisible(true);
        plot.setRangeZeroBaselineVisible(true);
        plot.setRangeZeroBaselineStroke(new BasicStroke(2.0f));
        plot.setDomainGridlinePaint(Color.BLACK);
    }

    public void addSeries(Data data, String seriesName, double step) {
        XYSeries series = createSeries(data, duration, seriesName, step);
        dataset.addSeries(series);

        int seriesIndex = dataset.getSeriesCount() - 1;
        renderer.setSeriesPaint(seriesIndex, getColor(seriesIndex));
        renderer.setSeriesStroke(seriesIndex, new BasicStroke(3.0f));
    }

    private XYSeries createSeries(Data data, double duration, String seriesName, double step) {
        XYSeries series = new XYSeries(seriesName);
        for (double i = 0.0; i < duration; i += step) {
            series.add(i, data.get(i));
        }
        series.add(duration, data.get(duration));
        return series;
    }

    private Color getColor(int i) {
        int index = i % colors.size();
        return colors.get(index);
    }

    public void savePlot(String directory) {
        File file = new File(directory + File.separator + title.toLowerCase() + ".png");

        File path = new File(directory);

        if (!path.exists()) {
            if (!file.getParentFile().mkdirs()) {
                System.err.println("Failed to create plot output directory " + file.getName());
            }
        }

        try {
            boolean success;
            if (!file.exists()) {
                success = file.createNewFile();
            } else {
                success = file.delete() && file.createNewFile();
            }
            if (!success) {
                System.err.println("Failed to create file " + file.getName());
            }
        } catch (IOException ex) {
            System.err.println(ex);
        }

        int width = 640;
        int height = 480;

        try {
            ChartUtils.saveChartAsPNG(file, chart, width, height);
        } catch (IOException ex) {
            System.err.println(ex);
        }
    }
}