package com.robo;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.Ellipse2D;
import java.util.Date;
import java.util.Random;

public class Circle {

    public static void main(String[] args) {
        new Circle();
    }

    public Circle() {
        EventQueue.invokeLater(new Runnable() {
            @Override
            public void run() {
                try {
                    UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
                } catch (ClassNotFoundException | InstantiationException | IllegalAccessException | UnsupportedLookAndFeelException ex) {
                }

                JFrame frame = new JFrame("Circle");
                frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
                frame.setLayout(new BorderLayout());
                frame.add(new CirclePane());
                frame.pack();
                frame.setLocationRelativeTo(null);
                frame.setVisible(true);
            }
        });
    }

    public class CirclePane extends JPanel {

        private Random random = new Random(new Date().getTime());
        private double x = 100;
        private double y = 100;
        private int radius = 20;
        private double theta = random.nextInt(360)/(2 * Math.PI);
        private double delta_x = Math.cos(theta);
        private double delta_y = Math.sin(theta);

        public CirclePane() {
            Timer timer = new Timer(40, new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    if (x + radius < getWidth() && x > 0 && y + radius < getHeight() && y > 0) {
                        x += delta_x;
                        y += delta_y;
                    }
                    repaint();
                }
            });
            timer.start();
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(200, 200);
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            g.setColor(Color.RED);
            drawOval(g, x, y, radius);
        }

        private void drawOval(Graphics gg, double x, double y, double radius) {
            Graphics2D g = (Graphics2D) gg;
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
            Ellipse2D.Double shape = new Ellipse2D.Double(x, y, radius, radius);
            g.fill(shape);
        }
    }

}