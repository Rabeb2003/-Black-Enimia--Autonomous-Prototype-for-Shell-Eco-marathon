#!/usr/bin/env python3
"""
Interface Graphique Chronomètre pour Panneau STOP (ENIMIA Black).
Ouvre une fenêtre moderne et élégante affichant une horloge circulaire de compte à rebours de 5 secondes.
"""

import sys
import os
import time
import math
import argparse
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import QTimer, Qt, QPoint
from PyQt5.QtGui import QPainter, QColor, QFont, QPen, QBrush, QPolygon, QLinearGradient


class StopSignClockWidget(QWidget):
    def __init__(self, duration=5.0):
        super().__init__()
        self.duration = float(duration)
        self.remaining = self.duration
        self.start_time = time.monotonic()
        self.completed = False

        self.setWindowTitle("ENIMIA - Arrêt STOP Chronomètre (5s)")
        self.setFixedSize(480, 560)
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground, False)

        # Center on screen
        desktop = QApplication.desktop().screenGeometry()
        self.move((desktop.width() - 480) // 2, (desktop.height() - 560) // 2)

        # High-frequency timer for ultra-smooth 60 FPS animation
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(16)  # ~60 fps

    def tick(self):
        now = time.monotonic()
        elapsed = now - self.start_time
        self.remaining = max(0.0, self.duration - elapsed)

        if self.remaining <= 0.0 and not self.completed:
            self.completed = True
            # Auto-close after 800ms of "REPRISE DU CIRCUIT"
            QTimer.singleShot(800, self.close)

        self.update()

    def draw_octagon(self, painter, cx, cy, radius):
        """Draw a classic red STOP octagon badge."""
        points = []
        for i in range(8):
            angle = math.radians(22.5 + i * 45.0)
            px = int(cx + radius * math.cos(angle))
            py = int(cy + radius * math.sin(angle))
            points.append(QPoint(px, py))
        poly = QPolygon(points)
        painter.setBrush(QBrush(QColor(220, 20, 30)))
        painter.setPen(QPen(QColor(255, 255, 255), 3))
        painter.drawPolygon(poly)

        # White inner border
        inner_points = []
        for i in range(8):
            angle = math.radians(22.5 + i * 45.0)
            px = int(cx + (radius - 5) * math.cos(angle))
            py = int(cy + (radius - 5) * math.sin(angle))
            inner_points.append(QPoint(px, py))
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.drawPolygon(QPolygon(inner_points))

        # Text "STOP"
        painter.setPen(QColor(255, 255, 255))
        painter.setFont(QFont("DejaVu Sans", 13, QFont.Bold))
        painter.drawText(cx - 35, cy - 15, 70, 30, Qt.AlignCenter, "STOP")

    def paintEvent(self, event):
        import math
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Modern dark gradient background
        bg_gradient = QLinearGradient(0, 0, 0, self.height())
        bg_gradient.setColorAt(0.0, QColor(20, 22, 28))
        bg_gradient.setColorAt(1.0, QColor(10, 11, 15))
        painter.fillRect(self.rect(), bg_gradient)

        # Window outline border
        painter.setPen(QPen(QColor(60, 65, 80), 2))
        painter.drawRect(1, 1, self.width() - 2, self.height() - 2)

        # Top Header Banner
        painter.fillRect(2, 2, self.width() - 4, 60, QColor(28, 32, 42))
        painter.setPen(QColor(255, 255, 255))
        painter.setFont(QFont("DejaVu Sans", 13, QFont.Bold))
        painter.drawText(20, 20, self.width() - 40, 25, Qt.AlignLeft, "SYSTEME AUTONOME - ENIMIA BLACK")

        # Status badge
        status_bg = QColor(220, 30, 40) if not self.completed else QColor(30, 200, 80)
        painter.setBrush(QBrush(status_bg))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(self.width() - 170, 16, 150, 28, 6, 6)
        painter.setPen(QColor(255, 255, 255))
        painter.setFont(QFont("DejaVu Sans", 9, QFont.Bold))
        status_text = "ARRET EN COURS" if not self.completed else "REPRISE CIRCUIT"
        painter.drawText(self.width() - 170, 16, 150, 28, Qt.AlignCenter, status_text)

        # STOP Octagon Icon
        self.draw_octagon(painter, self.width() // 2, 115, 34)

        # Circular Clock Dial
        dial_cx = self.width() // 2
        dial_cy = 285
        dial_radius = 125

        # Background track of the gauge
        track_pen = QPen(QColor(40, 45, 58), 14)
        track_pen.setCapStyle(Qt.RoundCap)
        painter.setPen(track_pen)
        painter.drawEllipse(dial_cx - dial_radius, dial_cy - dial_radius, dial_radius * 2, dial_radius * 2)

        # Progress Arc (Sweeps clockwise from 90 degrees at top)
        ratio = self.remaining / self.duration
        span_angle = int(ratio * 360 * 16)
        start_angle = 90 * 16

        # Dial arc color: Red during countdown, transforms to electric blue/green when done
        if self.completed:
            arc_color = QColor(40, 220, 100)
        elif self.remaining <= 1.5:
            arc_color = QColor(255, 170, 0)
        else:
            arc_color = QColor(235, 45, 55)

        arc_pen = QPen(arc_color, 14)
        arc_pen.setCapStyle(Qt.RoundCap)
        painter.setPen(arc_pen)
        painter.drawArc(dial_cx - dial_radius, dial_cy - dial_radius, dial_radius * 2, dial_radius * 2, start_angle, -span_angle)

        # Analog Clock Tick Marks (12 ticks around clock)
        for i in range(12):
            angle = math.radians(i * 30.0)
            inner_r = dial_radius - 22
            outer_r = dial_radius - 12
            x1 = int(dial_cx + inner_r * math.cos(angle))
            y1 = int(dial_cy + inner_r * math.sin(angle))
            x2 = int(dial_cx + outer_r * math.cos(angle))
            y2 = int(dial_cy + outer_r * math.sin(angle))
            painter.setPen(QPen(QColor(100, 110, 130), 2))
            painter.drawLine(x1, y1, x2, y2)

        # Analog Clock Hand (rotating)
        hand_angle = math.radians(90.0 - (1.0 - ratio) * 360.0)
        hand_len = dial_radius - 32
        hand_x = int(dial_cx + hand_len * math.cos(hand_angle))
        hand_y = int(dial_cy - hand_len * math.sin(hand_angle))
        painter.setPen(QPen(arc_color, 4))
        painter.drawLine(dial_cx, dial_cy, hand_x, hand_y)
        # Center pivot
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(dial_cx - 6, dial_cy - 6, 12, 12)

        # Big Digital Countdown Timer (Center Display)
        painter.setPen(QColor(255, 255, 255))
        painter.setFont(QFont("DejaVu Sans", 32, QFont.Bold))
        if self.completed:
            digits_text = "0.0 s"
        else:
            digits_text = f"{self.remaining:.1f} s"
        painter.drawText(dial_cx - 100, dial_cy + 40, 200, 45, Qt.AlignCenter, digits_text)

        painter.setFont(QFont("DejaVu Sans", 10))
        painter.setPen(QColor(150, 160, 180))
        painter.drawText(dial_cx - 100, dial_cy + 85, 200, 20, Qt.AlignCenter, "TEMPS RESTANT")

        # Bottom Informational Card
        card_rect_y = 445
        painter.fillRect(20, card_rect_y, self.width() - 40, 95, QColor(22, 25, 34))
        painter.setPen(QPen(QColor(45, 50, 65), 1))
        painter.drawRoundedRect(20, card_rect_y, self.width() - 40, 95, 8, 8)

        if not self.completed:
            painter.setPen(QColor(255, 80, 80))
            painter.setFont(QFont("DejaVu Sans", 11, QFont.Bold))
            painter.drawText(35, card_rect_y + 25, "PANNEAU STOP DETECTE (REGULATION SEM)")
            painter.setPen(QColor(200, 205, 220))
            painter.setFont(QFont("DejaVu Sans", 9))
            painter.drawText(35, card_rect_y + 48, "Prototype immobilise completement a la ligne.")
            painter.drawText(35, card_rect_y + 68, f"Arret obligatoire : {self.duration:.1f} secondes chrono...")
        else:
            painter.setPen(QColor(50, 220, 120))
            painter.setFont(QFont("DejaVu Sans", 11, QFont.Bold))
            painter.drawText(35, card_rect_y + 25, "ARRET 5 SECONDES COMPLETE [OK]")
            painter.setPen(QColor(220, 255, 230))
            painter.setFont(QFont("DejaVu Sans", 9))
            painter.drawText(35, card_rect_y + 48, "Demarrage moteur & controleur de voie.")
            painter.drawText(35, card_rect_y + 68, "Reprise de la navigation sur le circuit en cours...")


def main():
    parser = argparse.ArgumentParser(description="Chronomètre UI Arrêt STOP 5s")
    parser.add_argument("--duration", type=float, default=5.0, help="Durée de l'arrêt en secondes")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    clock = StopSignClockWidget(duration=args.duration)
    clock.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
