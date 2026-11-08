#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fused Symmetrical Lane Detector for lane_follower.
Key Improvements:
1) Fits 2nd or 3rd order polynomials for BOTH left and right lanes.
2) Dynamic base tracking across full lateral width so lanes aren't lost in sharp turns.
3) Publishes:
   - /lane/left_fit  (Float32MultiArray)
   - /lane/right_fit (Float32MultiArray)
   - /car/bev_image  (Image)
   - /car/sliding_window_image (Image)
"""
import rclpy
import cv2
import numpy as np
import warnings
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

fixed_points = [(10, 450), (540, 430), (435, 240), (190, 240)]
x_m_per_pixel = 0.0016472868217054263
y_m_per_pixel = 0.003703703703703704

prev_left_base, prev_right_base = None, None
warnings.filterwarnings("ignore", category=np.RankWarning)

class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")
        self.bridge = CvBridge()

        # Publishers
        self.pub_left = self.create_publisher(Float32MultiArray, "/lane/left_fit", 1)
        self.pub_right = self.create_publisher(Float32MultiArray, "/lane/right_fit", 1)
        self.pub_bev = self.create_publisher(Image, "/car/bev_image", 1)
        self.pub_dbg = self.create_publisher(Image, "/car/sliding_window_image", 1)

        # Subscribers
        self.create_subscription(Image, "/oak/left/image_raw", self.cb_image, 1)

    @staticmethod
    def get_bird_eye_view(img, out_size, pts):
        h, w = out_size[1], out_size[0]
        M = cv2.getPerspectiveTransform(
            np.float32([pts[0], pts[1], pts[3], pts[2]]),
            np.float32([[0, h], [w, h], [0, 0], [w, 0]])
        )
        return cv2.warpPerspective(img, M, out_size)

    @staticmethod
    def mask_hls(img, lower, upper, ksize=(21, 21)):
        hls = cv2.cvtColor(cv2.GaussianBlur(img, ksize, 0), cv2.COLOR_BGR2HLS)
        return cv2.inRange(hls, np.array(lower), np.array(upper))

    @staticmethod
    def remove_circles(mask, rmin=25, rmax=56):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if len(c) < 5:
                continue
            (_, _), r = cv2.minEnclosingCircle(c)
            if rmin < r < rmax:
                cv2.drawContours(mask, [c], -1, 0, -1)
        return mask

    def sliding_window(self, bev, r_n=18, l_n=18, r_margin=80, l_margin=80, minpix=40):
        global prev_left_base, prev_right_base
        mask = self.mask_hls(bev, [0, 190, 0], [180, 255, 110])
        mask = self.remove_circles(mask)

        # Histogram across bottom 30%
        hist = np.sum(mask[int(mask.shape[0] * 0.70):, :], axis=0)
        mid = hist.shape[0] // 2

        left_peak = np.argmax(hist[:mid]) if np.max(hist[:mid]) > 10 else None
        right_peak = (np.argmax(hist[mid:]) + mid) if np.max(hist[mid:]) > 10 else None

        if left_peak is not None:
            leftx_base = left_peak
            prev_left_base = leftx_base
        elif prev_left_base is not None:
            leftx_base = prev_left_base
        else:
            leftx_base = mid // 2

        if right_peak is not None:
            rightx_base = right_peak
            prev_right_base = rightx_base
        elif prev_right_base is not None:
            rightx_base = prev_right_base
        else:
            rightx_base = mid + mid // 2

        nz_y, nz_x = mask.nonzero()
        out = np.dstack((mask, mask, mask))

        l_h = mask.shape[0] // l_n
        r_h = mask.shape[0] // r_n
        lx_cur, rx_cur = leftx_base, rightx_base

        lx, ly, rx, ry = [], [], [], []

        for win in range(max(r_n, l_n)):
            ly_low, ly_high = mask.shape[0] - (win + 1) * l_h, mask.shape[0] - win * l_h
            ry_low, ry_high = mask.shape[0] - (win + 1) * r_h, mask.shape[0] - win * r_h
            lx_low, lx_high = lx_cur - l_margin, lx_cur + l_margin
            rx_low, rx_high = rx_cur - r_margin, rx_cur + r_margin

            # Left window
            good_l = ((nz_y >= ly_low) & (nz_y < ly_high) & (nz_x >= lx_low) & (nz_x < lx_high)).nonzero()[0]
            if len(good_l) > minpix:
                lx_cur = int(np.mean(nz_x[good_l]))
                lx.append(lx_cur)
                ly.append(int(np.mean(nz_y[good_l])))
                cv2.rectangle(out, (lx_low, ly_low), (lx_high, ly_high), (255, 0, 0), 2)

            # Right window
            good_r = ((nz_y >= ry_low) & (nz_y < ry_high) & (nz_x >= rx_low) & (nz_x < rx_high)).nonzero()[0]
            if len(good_r) > minpix:
                rx_cur = int(np.mean(nz_x[good_r]))
                rx.append(rx_cur)
                ry.append(int(np.mean(nz_y[good_r])))
                cv2.rectangle(out, (rx_low, ry_low), (rx_high, ry_high), (0, 0, 255), 2)

        return out, lx, ly, rx, ry

    def cb_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        bev = self.get_bird_eye_view(frame, (frame.shape[1], frame.shape[0]), fixed_points)
        dbg, lx, ly, rx, ry = self.sliding_window(bev)

        left_fit_msg = Float32MultiArray()
        right_fit_msg = Float32MultiArray()

        # Fit 2nd or 3rd order polynomials symmetrically for BOTH sides!
        if len(lx) >= 4:
            deg = 3 if len(lx) >= 6 else 2
            lfit = np.polyfit((450 - np.array(ly)) * y_m_per_pixel,
                              (275 - np.array(lx)) * x_m_per_pixel, deg)
            left_fit_msg.data = lfit.tolist()
            self.pub_left.publish(left_fit_msg)
        elif len(lx) >= 2:
            lfit = np.polyfit((450 - np.array(ly)) * y_m_per_pixel,
                              (275 - np.array(lx)) * x_m_per_pixel, 1)
            left_fit_msg.data = lfit.tolist()
            self.pub_left.publish(left_fit_msg)

        if len(rx) >= 4:
            deg = 3 if len(rx) >= 6 else 2
            rfit = np.polyfit((450 - np.array(ry)) * y_m_per_pixel,
                              (275 - np.array(rx)) * x_m_per_pixel, deg)
            right_fit_msg.data = rfit.tolist()
            self.pub_right.publish(right_fit_msg)
        elif len(rx) >= 2:
            rfit = np.polyfit((450 - np.array(ry)) * y_m_per_pixel,
                              (275 - np.array(rx)) * x_m_per_pixel, 1)
            right_fit_msg.data = rfit.tolist()
            self.pub_right.publish(right_fit_msg)

        try:
            self.pub_bev.publish(self.bridge.cv2_to_imgmsg(bev, "bgr8"))
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
        except CvBridgeError as e:
            self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
