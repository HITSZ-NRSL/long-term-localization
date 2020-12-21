// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-1.

#include "depth_clustering/depth_clustering_utils.h"

#include <algorithm>

namespace long_term_relocalization {


int WrapCircularIndex(int x, int width) {
  if (x >= width) {
    return x - width;
  } else if (x < 0) {
    return x + width;
  }
  return x;
}

class color {
private:
public:
  using rgb = std::array<uint8_t, 3>;
  using rgbs = std::vector<rgb>;
  static rgb HSL2RGB(double H, double S, double L) {
    rgb res{{0, 0, 0}};

    H = H < 0 || H > 360 ? 180 : H;
    S = S < 0 || S > 1 ? 0.5 : S;
    L = L < 0 || L > 1 ? 0.5 : L;

    double C = (1 - std::abs(2 * L - 1)) * S;
    double X = C * (1 - std::abs(std::fmod(H / 60, 2) - 1));
    double m = L - C / 2;

    double RR, GG, BB;

    if (H < 60) {
      RR = C;
      GG = X;
      BB = 0;
    } else if (H < 120) {
      RR = X;
      GG = C;
      BB = 0;
    } else if (H < 180) {
      RR = 0;
      GG = C;
      BB = X;
    } else if (H < 240) {
      RR = 0;
      GG = X;
      BB = C;
    } else if (H < 300) {
      RR = X;
      GG = 0;
      BB = C;
    } else {
      RR = X;
      GG = 0;
      BB = C;
    }

    res[0] = static_cast<uint8_t>((RR + m) * 255);
    res[1] = static_cast<uint8_t>((GG + m) * 255);
    res[2] = static_cast<uint8_t>((BB + m) * 255);
    return res;
  };
  static rgbs get_rgbs(uint32_t n, double S = 0.5, double L = 0.5) {
    n = !n ? 1 : n; // at least 1
    rgbs res(n);
    double step = 360 / n;
    double H = 0;
    for (uint32_t i = 0; i < n; i++, H += step) {
      res[i] = HSL2RGB(H, S, L);
    }

    return res;
  }
};

cv::Mat RenderRangeImage(const common::FixedArray2D<double> &range_image) {
  const double resolution = 1.0;
  const double depth_gap = 100.0;

  uint32_t num = static_cast<uint32_t>(depth_gap / resolution);
  color::rgbs colors = color::get_rgbs(num);

  cv::Mat depth_image = cv::Mat::zeros(range_image.rows(), range_image.cols(), CV_8UC3);
  for (int r = 0; r < range_image.rows(); ++r) {
    for (int c = 0; c < range_image.cols(); ++c) {
      const double range = std::min(range_image(r, c), depth_gap);
      uint32_t idx = static_cast<uint32_t>(range / resolution);
      if (idx >= num) {
        idx = num - 1;
      }

      const color::rgb rgb = colors[idx];
      if (range_image(r, c) < common::kEpsilon) {
        depth_image.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);
      } else {
        depth_image.at<cv::Vec3b>(r, c) = cv::Vec3b(rgb[2], rgb[1], rgb[0]);
      }
    }
  }

  return depth_image;
}


} // namespace long_term_relocalization
