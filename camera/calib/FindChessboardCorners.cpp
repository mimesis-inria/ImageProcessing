
sofaor::processor::cam::calib::FindPatternCorners::FindPatternCorners()
    : ImageFilter(false),
      d_imagePoints(initData(&d_imagePoints, "imagePoints",
                             "output vector of image points")),
      d_patternType(initData(
          &d_patternType, "patternType",
          "dotted pattern or chessboard pattern. (Options are: DOT, CHESS)")),
      d_patternSize(
          initData(&d_patternSize, "patternSize",
                   "dimensions of the pattern (cols, rows). More precisely, "
                   "the number of dots on a "
                   "dotted pattern, or the number of square intersections "
                   "on a chessboard pattern (nb squares - 1).")),
      d_detectRate(initData(&d_detectRate, "captureRate",
                            "number of frames captured per second")),
      d_flags(initData(
          &d_flags, cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_ADAPTIVE_THRESH,
          "flags",
          "Opencv flags value to determine how the pattern is detected")),
      d_refineCorners(initData(
          &d_refineCorners, true, "refineCorners",
          "set to false if you don't want cv::cornerSubPix() to be called"))
{
  helper::OptionsGroup* o = d_patternType.beginEdit();
  o->setNames(2, "DOT", "CHESS");
  o->setSelectedItem("CHESS");
  d_patternType.endEdit();
}

void sofaor::processor::cam::calib::FindPatternCorners::applyFilter(
    const cv::Mat& in, cv::Mat& out, bool)
{
  bool found = false;
  std::vector<cv::Point2f> corners;
  if (d_patternType.getValue().getSelectedItem() == "CHESS")
    found = cv::findChessboardCorners(
        in,
        cv::Size(d_patternSize.getValue().x(), d_patternSize.getValue().y()),
        corners, d_flags.getValue());
  else
    found = cv::findCirclesGrid(
        in,
        cv::Size(d_patternSize.getValue().x(), d_patternSize.getValue().y()),
        corners, d_flags.getValue());
  if (found && d_refineCorners.getValue())
    cv::cornerSubPix(
        *dst, *corners, cv::Size(5, 5), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                         0.1));
  out = in.clone();
  cv::drawChessboardCorners(
      m, cv::Size(d_patternSize.getValue().x(), d_patternSize.getValue().y()),
      corners, found);

  helper::vector<defaulttype::vec2i>& pts = *d_imagePoints.beginEdit();
  pts.clear();
  for (auto pt : corners) pts.push_back(defaulttype::Vec2i(pt.x, pt.y));
  d_imagePoints.endEdit();
}

void sofaor::processor::cam::calib::FindPatternCorners::init()
{
  ImageFilter::init();
}
