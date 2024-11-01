#pragma once
#include "types.h"
#include <string>
#include <vector>
#include <functional>
#include <opencv4/opencv2/opencv.hpp>

namespace route_opt {

struct VisualizerConfig {
  int width = 800;
  int height = 800;
  int fps = 30;
  std::string outputPath = "route.mp4";

  cv::Scalar backgroundColor = cv::Scalar(0, 0, 0);
  cv::Scalar pointColor = cv::Scalar(0, 255, 255);
  cv::Scalar routeColor = cv::Scalar(0, 165, 255);
  cv::Scalar activePointColor = cv::Scalar(0, 255, 0);
  cv::Scalar gridColor = cv::Scalar(50, 50, 50);

  int pointRadius = 5;
  int lineThickness = 2;
  bool showGrid = true;
  bool showProgress = true;
  int transitionFrames = 15;
  double padding = 0.1;

};

class RouteVisualizer {
  public:
    explicit RouteVisualizer(const VisualizerConfig& config = VisualizerConfig{});
    ~RouteVisualizer();

    void beginRecording();
    void addFrame(const Pointvector& points, const Route& currentRoute);
    void addIntermediateRoute(const Route& route, double progress);
    void finalizeVideo();

    using ProgressCallback = std::function<void(const Route&, double)>;
    void setProgressCallback(ProgressCallback& callback);

    void showPreview(bool enable);
    void updatePreview();

  private:
      VisualizerConfig config_;
      cv::VideoWriter videoWriter_;
      cv::Mat canvas_;
      bool isRecording_ = false;
      bool showPreview_ = false;
      ProgressCallback progressCallback_;

      struct Bounds {
          double minX, maxX, minY, maxY;
      };
      Bounds calculateBounds(const Pointvector& points) const;
      cv::Point2i transformPoint(const Point& point, const Bounds& bounds) const;

      void drawBackground();
      void drawGrid(const Bounds& bounds);
      void drawPoints(cosnt Pointvector& points, const Bounds& bounds);
      void drawRoute(const PointVector& points,const Route& route,
                     const Bounds& bounds, double progress = 1.0);
      void drawProgressInfo(double progress);

      void createTransition(const Route& fromRoute, const Route& toRoute,
                            const PointVector& points, const Bounds& bounds);

      void validatePoints(const Pointvector& points);
      void initializeVideo();
      void resizeCanvasIfNeeded();
};


};