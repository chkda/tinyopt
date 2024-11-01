#include "visualizer.h"
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace route_opt {

    RouteVisualizer::RouteVisualizer(const VisualizerConfig &config) :
        config_(config) {
        canvas_ = cv::Mat(config_.height, config_.width, CV_8UC3);
    }

    RouteVisualizer::~RouteVisualizer() {
        if (isRecording_) {
            finalizeVideo();
        }
    }

    void RouteVisualizer::beginRecording() {
        if (isRecording_) {
            throw std::runtime_error("Recording already in progress");
        }

        videoWriter_.open(config_.outputPath,
                          cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                          config_.fps,
                          cv::Size(config_.width, config_.height));

        if (!videoWriter_.isOpened()) {
            throw std::runtime_error("Failed to open video writer");
        }
        isRecording_ = true;
    }

    void RouteVisualizer::addFrame(const PointVector &points, const Route &currentRoute) {
        if (!isRecording_) {
            throw std::runtime_error("Recording not started");
        }

        validatePoints(points);
        auto bounds = calculateBounds(points);

        drawBackground();
        if (config_.showGrid) {
            drawGrid(bounds);
        }

        drawPoints(points, bounds);
        drawRoute(points, currentRoute, bounds);

        if (config_.showGrid) {
            drawProgressInfo(1.0);
        }

        videoWriter_.write(canvas_);

        if (showPreview_) {
            updatePreview();
        }
    }

    void RouteVisualizer::addIntermediateRoute(const Route &route, double progress) {
        if (progressCallback_) {
            progressCallback_(route, progress);
        }
    }

    void RouteVisualizer::finalizeVideo() {
        if (!isRecording_) {
            return;
        }

        videoWriter_.release();
        isRecording_ = false;
    }

    void RouteVisualizer::setProgressCallback(ProgressCallback &callback) {
        progressCallback_ = std::move(callback);
    }

    void RouteVisualizer::showPreview(bool enable) {
        showPreview_ = enable;
        if (!enable && !canvas_.empty()) {
            cv::destroyWindow("Route Optimization preview");
        }
    }

    void RouteVisualizer::updatePreview() {
        cv::imshow("Route Optimization preview", canvas_);
        cv::waitKey(1);
    }

    RouteVisualizer::Bounds RouteVisualizer::calculateBounds(const PointVector &points) const {
        if (points.empty()) {
            return {0, 1, 0, 1};
        }

        Bounds bounds{
                points[0].x, points[0].x,
                points[0].y, points[0].y
        };

        for (const auto &point: points) {
            bounds.minX = std::min(bounds.minX, point.x);
            bounds.minY = std::min(bounds.minY, point.y);
            bounds.maxX = std::max(bounds.maxX, point.x);
            bounds.maxY = std::max(bounds.maxY, point.y);
        }

        double padX = (bounds.maxX - bounds.minX) * config_.padding;
        double padY = (bounds.maxY - bounds.minY) * config_.padding;

        bounds.minX -= padX;
        bounds.minY -= padY;
        bounds.maxX += padX;
        bounds.maxY += padY;

        return bounds;
    }

    cv::Point2i RouteVisualizer::transformPoint(const Point &point, const Bounds &bounds) const {
        int x = static_cast<int>((point.x - bounds.minX) / (bounds.maxX - bounds.minX) * (config_.width - 20) + 10);
        int y = static_cast<int>((point.y - bounds.minY) / (bounds.maxY - bounds.minY) * (config_.height - 20) + 10);
        return cv::Point2i(x, y);
    }

    void RouteVisualizer::drawBackground() {
        canvas_ = config_.backgroundColor;
    }

    void RouteVisualizer::drawGrid(const Bounds &bounds) {
        for (int x = 0; x < config_.width; x += 50) {
            cv::line(canvas_, cv::Point(x, 0), cv::Point(x, config_.height), config_.gridColor, 1);
        }

        for (int y = 0; y < config_.height; y += 50) {
            cv::line(canvas_, cv::Point(0, y), cv::Point(config_.width, y), config_.gridColor, 1);
        }
    }

    void RouteVisualizer::drawPoints(const PointVector &points, const Bounds &bounds) {
        for (const auto &point: points) {
            cv::Point2i pos = transformPoint(point, bounds);
            cv::circle(canvas_, pos, config_.pointRadius, config_.pointColor, -1);
        }
    }

    void RouteVisualizer::drawRoute(const PointVector &points, const Route &route,
                                    const Bounds &bounds, double progress) {

        if (route.path.empty()) {
            return;
        }

        size_t numPoints = static_cast<size_t>(route.path.size() * progress);
        for (size_t i = 1; i < numPoints; ++i) {
            cv::Point2i p1 = transformPoint(points[route.path[i - 1]], bounds);
            cv::Point2i p2 = transformPoint(points[route.path[i]], bounds);

            cv::line(canvas_, p1, p2, config_.routeColor, config_.lineThickness);

            if (i == numPoints - 1) {
                cv::circle(canvas_, p2, config_.pointRadius + 2, config_.activePointColor, -1);
            }
        }

        if (progress >= 1.0 && route.path.size() > 2) {
            cv::Point2i p1 = transformPoint(points[route.path.back()], bounds);
            cv::Point2i p2 = transformPoint(points[route.path.front()], bounds);
            cv::line(canvas_, p1, p2, config_.routeColor, config_.lineThickness);
        }
    }

    void RouteVisualizer::drawProgressInfo(double progress) {
        std::string progressText = "Progress: " + std::to_string(int(progress * 100)) + "%";
        cv::putText(canvas_, progressText,
                    cv::Point(10, config_.height - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 255), 1);
    }

    void RouteVisualizer::createTransition(const Route &fromRoute, const Route &toRoute,
                                           const PointVector &points, const Bounds &bounds) {
        for (int frame = 0; frame < config_.transitionFrames; ++frame) {
            double progress = static_cast<double>(frame) / config_.transitionFrames;
            drawBackground();
            if (config_.showGrid) {
                drawGrid(bounds);
            }
            drawPoints(points, bounds);
            drawRoute(points, fromRoute, bounds, 1.0 - progress);
            drawRoute(points, toRoute, bounds, progress);

            videoWriter_.write(canvas_);
            if (showPreview_) {
                updatePreview();
            }
        }
    }

    void RouteVisualizer::validatePoints(const PointVector &points) {
        if (points.empty()) {
            throw std::invalid_argument("Points vector cannot be empty");
        }
    }

    void RouteVisualizer::initializeVideo() {
        if (!videoWriter_.isOpened()) {
            throw std::runtime_error("Video writer not properly initialized");
        }
    }

    void RouteVisualizer::resizeCanvasIfNeeded(int width, int height) {
        if (canvas_.rows != height || canvas_.cols != width) {
            canvas_ = cv::Mat(height, width, CV_8UC3);
        }
    }

};
