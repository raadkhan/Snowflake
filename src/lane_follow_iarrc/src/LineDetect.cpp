/*
 * Created by: Raad Khan
 * Created On: July 1, 2017
 * Description: Detects lane lines and generates destination point.
 * Usage: LaneFollow node uses this as a helper class.
 */

#include "LineDetect.h"

#include <Eigen/QR>

using namespace cv;

LineDetect::LineDetect() : white(255), vertical_slices(10), degree(3) {}

cv::Point2d LineDetect::getLaneIntersectPoint(std::vector<Polynomial> lane_lines,
                                              int order) {
    Polynomial LeftLanePolynomial  = lane_lines[0];
    Polynomial RightLanePolynomial = lane_lines[1];
    Polynomial IntersectPolynomial = Polynomial();

    // create the intersect polynomial
    for (int i = 0; i <= order; i++) {

        IntersectPolynomial.coefficients.emplace_back(
        RightLanePolynomial.coefficients[i] -
        LeftLanePolynomial.coefficients[i]);
    }

    // initialize the intersect polynomial coefficients matrix
    cv::Mat intersect_coefficients(1, order + 1, CV_64F, Scalar::all(0));

    memcpy(intersect_coefficients.data,
           IntersectPolynomial.coefficients.data(),
           IntersectPolynomial.coefficients.size() * sizeof(double));

    // initialize the intersect polynomial roots matrix
    // a nice solution will contain 3 roots each with an x channel and
    // a y channel
    // i.e: intersect_roots =
    //      [x1, y1;
    //       x2, y2;
    //       x3, y3]
    // where one of these roots will be the intersect root
    cv::Mat intersect_roots(3, 1, CV_64F, Scalar::all(0));

    // solve for the intersect polynomial roots
    cv::solvePoly(intersect_coefficients, intersect_roots);

    int max_row = intersect_roots.rows - 1; // we expect there to be 3 rows
    int max_col = intersect_roots.cols - 1; // we expect there to be 1 col

    std::vector<double> possible_x_intersects;

    for (int i = 0; i <= max_row; i++) {

        // find the real roots knowing its y components are very close to 0
        if ((intersect_roots.at<cv::Vec2d>(i, max_col)[1] >= -1e-10) &&
            (intersect_roots.at<cv::Vec2d>(i, max_col)[1] <= +1e-10)) {

            // store x components of the possible intersect roots
            possible_x_intersects.emplace_back(
            intersect_roots.at<double>(i, max_col));
        }
    }

    double x_intersect;
    double y_intersect;

    // throw an exception if no intersect roots exist
    if (possible_x_intersects.size() == 1 && possible_x_intersects[0] == 0) {

        std::cout << "intersect_roots = " << std::endl
                  << intersect_roots << std::endl
                  << std::endl;

        throw LineDetect::NoLaneIntersectRoots;
    }

    // there exists one unique intersect root
    else if (possible_x_intersects.size() == 1) {

        // set x component of the intersect root
        x_intersect = possible_x_intersects[0];

        y_intersect = 0;

        // solve the right/left lane polynomial given the x intersect and
        // sum up powered terms
        for (int i = order; i > 0; i--) {

            y_intersect +=
            (RightLanePolynomial.coefficients[i] * pow(x_intersect, i));
        }

        // add powered terms with last term to get the y intersect
        y_intersect += RightLanePolynomial.coefficients[0];

    }

    // there exists more than one intersect root
    else {
        // attempt to find the correct intersect root
        for (double possible_x_intersect : possible_x_intersects) {

            x_intersect = possible_x_intersect;
            y_intersect = 0;

            // set x value of the right/left lane polynomial to the x intersect
            // and sum up powered terms
            for (int j = order; j > 0; j--) {

                y_intersect +=
                (RightLanePolynomial.coefficients[j] * pow(x_intersect, j));
            }

            // add powered terms with last term to get the y intersect
            y_intersect += RightLanePolynomial.coefficients[0];

            // the correct y intersect is positive
            if (y_intersect > 0) { goto create_point; }
        }

        // otherwise throw an exception since no valid intersect root exists
        std::cout << "intersect_roots = " << std::endl
                  << intersect_roots << std::endl
                  << std::endl;
        throw LineDetect::NoLaneIntersectRoots;
    }

create_point:
    // create a Point2d storing the intersect in the ROS coordinate frame
    cv::Point2d lane_intersect_point = {y_intersect, -x_intersect};

    return lane_intersect_point;
}

std::vector<Polynomial>
LineDetect::getLaneLines(std::vector<std::vector<cv::Point2d>> lane_points) {
    Polynomial poly_line;

    // contains left and right lane polynomials
    std::vector<Polynomial> lane_lines;

    for (std::vector<cv::Point2d> const &points : lane_points) {

        poly_line = this->fitPolyToLine(points, degree);
        lane_lines.emplace_back(poly_line);
    }

    return lane_lines;
}

Polynomial LineDetect::fitPolyToLine(std::vector<cv::Point2d> points,
                                     int order) {
    Polynomial PolyLine = Polynomial();

    assert(points.size() >= order + 1);

    std::vector<double> xv(points.size(), 0);
    std::vector<double> yv(points.size(), 0);

    for (size_t i = 0; i < points.size(); i++) {

        xv[i] = points[i].x;
        yv[i] = points[i].y;
    }

    Eigen::MatrixXd A(xv.size(), order + 1);
    Eigen::VectorXd yvMapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
    Eigen::VectorXd result;

    // create matrix
    for (size_t i = 0; i < points.size(); i++)

        for (size_t j = 0; j <= order; j++)

            A(i, j) = pow((xv.at(i)), j);

    // solve for linear least squares fit to get the coefficients
    result = A.householderQr().solve(yvMapped);

    for (size_t i = 0; i < result.size(); i++) {
        // and emplace them back to the coefficients array
        // from the lowest to highest order term
        PolyLine.coefficients.emplace_back(result[i]);
    }

    return PolyLine;
}

std::vector<std::vector<cv::Point2d>>
LineDetect::getLanePoints(cv::Mat& filtered_image, int min_left_peak, int min_right_peak) {
    std::vector<Window> base_windows =
    this->getBaseWindows(filtered_image, min_left_peak, min_right_peak);

    // contains left and right lane points
    std::vector<std::vector<cv::Point2d>> lane_points(
    base_windows.size(), std::vector<cv::Point2d>());

    // the leftmost/rightmost a window's center can be within the image
    std::pair<int, int> window_bounds(window_width,
                                      filtered_image.cols - window_width);


    // iterate through window slices bottom up
    for (int window_slice_index = 0; window_slice_index < vertical_slices;
         window_slice_index++) {

        // iterate through base windows left to right
        for (int lor_window_index = 0; lor_window_index < base_windows.size();
             lor_window_index++) {

            // obtain the left or right base window
            Window BaseWindow = base_windows[lor_window_index];

            // obtain the window slice
            cv::Mat window_slice = this->getWindowSlice(filtered_image,
                                                        BaseWindow,
                                                        window_slice_index);

            // obtain the window histogram
            int_vec window_histogram = this->getHistogram(window_slice);

            // obtain the window histogram's peak
            Peak WindowPeak = this->getWindowHistogramPeaks(window_histogram);

            // shift the base window's center to align with peak position
            BaseWindow.center += WindowPeak.position - window_slice.cols / 2;

            // throw an exception if base window's center is shifted
            // such that the base window goes out of the image's bounds
            if (BaseWindow.center <= window_bounds.first ||
                BaseWindow.center >= window_bounds.second) {

                std::cout << "window peak position (relative) = "
                          << WindowPeak.position << std::endl
                          << "base window center (absolute) = "
                          << BaseWindow.center << std::endl;

                throw LineDetect::BaseWindowsOutOfBounds;
            }

            // get y value of the lane point
            int y =
            (window_slice_index * filtered_image.rows / vertical_slices) +
            window_slice.rows / 2;

            // create the lane point
            cv::Point2d point{(double) BaseWindow.center, (double) y};

            // and store it
            lane_points[lor_window_index].push_back(point);
        }
    }

    return lane_points;
}

std::vector<Window> LineDetect::getBaseWindows(cv::Mat& filtered_image,
                                               int min_left_peak,
                                               int min_right_peak) {
    window_width = filtered_image.cols / 6;

    int_vec base_histogram = this->getHistogram(filtered_image);

    std::pair<Peak, Peak> BasePeaks =
            this->getBaseHistogramPeaks(base_histogram,
                                        filtered_image.cols,
                                        min_left_peak,
                                        min_right_peak);

    // contains left and right base windows
    std::vector<Window> base_windows;

    Window LeftWindow{BasePeaks.first.value, window_width};
    Window RightWindow{BasePeaks.second.value, window_width};

    base_windows.emplace_back(LeftWindow);
    base_windows.emplace_back(RightWindow);

    return base_windows;
}

int_vec LineDetect::getHistogram(cv::Mat &ROI) {
    int_vec histogram(ROI.cols, 0);

    // counts number of white pixels in each column
    for (int i = 0; i < ROI.rows; i++) {

        for (int j = 0; j < ROI.cols; j++) {

            int pixel_value = ROI.at<uchar>(i, j);

            if (pixel_value == white) { histogram[j]++; }
        }
    }

    return histogram;
}

std::pair<Peak, Peak>
LineDetect::getBaseHistogramPeaks(int_vec base_histogram,
                                  int image_width,
                                  int min_left_peak,
                                  int min_right_peak) {
    // contains left and right base peaks
    std::pair<Peak, Peak> BasePeaks;

    int peak_values = 0;

    // finds left base peak
    for (int i = 0; i < (base_histogram.size() / 2); i++) {

        if (base_histogram[i] > peak_values) {
            peak_values = base_histogram[i];
            BasePeaks.first.position = i;
            BasePeaks.first.value = peak_values;
        }
    }

    // throw an exception if left base peak position is too close
    // to the left edge of image to create a left base window later
    // or if left base peak is not high enough to be considered a
    // left lane line
    if ( BasePeaks.first.position <= window_width ||
         BasePeaks.first.value < min_left_peak) {

        std::cout << "left peak position = "
                  << BasePeaks.first.position << std::endl
                  << "left peak value = "
                  << BasePeaks.first.value << std::endl;

        throw LineDetect::NoBasePeaks;
    }

    else {

        peak_values = 0;

        // finds right base peak
        for (auto i = (int) (base_histogram.size() / 2);
             i < base_histogram.size(); i++) {

            if (base_histogram[i] > peak_values) {

                peak_values  = base_histogram[i];
                BasePeaks.second.position = i;
                BasePeaks.second.value = peak_values;
            }
        }

        // throw an exception if right base peak position is too close
        // to the right edge of image to create a right base window later
        // or if right base peak is not high enough to be considered a
        // right lane line
        if (BasePeaks.second.position >= (image_width - window_width) ||
            BasePeaks.second.value < min_right_peak) {

            std::cout << "right peak position = "
                      << BasePeaks.second.position << std::endl
                      << "right peak value = "
                      << BasePeaks.second.value << std::endl;

            throw LineDetect::NoBasePeaks;
        }

        else { return BasePeaks; }
    }
}

cv::Mat LineDetect::getWindowSlice(cv::Mat &filtered_image,
                                   Window BaseWindow,
                                   int vertical_slice_index) {
    // create window slice
    cv::Mat window_slice = filtered_image(

    Range(vertical_slice_index * filtered_image.rows / vertical_slices,
          (vertical_slice_index + 1) * filtered_image.rows / vertical_slices),

    Range(BaseWindow.getLeftSide(), BaseWindow.getRightSide()));

    return window_slice;
}

Peak LineDetect::getWindowHistogramPeaks(int_vec window_histogram) {
    Peak WindowPeak{};
    int peak_value = 0;

    for (int i = 0; i < window_histogram.size(); i++) {

        if (window_histogram[i] > peak_value) {

            peak_value = window_histogram[i];
            WindowPeak.position = i;
            WindowPeak.value = peak_value;
        }
    }

    return WindowPeak;
}