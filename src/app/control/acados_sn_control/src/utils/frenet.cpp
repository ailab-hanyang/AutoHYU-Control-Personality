// 24-10-01 Junhee Lee
// reference : https://github.dev/luator/boost_python_catkin_example

#include <iostream>
#include <vector>
#include <boost/python.hpp>
#include <cmath>
#include <limits>
#include <algorithm>
#include <memory>
#include <nanoflann.hpp>
#include <boost/noncopyable.hpp>
#include <omp.h> // OpenMP for parallel processing

using namespace std;
using namespace boost::python;
using namespace nanoflann;

// Template function to convert C++ vector to Python list
template <typename T>
boost::python::list vector_to_list(const std::vector<T>& vec) {
    boost::python::list list;
    for (const auto& item : vec) {
        list.append(item);
    }
    return list;
}

// Structure for Point Cloud used by nanoflann's k-d tree
struct PointCloud {
    std::vector<std::pair<double, double>> pts;

    // Required by nanoflann: Return the number of points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Required by nanoflann: Return the dim'th component of the idx'th point
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return pts[idx].first;
        else return pts[idx].second;
    }

    // Optional: Let nanoflann know that no bounding box is defined
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

// RoadMap class definition inheriting from boost::noncopyable to prevent copying
class RoadMap : boost::noncopyable {
public:
    // Road's x and y coordinates
    std::vector<double> cx, cy;
    // Accumulated distance s and segment distances ds
    std::vector<double> s, ds;
    double total_length;

    // Spline class definition for calculating splines
    class Spline {
    public:
        std::vector<double> x; // s values
        std::vector<double> a, b, c, d; // Spline coefficients
        size_t nx; // Number of points

        // Constructor: Calculate spline coefficients using Thomas algorithm (tridiagonal solver)
        // https://mathworld.wolfram.com/CubicSpline.html
        Spline(const std::vector<double>& s, const std::vector<double>& y) {
            x = s;
            nx = x.size();
            a = y;
            c.resize(nx, 0.0);
            b.resize(nx - 1, 0.0);
            d.resize(nx - 1, 0.0);

            std::vector<double> h(nx - 1);
            for (size_t i = 0; i < nx - 1; ++i) {
                h[i] = x[i + 1] - x[i];
            }

            // Tridiagonal matrix setup for natural spline
            std::vector<double> A_diag(nx, 0.0);
            std::vector<double> A_lower(nx - 1, 0.0);
            std::vector<double> A_upper(nx - 1, 0.0);
            std::vector<double> B_vec(nx, 0.0);

            // Natural spline boundary conditions
            A_diag[0] = 1.0;
            A_diag[nx - 1] = 1.0;

            for (size_t i = 1; i < nx - 1; ++i) {
                A_lower[i - 1] = h[i - 1];
                A_diag[i] = 2.0 * (h[i - 1] + h[i]);
                A_upper[i] = h[i];
                B_vec[i] = 3.0 * ((a[i + 1] - a[i]) / h[i] - (a[i] - a[i - 1]) / h[i - 1]);
            }

            // Forward sweep of Thomas algorithm
            for (size_t i = 1; i < nx; ++i) {
                double m = A_lower[i - 1] / A_diag[i - 1];
                A_diag[i] -= m * A_upper[i - 1];
                B_vec[i] -= m * B_vec[i - 1];
            }

            // Back substitution of Thomas algorithm
            c[nx - 1] = B_vec[nx - 1] / A_diag[nx - 1];
            for (int i = nx - 2; i >= 0; --i) {
                c[i] = (B_vec[i] - A_upper[i] * c[i + 1]) / A_diag[i];
            }

            // Compute b and d coefficients
            for (size_t i = 0; i < nx - 1; ++i) {
                b[i] = (a[i + 1] - a[i]) / h[i] - h[i] * (2.0 * c[i] + c[i + 1]) / 3.0;
                d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
            }
        }

        // Calculate position at s = t
        double calc(double t) const {
            size_t idx = find_segment(t);
            double dt = t - x[idx];
            return a[idx] + b[idx] * dt + c[idx] * dt * dt + d[idx] * dt * dt * dt;
        }

        // First derivative (velocity) at s = t
        double calcd(double t) const {
            size_t idx = find_segment(t);
            double dt = t - x[idx];
            return b[idx] + 2.0 * c[idx] * dt + 3.0 * d[idx] * dt * dt;
        }

        // Second derivative (acceleration) at s = t
        double calcdd(double t) const {
            size_t idx = find_segment(t);
            double dt = t - x[idx];
            return 2.0 * c[idx] + 6.0 * d[idx] * dt;
        }

    private:
        // Find the segment index for a given s = t
        size_t find_segment(double t) const {
            if (t <= x[0]) return 0;
            if (t >= x[nx - 1]) return nx - 2;
            return std::upper_bound(x.begin(), x.end(), t) - x.begin() - 1;
        }
    };

    // Unique pointers to Spline instances for x and y coordinates
    std::unique_ptr<Spline> sx;
    std::unique_ptr<Spline> sy;

    // PointCloud and k-d tree for efficient nearest neighbor search
    PointCloud cloud;
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<double, PointCloud>,
        PointCloud,
        2 /* dim */
    > waypoint_kd_tree_t;

    std::unique_ptr<waypoint_kd_tree_t> kd_tree;

    // Constructor: Initialize RoadMap with Python lists for x and y coordinates
    RoadMap(const boost::python::list& x, const boost::python::list& y) {
        cx = list_to_vector(x);
        cy = list_to_vector(y);
        s = calc_s(cx, cy);
        if (!s.empty()) {
            total_length = s.back();
        } else {
            total_length = 0.0;
        }

        // Initialize splines for x and y
        sx = std::make_unique<Spline>(s, cx);
        sy = std::make_unique<Spline>(s, cy);

        // Initialize PointCloud with x and y coordinates
        for (size_t i = 0; i < cx.size(); ++i) {
            cloud.pts.emplace_back(cx[i], cy[i]);
        }

        // Initialize and build k-d tree
        kd_tree = std::make_unique<waypoint_kd_tree_t>(2, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
        kd_tree->buildIndex();
    }

    // Destructor: unique_ptr automatically handles memory deallocation
    ~RoadMap() = default;

    // Calculate accumulated distance s and segment distances ds
    std::vector<double> calc_s(const std::vector<double>& x, const std::vector<double>& y) {
        std::vector<double> dx, dy, s_calc(1, 0.0);
        for (size_t i = 1; i < x.size(); ++i) {
            dx.push_back(x[i] - x[i - 1]);
            dy.push_back(y[i] - y[i - 1]);
        }

        ds.resize(dx.size());
        for (size_t i = 0; i < dx.size(); ++i) {
            ds[i] = hypot(dx[i], dy[i]);
            if (ds[i] < 1e-6) ds[i] = 1e-6; // Prevent very small distances
        }

        for (size_t i = 0; i < ds.size(); ++i) {
            s_calc.push_back(s_calc.back() + ds[i]);
        }
        return s_calc;
    }

    // Find the closest waypoint indices for given x and y coordinates using k-d tree
    boost::python::list ClosestWaypoint(const boost::python::list& x, const boost::python::list& y) const {
        std::vector<double> vec_x = list_to_vector(x);
        std::vector<double> vec_y = list_to_vector(y);
        std::vector<int> idx_closest(vec_x.size());

        // Parallelize the loop using OpenMP
        #pragma omp parallel for
        for (size_t i = 0; i < vec_x.size(); ++i) {
            double query_pt[2] = { vec_x[i], vec_y[i] };
            size_t ret_index;
            double out_dist_sqr;
            nanoflann::KNNResultSet<double> resultSet(1);
            resultSet.init(&ret_index, &out_dist_sqr);
            kd_tree->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParameters(10));
            idx_closest[i] = static_cast<int>(ret_index);
        }

        return vector_to_list(idx_closest);
    }

    // Apply cycling to s value based on total road length
    double FrenetSCycle(double s_val) const {
        return fmod(s_val, total_length);
    }

    // Calculate curvature at a given s value
    double GetCurvature(double s_val) const {
        double s_cycled = FrenetSCycle(s_val);
        double dx = sx->calcd(s_cycled);
        double ddx = sx->calcdd(s_cycled);
        double dy = sy->calcd(s_cycled);
        double ddy = sy->calcdd(s_cycled);
        double k = (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 1.5);
        return k;
    }

    // Calculate yaw angle at a given s value
    double GetYaw(double s_val) const {
        double s_cycled = FrenetSCycle(s_val);
        double dx = sx->calcd(s_cycled);
        double dy = sy->calcd(s_cycled);
        return atan2(dy, dx);
    }

    // Convert Cartesian coordinates to Frenet coordinates
    boost::python::tuple ToFrenet(const boost::python::list& x_p, const boost::python::list& y_p) {
        std::vector<double> x = list_to_vector(x_p);
        std::vector<double> y = list_to_vector(y_p);

        size_t n = x.size();
        std::vector<double> frenet_s(n, 0.0);
        std::vector<double> frenet_d(n, 0.0);

        // Initial estimate: closest waypoint
        boost::python::list idx_list = ClosestWaypoint(x_p, y_p);
        for (size_t i = 0; i < n; ++i) {
            int idx = extract<int>(idx_list[i]);
            frenet_s[i] = s[idx];
        }

        // Iteratively refine s values
        size_t max_iter = 100; // Reduced maximum iterations for efficiency
        double tol = 1e-2; // Reduced tolerance for faster convergence
        for (size_t j = 0; j < max_iter; ++j) {
            bool converged = true;
            #pragma omp parallel for reduction(&&:converged)
            for (size_t i = 0; i < n; ++i) {
                double s_val = FrenetSCycle(frenet_s[i]);
                double x0 = sx->calc(s_val);
                double y0 = sy->calc(s_val);
                double dx_val = x[i] - x0;
                double dy_val = y[i] - y0;
                double slope = GetYaw(s_val);
                double cos_slope = cos(slope);
                double sin_slope = sin(slope);
                double norm_distance = dx_val * cos_slope + dy_val * sin_slope;
                frenet_s[i] += norm_distance;

                // Apply cycling to s
                frenet_s[i] = FrenetSCycle(frenet_s[i]);

                if (fabs(norm_distance) >= tol) {
                    converged = false;
                }
            }
            if (converged) {
                break;
            }
        }

        // Calculate d values
        #pragma omp parallel for
        for (size_t i = 0; i < n; ++i) {
            double s_val = FrenetSCycle(frenet_s[i]);
            double x0 = sx->calc(s_val);
            double y0 = sy->calc(s_val);
            double dx_val = x[i] - x0;
            double dy_val = y[i] - y0;
            double slope = GetYaw(s_val);
            double cos_slope = cos(slope);
            double sin_slope = sin(slope);
            frenet_d[i] = -dx_val * sin_slope + dy_val * cos_slope;
        }

        // Return Frenet coordinates as a Python tuple
        return boost::python::make_tuple(vector_to_list(frenet_s), vector_to_list(frenet_d));
    }

    // Convert Frenet coordinates to Cartesian coordinates
    boost::python::tuple ToCartesian(const boost::python::list& s_p, const boost::python::list& d_p) const {
        std::vector<double> s_vals = list_to_vector(s_p);
        std::vector<double> d_vals = list_to_vector(d_p);
        size_t n = s_vals.size();
        std::vector<double> x(n, 0.0);
        std::vector<double> y(n, 0.0);

        #pragma omp parallel for
        for (size_t i = 0; i < n; ++i) {
            double s_val = FrenetSCycle(s_vals[i]);
            double x0 = sx->calc(s_val);
            double y0 = sy->calc(s_val);
            double slope = GetYaw(s_val);
            double cos_slope = cos(slope);
            double sin_slope = sin(slope);
            x[i] = x0 + d_vals[i] * cos_slope;
            y[i] = y0 + d_vals[i] * sin_slope;
        }

        // Return Cartesian coordinates as a Python tuple
        return boost::python::make_tuple(vector_to_list(x), vector_to_list(y));
    }

    // Convert Frenet velocities to Cartesian velocities
    boost::python::tuple ToCartesianVelocity(const boost::python::list& vs_p, const boost::python::list& vd_p, const boost::python::list& s_p) const {
        std::vector<double> vs = list_to_vector(vs_p);
        std::vector<double> vd = list_to_vector(vd_p);
        std::vector<double> s_vals = list_to_vector(s_p);
        size_t n = vs.size();
        std::vector<double> vx(n, 0.0);
        std::vector<double> vy(n, 0.0);

        #pragma omp parallel for
        for (size_t i = 0; i < n; ++i) {
            double slope = GetYaw(FrenetSCycle(s_vals[i]));
            double cos_slope = cos(slope);
            double sin_slope = sin(slope);
            vx[i] = vs[i] * cos_slope - vd[i] * sin_slope;
            vy[i] = vs[i] * sin_slope + vd[i] * cos_slope;
        }

        // Return Cartesian velocities as a Python tuple
        return boost::python::make_tuple(vector_to_list(vx), vector_to_list(vy));
    }

private:
    // Convert Python list to C++ vector
    std::vector<double> list_to_vector(const boost::python::list& list) const {
        std::vector<double> vec(len(list));
        for (size_t i = 0; i < vec.size(); ++i) {
            vec[i] = extract<double>(list[i]);
        }
        return vec;
    }
};

// Define the Boost.Python module
BOOST_PYTHON_MODULE(libfrenet3)
{
    class_<RoadMap, boost::noncopyable>("RoadMap", init<list, list>())
        .def("ClosestWaypoint", &RoadMap::ClosestWaypoint)
        .def("FrenetSCycle", &RoadMap::FrenetSCycle)
        .def("GetCurvature", &RoadMap::GetCurvature)
        .def("GetYaw", &RoadMap::GetYaw)
        .def("ToFrenet", &RoadMap::ToFrenet)
        .def("ToCartesian", &RoadMap::ToCartesian)
        .def("ToCartesianVelocity", &RoadMap::ToCartesianVelocity);
}
