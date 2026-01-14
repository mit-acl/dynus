#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <future>
#include <tuple>
#include <type_traits>
#include <thread>
#include <cctype>

#include "timer.hpp"
#include "dgp/termcolor.hpp"
#include "mighty/mighty_type.hpp"
#include <mighty/utils.hpp>
#include "dgp/dgp_manager.hpp"
#include <mighty/gurobi_solver.hpp>
#include <decomp_rviz_plugins/data_ros_utils.hpp>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace fs = std::filesystem;
using namespace std::chrono;

using namespace mighty;

using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Matrix<double, 3, 1>;

struct BenchResult
{
    std::string file;
    bool success{false};
    bool gurobi_error{false};

    double per_opt_runtime_ms{0.0};   // from GRB Runtime * 1000
    double total_opt_runtime_ms{0.0}; // end-to-end wall time for this case
    double total_traj_time_sec{0.0};
    std::string status;

    Vec3d start{0, 0, 0};
    Vec3d goal{0, 0, 0};

    double factor_used{0.0};
    double cost_value{-100000.0};

    // -------- Constraint violation metrics (sampled at dt=dc) --------
    // Safe corridor (union of polytopes): for each sample, compute min_i max(A_i p - b_i).
    // max over samples of that value is reported here. <= 0 means "inside at least one polytope".
    double corridor_max_min_violation{0.0}; // max_t min_poly max(Ap-b) (positive means outside corridor union)
    double corridor_t_at_max{0.0};
    Vec3d corridor_p_at_max{0, 0, 0};
    int corridor_best_poly_idx_at_max{-1};

    // Dynamic constraints
    double v_max_observed{0.0};
    double v_max_excess{0.0}; // max( ||v|| - v_max ) (axis-wise in current code)
    double v_t_at_max{0.0};

    double a_max_observed{0.0};
    double a_max_excess{0.0}; // max( ||a|| - a_max )
    double a_t_at_max{0.0};

    double j_max_observed{0.0};
    double j_max_excess{0.0}; // max( ||j|| - j_max )
    double j_t_at_max{0.0};

    // Convenience flags (derived from the above using tolerances)
    bool corridor_violated{false};
    bool v_violated{false};
    bool a_violated{false};
    bool j_violated{false};

    // -------- Smoothness metrics (sampled at dt=dc; jerk via finite-diff of accel) --------
    // S_jerk    = ∫_0^T ||j(t)|| dt
    // Sbar_jerk = sqrt( (1/T) ∫_0^T ||j(t)||^2 dt )   (RMS jerk; time-normalized)
    double jerk_smoothness_l1{0.0};
    double jerk_rms{0.0};

    double traj_length_m{0.0}; // ∫ ||v(t)|| dt

    // Visualization payloads (cached)
    decomp_ros_msgs::msg::PolyhedronArray poly_msg;
    visualization_msgs::msg::MarkerArray opt_traj_ma;    // committed colored
    visualization_msgs::msg::MarkerArray global_path_ma; // marker array path
};

// ------------------------ helpers ------------------------

static void writeU32(std::ofstream &ofs, uint32_t v)
{
    ofs.write(reinterpret_cast<const char *>(&v), sizeof(v));
}
static void writeD(std::ofstream &ofs, double v)
{
    ofs.write(reinterpret_cast<const char *>(&v), sizeof(v));
}
static uint32_t readU32(std::ifstream &ifs)
{
    uint32_t v;
    ifs.read(reinterpret_cast<char *>(&v), sizeof(v));
    if (!ifs)
        throw std::runtime_error("Corrupt .mysco2 (u32).");
    return v;
}
static double readD(std::ifstream &ifs)
{
    double v;
    ifs.read(reinterpret_cast<char *>(&v), sizeof(v));
    if (!ifs)
        throw std::runtime_error("Corrupt .mysco2 (double).");
    return v;
}

static void fillStateFromPos(state &s, const Vec3d &p)
{
    s.pos = p;
    s.vel = Eigen::Vector3d::Zero();
    s.accel = Eigen::Vector3d::Zero();
}

// Build Polyhedron<3> from A x <= b
static Polyhedron<3> polyFromHalfspacesSeeded(
    Eigen::MatrixXd A,
    Eigen::VectorXd b,
    const Vec3f &seed,
    double eps,
    bool *did_flip_any = nullptr)
{
    if (did_flip_any)
        *did_flip_any = false;

    // Ensure seed satisfies all inequalities by flipping rows that violate it.
    for (int r = 0; r < A.rows(); ++r)
    {
        const double v = A.row(r).dot(seed) - b(r);
        if (v > eps)
        {
            A.row(r) *= -1.0;
            b(r) *= -1.0;
            if (did_flip_any)
                *did_flip_any = true;
        }
    }

    Polyhedron<3> poly;
    for (int r = 0; r < A.rows(); ++r)
    {
        Vec3f a = A.row(r).transpose();
        const double norm = a.norm();
        if (norm < 1e-12)
            continue;

        // Inequality: a^T x <= b
        const Vec3f n = a / norm;
        const double d = b(r) / norm;
        const Vec3f p0 = n * d;

        // Hyperplane(point, normal)
        poly.add(Hyperplane<3>(p0, n));
    }

    return poly;
}

// Load one .mysco2 file and reconstruct:
// - path (vec_Vecf<3>)
// - seg_end_times
// - poly_out (vec_E<Polyhedron<3>>)
// - l_constraints (std::vector<LinearConstraint3D>)
static void loadMysco2(
    const fs::path &file,
    Vec3d &start,
    Vec3d &goal,
    vec_Vecf<3> &path,
    std::vector<double> &seg_end_times,
    vec_E<Polyhedron<3>> &poly_out,
    std::vector<LinearConstraint3D> &l_constraints,
    double poly_seed_eps, bool debug_poly_check)
{
    std::ifstream ifs(file, std::ios::binary);
    if (!ifs)
        throw std::runtime_error("Failed to open: " + file.string());

    char magic[8];
    ifs.read(magic, 8);
    if (!ifs)
        throw std::runtime_error("Corrupt .mysco2 (magic): " + file.string());

    const std::string m(magic, magic + 8);
    if (m.rfind("MYSCO2", 0) != 0)
        throw std::runtime_error("Bad magic in: " + file.string());

    const uint32_t version = readU32(ifs);
    if (version != 1)
        throw std::runtime_error("Unsupported .mysco2 version in: " + file.string());

    const double a0 = readD(ifs);
    const double a1 = readD(ifs);
    const double a2 = readD(ifs);

    const double g0 = readD(ifs);
    const double g1 = readD(ifs);
    const double g2 = readD(ifs);

    start = Vec3d(a0, a1, a2);
    goal = Vec3d(g0, g1, g2);

    const uint32_t num_path_pts = readU32(ifs);
    path.clear();
    path.reserve(num_path_pts);
    for (uint32_t i = 0; i < num_path_pts; ++i)
    {
        Vec3f p;
        const double px0 = readD(ifs);
        const double px1 = readD(ifs);
        const double px2 = readD(ifs);
        p << px0, px1, px2;
        path.push_back(p);
    }

    const uint32_t num_seg = readU32(ifs);
    seg_end_times.resize(num_seg);
    for (uint32_t i = 0; i < num_seg; ++i)
        seg_end_times[i] = readD(ifs);

    if (path.size() < 2 || (path.size() - 1) != num_seg)
        throw std::runtime_error("File inconsistent: path.size()-1 != num_seg in " + file.string());

    poly_out.clear();
    poly_out.resize(num_seg);

    l_constraints.clear();
    l_constraints.resize(num_seg);

    for (uint32_t si = 0; si < num_seg; ++si)
    {
        const uint32_t mplanes = readU32(ifs);

        Eigen::MatrixXd A(mplanes, 3);
        Eigen::VectorXd b(mplanes);

        for (uint32_t r = 0; r < mplanes; ++r)
            for (int c = 0; c < 3; ++c)
                A(r, c) = readD(ifs);

        for (uint32_t r = 0; r < mplanes; ++r)
            b(r) = readD(ifs);

        const Vec3f pt_inside = (path[si] + path[si + 1]) / 2.0;

        // Make seed-consistent halfspaces
        Eigen::MatrixXd A_fix = A;
        Eigen::VectorXd b_fix = b;
        for (int r = 0; r < A_fix.rows(); ++r)
        {
            const double v = A_fix.row(r).dot(pt_inside) - b_fix(r);
            if (v > poly_seed_eps)
            {
                A_fix.row(r) *= -1.0;
                b_fix(r) *= -1.0;
            }
        }

        poly_out[si] = polyFromHalfspacesSeeded(A_fix, b_fix, pt_inside, poly_seed_eps, nullptr);

        LinearConstraint3D lc;
        lc.A_ = A_fix;
        lc.b_ = b_fix;
        l_constraints[si] = lc;

        const double max_v = (lc.A_ * pt_inside - lc.b_).maxCoeff();
        if (max_v > 1e-4)
        {
            std::cerr << "[SFC LOAD] WARNING seg=" << si
                      << " seed violates A_fix x<=b_fix by " << max_v
                      << " file=" << file.string() << std::endl;
        }
    }
}

// ------------------------ constraint checks (sample at dt=dc) ------------------------

template <typename T, typename = void>
struct has_member_pos : std::false_type
{
};
template <typename T>
struct has_member_pos<T, std::void_t<decltype(std::declval<const T &>().pos)>> : std::true_type
{
};

template <typename T, typename = void>
struct has_member_vel : std::false_type
{
};
template <typename T>
struct has_member_vel<T, std::void_t<decltype(std::declval<const T &>().vel)>> : std::true_type
{
};

template <typename T, typename = void>
struct has_member_accel : std::false_type
{
};
template <typename T>
struct has_member_accel<T, std::void_t<decltype(std::declval<const T &>().accel)>> : std::true_type
{
};

static inline Vec3d getPosSafe(const state &s)
{
    if constexpr (has_member_pos<state>::value)
        return s.pos;
    return Vec3d::Zero();
}

static inline Vec3d getVelSafe(const state &s)
{
    if constexpr (has_member_vel<state>::value)
        return s.vel;
    return Vec3d::Zero();
}

static inline Vec3d getAccelSafe(const state &s)
{
    if constexpr (has_member_accel<state>::value)
        return s.accel;
    return Vec3d::Zero();
}

static inline double maxHalfspaceViolation(const LinearConstraint3D &lc, const Vec3f &p)
{
    if (lc.A_.rows() == 0)
        return 0.0;
    Eigen::VectorXd d = lc.A_ * p - lc.b_;
    return d.maxCoeff();
}

struct ConstraintReport
{
    double corridor_max_min_violation{0.0};
    double corridor_t_at_max{0.0};
    Vec3d corridor_p_at_max{0, 0, 0};
    int corridor_best_poly_idx_at_max{-1};

    double v_max_observed{0.0};
    double v_max_excess{0.0};
    double v_t_at_max{0.0};

    double a_max_observed{0.0};
    double a_max_excess{0.0};
    double a_t_at_max{0.0};

    double j_max_observed{0.0};
    double j_max_excess{0.0};
    double j_t_at_max{0.0};

    bool corridor_violated{false};
    bool v_violated{false};
    bool a_violated{false};
    bool j_violated{false};

    // -------- Smoothness metrics (sampled at dt=dc; jerk via finite-diff of accel) --------
    // S_jerk    = ∫_0^T ||j(t)|| dt
    // Sbar_jerk = sqrt( (1/T) ∫_0^T ||j(t)||^2 dt )   (RMS jerk; time-normalized)
    double jerk_smoothness_l1{0.0};
    double jerk_rms{0.0};

    double traj_length_m{0.0}; // ∫ ||v(t)|| dt
};

static ConstraintReport analyzeConstraintsSampled(
    const std::vector<state> &samples,
    const std::vector<LinearConstraint3D> &l_constraints,
    double dc,
    double v_max, double a_max, double j_max,
    double corridor_tol = 1e-6,
    double dyn_tol = 1e-6)
{
    ConstraintReport rep;

    if (samples.empty())
        return rep;

    const int num_poly = static_cast<int>(l_constraints.size());
    const bool have_polys = (num_poly > 0);

    auto maxAbsComponent = [](const Vec3d &x) -> double
    {
        return x.cwiseAbs().maxCoeff();
    };

    // Corridor union
    if (have_polys)
    {
        double worst = -std::numeric_limits<double>::infinity();
        double worst_t = 0.0;
        Vec3d worst_p(0, 0, 0);
        int worst_best_idx = -1;

        for (size_t k = 0; k < samples.size(); ++k)
        {
            const double t = static_cast<double>(k) * dc;
            const Vec3d p_d = getPosSafe(samples[k]);
            const Vec3f p = p_d.cast<double>();

            double best = std::numeric_limits<double>::infinity();
            int best_idx = -1;

            for (int i = 0; i < num_poly; ++i)
            {
                const double v = maxHalfspaceViolation(l_constraints[i], p);
                if (v < best)
                {
                    best = v;
                    best_idx = i;
                }
            }

            if (best > worst)
            {
                worst = best;
                worst_t = t;
                worst_p = p_d;
                worst_best_idx = best_idx;
            }
        }

        rep.corridor_max_min_violation = worst;
        rep.corridor_t_at_max = worst_t;
        rep.corridor_p_at_max = worst_p;
        rep.corridor_best_poly_idx_at_max = worst_best_idx;
        rep.corridor_violated = (worst > corridor_tol);
    }

    // Build v,a,j
    std::vector<Vec3d> vel(samples.size(), Vec3d::Zero());
    std::vector<Vec3d> acc(samples.size(), Vec3d::Zero());
    std::vector<Vec3d> jerk(samples.size(), Vec3d::Zero());

    if constexpr (has_member_vel<state>::value)
    {
        for (size_t k = 0; k < samples.size(); ++k)
            vel[k] = samples[k].vel;
    }
    else
    {
        if (samples.size() >= 2 && dc > 0.0)
        {
            vel[0] = Vec3d::Zero();
            for (size_t k = 1; k < samples.size(); ++k)
                vel[k] = (getPosSafe(samples[k]) - getPosSafe(samples[k - 1])) / dc;
        }
    }

    double length_m = 0.0;

    if (samples.size() >= 2 && dc > 0.0)
    {
        // If you have velocity vectors vel[k]
        for (size_t k = 1; k < samples.size(); ++k)
        {
            const double s0 = vel[k - 1].norm();
            const double s1 = vel[k].norm();
            length_m += 0.5 * (s0 + s1) * dc; // trapezoidal rule
        }
    }

    // store
    rep.traj_length_m = length_m;

    if constexpr (has_member_accel<state>::value)
    {
        for (size_t k = 0; k < samples.size(); ++k)
            acc[k] = samples[k].accel;
    }
    else
    {
        if (samples.size() >= 2 && dc > 0.0)
        {
            acc[0] = Vec3d::Zero();
            for (size_t k = 1; k < samples.size(); ++k)
                acc[k] = (vel[k] - vel[k - 1]) / dc;
        }
    }

    if constexpr (has_member_accel<state>::value == false)
    {
        for (size_t k = 1; k < samples.size(); ++k)
            jerk[k] = samples[k].jerk;
    }
    else
    {
        if (samples.size() >= 2 && dc > 0.0)
        {
            jerk[0] = Vec3d::Zero();
            for (size_t k = 1; k < samples.size(); ++k)
                jerk[k] = (acc[k] - acc[k - 1]) / dc;
        }
    }

    constexpr double buf = 1e-6;

    double vmax_obs = 0.0, vmax_ex = -std::numeric_limits<double>::infinity(), vmax_t = 0.0;
    double amax_obs = 0.0, amax_ex = -std::numeric_limits<double>::infinity(), amax_t = 0.0;
    double jmax_obs = 0.0, jmax_ex = -std::numeric_limits<double>::infinity(), jmax_t = 0.0;

    for (size_t k = 0; k < samples.size(); ++k)
    {
        const double t = static_cast<double>(k) * dc;

        const double vcomp_max = maxAbsComponent(vel[k]);
        vmax_obs = std::max(vmax_obs, vcomp_max);
        const double vex = vcomp_max - (v_max + buf);
        if (vex > vmax_ex)
        {
            vmax_ex = vex;
            vmax_t = t;
        }

        const double acomp_max = maxAbsComponent(acc[k]);
        amax_obs = std::max(amax_obs, acomp_max);
        const double aex = acomp_max - (a_max + buf);
        if (aex > amax_ex)
        {
            amax_ex = aex;
            amax_t = t;
        }

        const double jcomp_max = maxAbsComponent(jerk[k]);
        jmax_obs = std::max(jmax_obs, jcomp_max);
        const double jex = jcomp_max - (j_max + buf);
        if (jex > jmax_ex)
        {
            jmax_ex = jex;
            jmax_t = t;
        }
    }

    rep.v_max_observed = vmax_obs;
    rep.v_max_excess = std::max(0.0, vmax_ex);
    rep.v_t_at_max = vmax_t;
    rep.v_violated = (vmax_ex > dyn_tol);

    rep.a_max_observed = amax_obs;
    rep.a_max_excess = std::max(0.0, amax_ex);
    rep.a_t_at_max = amax_t;
    rep.a_violated = (amax_ex > dyn_tol);

    rep.j_max_observed = jmax_obs;
    rep.j_max_excess = std::max(0.0, jmax_ex);
    rep.j_t_at_max = jmax_t;
    rep.j_violated = (jmax_ex > dyn_tol);

    double jerk_l1 = 0.0;     // ∫ ||j|| dt   (approx)
    double jerk_l2_int = 0.0; // ∫ ||j||^2 dt (approx)

    if (samples.size() >= 2 && dc > 0.0)
    {
        for (size_t k = 1; k < samples.size(); ++k)
        {
            // Smoothness metrics use Euclidean norm.
            const double jnorm = jerk[k].norm();
            jerk_l1 += jnorm * dc;
            jerk_l2_int += (jnorm * jnorm) * dc;
        }
    }

    // Smoothness results.
    // Use the sampled trajectory duration implied by samples + dc for numerical consistency.
    const double T = (samples.size() >= 2 && dc > 0.0) ? (static_cast<double>(samples.size() - 1) * dc) : 0.0;
    rep.jerk_smoothness_l1 = jerk_l1;
    rep.jerk_rms = (T > 0.0) ? std::sqrt(jerk_l2_int / T) : 0.0;

    return rep;
}

// ------------------------ trajectory dump helpers (NEW) ------------------------

static inline bool ensureDir(const fs::path &p)
{
    std::error_code ec;
    if (fs::exists(p, ec))
        return fs::is_directory(p, ec);
    return fs::create_directories(p, ec);
}

static inline std::string sanitizeFilename(std::string s)
{
    for (char &c : s)
    {
        const unsigned char uc = static_cast<unsigned char>(c);
        if (!(std::isalnum(uc) || c == '-' || c == '_' || c == '.'))
            c = '_';
    }
    return s;
}

// Dump goal_setpoints (sampled at dt=dc) to CSV.
// If traj_dump_dt > dc, downsample by stride = round(traj_dump_dt/dc), clamped >= 1.
static void dumpTrajectoryCsvV1(
    const fs::path &out_csv,
    const std::string &planner_name,
    const std::string &case_file_basename,
    const std::string &frame_id,
    const std::vector<state> &samples,
    double dc,
    double traj_dump_dt_requested,
    double factor_used,
    double cost_value,
    double total_traj_time_sec)
{
    if (samples.empty())
        return;

    if (dc <= 0.0)
        dc = 0.01;

    int stride = 1;
    if (traj_dump_dt_requested > 0.0)
    {
        const double ratio = traj_dump_dt_requested / dc;
        const long long s = llround(ratio);
        stride = static_cast<int>(std::max<long long>(1LL, s));
    }
    const double dump_dt = stride * dc;

    std::ofstream ofs(out_csv);
    if (!ofs)
        throw std::runtime_error("Failed to open traj csv for write: " + out_csv.string());

    ofs << "# traj_format: mighty_local_traj_csv_v1\n";
    ofs << "# planner_name: " << planner_name << "\n";
    ofs << "# case_file: " << case_file_basename << "\n";
    ofs << "# frame_id: " << frame_id << "\n";
    ofs << "# dc_sec: " << std::fixed << std::setprecision(9) << dc << "\n";
    ofs << "# dump_dt_sec: " << std::fixed << std::setprecision(9) << dump_dt << "\n";
    ofs << "# factor_used: " << std::fixed << std::setprecision(9) << factor_used << "\n";
    ofs << "# cost_value: " << std::fixed << std::setprecision(9) << cost_value << "\n";
    ofs << "# total_traj_time_sec: " << std::fixed << std::setprecision(9) << total_traj_time_sec << "\n";
    ofs << "t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz\n";

    ofs << std::fixed << std::setprecision(9);

    Vec3d prev_a = getAccelSafe(samples.front());
    for (size_t k = 0; k < samples.size(); k += static_cast<size_t>(stride))
    {
        const double t = static_cast<double>(k) * dc;

        const Vec3d p = getPosSafe(samples[k]);
        const Vec3d v = getVelSafe(samples[k]);
        const Vec3d a = getAccelSafe(samples[k]);

        Vec3d j = Vec3d::Zero();
        if (k > 0 && dc > 0.0)
            j = (a - prev_a) / dc;

        // Update prev_a for next jerk computation (even if downsampling)
        prev_a = a;

        ofs << t << ","
            << p.x() << "," << p.y() << "," << p.z() << ","
            << v.x() << "," << v.y() << "," << v.z() << ","
            << a.x() << "," << a.y() << "," << a.z() << ","
            << j.x() << "," << j.y() << "," << j.z()
            << "\n";
    }

    ofs.flush();
}

// ------------------------ node ------------------------

class LocalTrajBenchmarkNode final : public rclcpp::Node
{
public:
    LocalTrajBenchmarkNode() : Node("local_traj_benchmark_node")
    {
        // I/O + visualization
        declare_parameter<std::string>("sfc_dir", "/home/kkondo/code/dynus_ws/src/dynus/data");
        declare_parameter<std::string>("file_ext", ".mysco2");
        declare_parameter<std::string>("frame_id", "map");

        declare_parameter<std::string>("poly_topic", "/NX01/poly_safe");
        declare_parameter<std::string>("traj_committed_topic", "/NX01/traj_committed_colored");
        declare_parameter<std::string>("dgp_path_topic", "/NX01/dgp_path_marker");

        declare_parameter<bool>("visualize", true);
        declare_parameter<double>("playback_period_sec", 0.1);
        declare_parameter<bool>("latched", true);

        // NEW: trajectory dump settings
        declare_parameter<bool>("traj_dump_enable", true);
        declare_parameter<std::string>("traj_dump_root_dir", "");
        declare_parameter<double>("traj_dump_dt", -1.0);

        // Solver control
        declare_parameter<double>("assumed_last_replan_time_sec", 0.05);

        // Minimal subset of mighty.yaml that SolverGurobi needs
        declare_parameter<int>("num_N", 6);
        declare_parameter<double>("dc", 0.01);

        declare_parameter<double>("x_min", -10.0);
        declare_parameter<double>("x_max", 10.0);
        declare_parameter<double>("y_min", -10.0);
        declare_parameter<double>("y_max", 10.0);
        declare_parameter<double>("z_min", 0.0);
        declare_parameter<double>("z_max", 5.0);

        declare_parameter<double>("v_max", 1.0);
        declare_parameter<double>("a_max", 2.0);
        declare_parameter<double>("j_max", 3.0);

        declare_parameter<double>("factor_constant_step_size", 0.1);

        declare_parameter<double>("w_max", 0.5);

        declare_parameter<double>("max_gurobi_comp_time_sec", 5.0);
        declare_parameter<double>("jerk_smooth_weight", 1.0e+1);
        declare_parameter<double>("goal_pull_weight", 1.0e+2);
        declare_parameter<double>("goal_pull_time_buffer", 1.5);

        declare_parameter<bool>("debug_verbose", false);

        declare_parameter<double>("poly_seed_eps", 1e-6);
        declare_parameter<bool>("debug_poly_check", true);

        declare_parameter<std::vector<std::string>>("planner_names", std::vector<std::string>{});
        declare_parameter<std::vector<int64_t>>("num_N_list", std::vector<int64_t>{});
        declare_parameter<std::vector<double>>("factor_initial_list", std::vector<double>{2.0, 1.0, 1.0});
        declare_parameter<std::vector<double>>("factor_final_list", std::vector<double>{4.0, 3.0, 2.0});

        declare_parameter<std::string>("planner_name", "DYNUS");
        declare_parameter<bool>("use_single_threaded", false);

        declare_parameter<bool>("using_variable_elimination", true);

        // Read params
        std::vector<std::string> planner_names = this->get_parameter("planner_names").as_string_array();
        std::vector<int64_t> num_N_list_64 = this->get_parameter("num_N_list").as_integer_array();


        if (planner_names.empty())
            planner_names.push_back(this->get_parameter("planner_name").as_string());

        std::vector<int> num_N_list;
        num_N_list.reserve(num_N_list_64.size());
        for (auto v : num_N_list_64)
            num_N_list.push_back(static_cast<int>(v));

        if (num_N_list.empty())
            num_N_list.push_back(this->get_parameter("num_N").as_int());

        std::vector<double> factor_initial_list = this->get_parameter("factor_initial_list").as_double_array();
        std::vector<double> factor_final_list = this->get_parameter("factor_final_list").as_double_array();

        use_single_threaded_ = get_parameter("use_single_threaded").as_bool();
        sfc_dir_ = get_parameter("sfc_dir").as_string();
        file_ext_ = get_parameter("file_ext").as_string();
        frame_id_ = get_parameter("frame_id").as_string();

        poly_topic_ = get_parameter("poly_topic").as_string();
        traj_committed_topic_ = get_parameter("traj_committed_topic").as_string();
        dgp_path_topic_ = get_parameter("dgp_path_topic").as_string();

        visualize_ = get_parameter("visualize").as_bool();
        playback_period_sec_ = get_parameter("playback_period_sec").as_double();
        latched_ = get_parameter("latched").as_bool();

        assumed_last_replan_time_sec_ = get_parameter("assumed_last_replan_time_sec").as_double();

        // Dump params (NEW)
        traj_dump_enable_ = get_parameter("traj_dump_enable").as_bool();
        traj_dump_root_dir_ = get_parameter("traj_dump_root_dir").as_string();
        traj_dump_dt_ = get_parameter("traj_dump_dt").as_double();

        // Fill parameters struct for SolverGurobi
        par_.dc = get_parameter("dc").as_double();

        par_.x_min = get_parameter("x_min").as_double();
        par_.x_max = get_parameter("x_max").as_double();
        par_.y_min = get_parameter("y_min").as_double();
        par_.y_max = get_parameter("y_max").as_double();
        par_.z_min = get_parameter("z_min").as_double();
        par_.z_max = get_parameter("z_max").as_double();

        par_.v_max = get_parameter("v_max").as_double();
        par_.a_max = get_parameter("a_max").as_double();
        par_.j_max = get_parameter("j_max").as_double();

        par_.factor_constant_step_size = get_parameter("factor_constant_step_size").as_double();

        par_.w_max = get_parameter("w_max").as_double();

        par_.max_gurobi_comp_time_sec = get_parameter("max_gurobi_comp_time_sec").as_double();
        par_.jerk_smooth_weight = get_parameter("jerk_smooth_weight").as_double();
        par_.goal_pull_weight = get_parameter("goal_pull_weight").as_double();
        par_.goal_pull_time_buffer = get_parameter("goal_pull_time_buffer").as_double();

        par_.debug_verbose = get_parameter("debug_verbose").as_bool();

        par_.using_variable_elimination = get_parameter("using_variable_elimination").as_bool();

        poly_seed_eps_ = get_parameter("poly_seed_eps").as_double();
        debug_poly_check_ = get_parameter("debug_poly_check").as_bool();

        for (const auto &planner_name : planner_names)
        {
            for (int idx = 0; idx < (int)num_N_list.size(); ++idx)
            {
                int num_N = num_N_list[idx];
                par_.factor_initial = factor_initial_list[idx];
                par_.factor_final = factor_final_list[idx];

                factors_.clear();
                for (double f = par_.factor_initial;
                     f <= par_.factor_final + 1e-6;
                     f += par_.factor_constant_step_size)
                {
                    factors_.push_back(f);
                }

                planner_name_ = planner_name;
                par_.num_N = num_N;

                if (planner_name_ == "faster")
                {
                    par_.goal_pull_weight = 0.0;
                }
                else
                {
                    par_.goal_pull_weight = get_parameter("goal_pull_weight").as_double();
                }

                std::string thread_string = use_single_threaded_ ? "single_thread" : "multi_thread";
                csv_out_ = "/home/kkondo/code/dynus_ws/src/dynus/benchmark_data/" + thread_string + "/" +
                           planner_name_ + "_" + std::to_string(par_.num_N) + "_benchmark.csv";

                // NEW: derive dump directory for this run
                if (traj_dump_enable_)
                {
                    fs::path root;
                    if (!traj_dump_root_dir_.empty())
                        root = fs::path(traj_dump_root_dir_);
                    else
                        root = fs::path(csv_out_).parent_path() / "traj_dump";

                    traj_dump_run_dir_ = (root / (planner_name_ + "_N" + std::to_string(par_.num_N))).string();

                    if (!ensureDir(fs::path(traj_dump_run_dir_)))
                    {
                        RCLCPP_WARN(get_logger(), "Failed to create traj dump dir: %s (disabling dump for this run)",
                                    traj_dump_run_dir_.c_str());
                        traj_dump_enable_this_run_ = false;
                    }
                    else
                    {
                        traj_dump_enable_this_run_ = true;
                        RCLCPP_INFO(get_logger(), "Trajectory dump enabled. dir=%s dt_req=%.4f (dc=%.4f)",
                                    traj_dump_run_dir_.c_str(), traj_dump_dt_, par_.dc);
                    }
                }
                else
                {
                    traj_dump_enable_this_run_ = false;
                }

                RCLCPP_INFO(get_logger(), "Benchmarking planner=%s num_N=%d factors=[%.2f .. %.2f] step=%.2f cases in %s (output %s) using %s",
                            planner_name_.c_str(),
                            par_.num_N,
                            par_.factor_initial,
                            par_.factor_final,
                            par_.factor_constant_step_size,
                            sfc_dir_.c_str(),
                            csv_out_.c_str(),
                            use_single_threaded_ ? "single thread" : "multiple threads");

                // Create one solver per factor (persistent, reused across cases)
                whole_traj_solver_ptrs_.clear();
                whole_traj_solver_ptrs_.reserve(factors_.size());

                for (size_t i = 0; i < factors_.size(); ++i)
                {
                    auto s = std::make_shared<SolverGurobi>();
                    s->setPlannerName(planner_name_);
                    s->initializeSolver(par_);
                    whole_traj_solver_ptrs_.push_back(s);
                }

                // Workers
                ellip_workers_.resize(whole_traj_solver_ptrs_.size());

                // Publishers
                rclcpp::QoS qos(rclcpp::KeepLast(1));
                qos.reliable();
                if (latched_)
                    qos.transient_local();

                pub_poly_ = create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(poly_topic_, qos);
                pub_traj_committed_colored_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                    traj_committed_topic_, 10);
                pub_dgp_path_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                    dgp_path_topic_, 10);

                // Load + solve
                loadAll();
                solveAll();
                writeCsv();

                if (visualize_ && !results_.empty())
                {
                    playback_timer_ = create_wall_timer(
                        std::chrono::duration<double>(std::max(0.05, playback_period_sec_)),
                        std::bind(&LocalTrajBenchmarkNode::publishNext, this));
                }
            }
        }
    }

private:
    void loadAll()
    {
        results_.clear();

        if (!fs::exists(sfc_dir_))
            throw std::runtime_error("sfc_dir does not exist: " + sfc_dir_);

        std::vector<fs::path> files;
        for (const auto &ent : fs::directory_iterator(sfc_dir_))
        {
            if (!ent.is_regular_file())
                continue;
            const auto p = ent.path();
            if (p.extension() == file_ext_)
                files.push_back(p);
        }

        std::sort(files.begin(), files.end());

        for (const auto &f : files)
        {
            BenchResult r;
            r.file = f.string();
            results_.push_back(std::move(r));
        }
    }

    void maybeDumpTrajectory(const std::string &case_fname,
                             const std::vector<state> &goal_setpoints,
                             const BenchResult &r)
    {
        if (!traj_dump_enable_this_run_)
            return;

        const std::string base = sanitizeFilename("traj_" + planner_name_ + "_N" + std::to_string(par_.num_N) + "__" + case_fname);
        const fs::path out_csv = fs::path(traj_dump_run_dir_) / (base + ".csv");

        dumpTrajectoryCsvV1(out_csv,
                            planner_name_,
                            case_fname,
                            frame_id_,
                            goal_setpoints,
                            par_.dc,
                            traj_dump_dt_,
                            r.factor_used,
                            r.cost_value,
                            r.total_traj_time_sec);
    }

    void solveAll()
    {
        using ThreadRet = std::tuple<bool, bool, double, double, std::string>;
        // (success, gurobi_error, per_opt_runtime_ms, factor, msg)

        auto maxViolation = [](const LinearConstraint3D &lc, const Vec3f &p) -> double
        {
            if (lc.A_.rows() == 0)
                return 0.0;
            Eigen::VectorXd d = lc.A_ * p - lc.b_;
            return d.maxCoeff();
        };

        auto abStats = [](const LinearConstraint3D &lc,
                          double &min_row_norm, double &max_row_norm,
                          double &min_b, double &max_b)
        {
            min_row_norm = std::numeric_limits<double>::infinity();
            max_row_norm = 0.0;
            min_b = std::numeric_limits<double>::infinity();
            max_b = -std::numeric_limits<double>::infinity();

            for (int r = 0; r < lc.A_.rows(); ++r)
            {
                const double rn = lc.A_.row(r).norm();
                min_row_norm = std::min(min_row_norm, rn);
                max_row_norm = std::max(max_row_norm, rn);
            }
            for (int r = 0; r < lc.b_.size(); ++r)
            {
                min_b = std::min(min_b, lc.b_(r));
                max_b = std::max(max_b, lc.b_(r));
            }

            if (!std::isfinite(min_row_norm))
                min_row_norm = 0.0;
            if (!std::isfinite(min_b))
                min_b = 0.0;
            if (!std::isfinite(max_b))
                max_b = 0.0;
        };

        auto applyConstraintReport = [](BenchResult &out, const ConstraintReport &rep)
        {
            out.corridor_max_min_violation = rep.corridor_max_min_violation;
            out.corridor_t_at_max = rep.corridor_t_at_max;
            out.corridor_p_at_max = rep.corridor_p_at_max;
            out.corridor_best_poly_idx_at_max = rep.corridor_best_poly_idx_at_max;

            out.v_max_observed = rep.v_max_observed;
            out.v_max_excess = rep.v_max_excess;
            out.v_t_at_max = rep.v_t_at_max;

            out.a_max_observed = rep.a_max_observed;
            out.a_max_excess = rep.a_max_excess;
            out.a_t_at_max = rep.a_t_at_max;

            out.j_max_observed = rep.j_max_observed;
            out.j_max_excess = rep.j_max_excess;
            out.j_t_at_max = rep.j_t_at_max;

            out.corridor_violated = rep.corridor_violated;
            out.v_violated = rep.v_violated;
            out.a_violated = rep.a_violated;
            out.j_violated = rep.j_violated;

            out.jerk_smoothness_l1 = rep.jerk_smoothness_l1;
            out.jerk_rms = rep.jerk_rms;

            out.traj_length_m = rep.traj_length_m;
        };

        for (auto &r : results_)
        {
            const std::string fname = fs::path(r.file).filename().string();

            try
            {
                Vec3d start, goal;
                vec_Vecf<3> path;
                std::vector<double> seg_end_times;
                vec_E<Polyhedron<3>> poly_out;
                std::vector<LinearConstraint3D> l_constraints;

                loadMysco2(r.file, start, goal, path, seg_end_times, poly_out, l_constraints,
                           poly_seed_eps_, debug_poly_check_);

                r.start = start;
                r.goal = goal;

                // Cache corridor polyhedra for RViz
                {
                    auto msg = DecompROS::polyhedron_array_to_ros(poly_out);
                    msg.header.frame_id = frame_id_;
                    msg.header.stamp = now();
                    msg.lifetime = rclcpp::Duration::from_seconds(1.0);
                    r.poly_msg = msg;
                }

                // Global path marker array
                {
                    r.global_path_ma.markers.clear();
                    vectorOfVectors2MarkerArray(path, &r.global_path_ma, color(RED));
                }

                const int num_seg = static_cast<int>(l_constraints.size());

                if (num_seg > par_.num_N)
                {
                    r.success = false;
                    r.status = "SKIP: polytopes(" + std::to_string(num_seg) +
                               ") > num_N(" + std::to_string(par_.num_N) + ")";
                    const auto t1 = steady_clock::now();
                    r.total_opt_runtime_ms = 1000000.0;
                    continue;
                }

                if (num_seg <= 0)
                    throw std::runtime_error("No segments/constraints loaded.");

                // Sanity checks
                const Vec3f p_start = path.front();
                const Vec3f p_goal = path.back();

                const double v_start = maxViolation(l_constraints.front(), p_start);
                const double v_goal = maxViolation(l_constraints.back(), p_goal);

                int bad_mid_count = 0;
                for (int i = 0; i < num_seg; ++i)
                {
                    const Vec3f mid = 0.5 * (path[i] + path[i + 1]);
                    const double vm = maxViolation(l_constraints[i], mid);
                    if (vm > 1e-6)
                        bad_mid_count++;
                }

                // If start/goal/mids violate, skip
                if (v_start > 1e-5 || v_goal > 1e-5 || bad_mid_count > 0)
                {
                    r.success = false;
                    r.gurobi_error = false;
                    r.status = "BAD_CONSTRAINTS: start/goal/mid violates corridor (see logs)";
                    const auto t1 = steady_clock::now();
                    r.total_opt_runtime_ms = 1000000.0;
                    continue;
                }

                // Reset all solvers
                for (auto &s : whole_traj_solver_ptrs_)
                    s->resetToNominalState();

                // Setup states
                state A, E;
                fillStateFromPos(A, start);
                fillStateFromPos(E, goal);

                // initial_dt
                whole_traj_solver_ptrs_[0]->setX0(A);
                whole_traj_solver_ptrs_[0]->setXf(E);
                const double initial_dt = whole_traj_solver_ptrs_[0]->getInitialDt();

                const std::vector<double> sub_goal = {goal.x(), goal.y(), goal.z()};
                const double goal_pull_time = par_.goal_pull_time_buffer * assumed_last_replan_time_sec_;

                if (use_single_threaded_)
                {
                    auto solver = whole_traj_solver_ptrs_[0];
                    solver->resetToNominalState();
                    solver->setX0(A);
                    solver->setXf(E);
                    solver->setInitialDt(initial_dt);
                    solver->setT0(0.0);
                    solver->setPolytopes(l_constraints);
                    solver->setSubGoal(sub_goal);
                    solver->setGoalPullTime(goal_pull_time);

                    bool gurobi_error = false;
                    double gurobi_ms = 0.0;

                    const auto t0 = steady_clock::now();
                    const bool ok = solver->generateNewTrajectory(gurobi_error, gurobi_ms, /*factor=*/1.0, true);
                    const auto t1 = steady_clock::now();
                    r.total_opt_runtime_ms = 1e3 * duration<double>(t1 - t0).count();

                    r.per_opt_runtime_ms = gurobi_ms;
                    r.factor_used = solver->getFactorThatWorked();
                    r.cost_value = solver->getObjectiveValue();

                    if (!ok)
                    {
                        r.success = false;
                        r.gurobi_error = gurobi_error;
                        r.status = gurobi_error ? "GRB_ERROR" : "NO_SOLUTION";
                    }
                    else
                    {
                        solver->getTotalTrajTime(r.total_traj_time_sec);
                        solver->fillGoalSetPoints();

                        std::vector<state> goal_setpoints;
                        solver->getGoalSetpoints(goal_setpoints);

                        const auto crep = analyzeConstraintsSampled(
                            goal_setpoints, l_constraints, par_.dc,
                            par_.v_max, par_.a_max, par_.j_max);
                        applyConstraintReport(r, crep);

                        r.opt_traj_ma = stateVector2ColoredMarkerArray(goal_setpoints, /*type=*/1, par_.v_max, this->now());
                        r.success = true;
                        r.gurobi_error = false;
                        r.status = "OK (factor=" + std::to_string(r.factor_used) + ")";

                        // NEW: dump trajectory
                        maybeDumpTrajectory(fname, goal_setpoints, r);
                    }
                }
                else
                {
                    // Launch async workers
                    std::vector<std::future<ThreadRet>> futures;
                    futures.reserve(whole_traj_solver_ptrs_.size());

                    const auto t0 = steady_clock::now();

                    for (size_t i = 0; i < whole_traj_solver_ptrs_.size(); ++i)
                    {
                        const double factor = factors_[i];
                        auto solver = whole_traj_solver_ptrs_[i];

                        futures.push_back(std::async(std::launch::async,
                                                     [solver, &l_constraints, A, E, sub_goal, initial_dt, goal_pull_time, factor]() -> ThreadRet
                                                     {
                                                         try
                                                         {
                                                             solver->setX0(A);
                                                             solver->setXf(E);
                                                             solver->setInitialDt(initial_dt);
                                                             solver->setT0(0.0);
                                                             solver->setPolytopes(l_constraints);
                                                             solver->setSubGoal(sub_goal);
                                                             solver->setGoalPullTime(goal_pull_time);

                                                             bool gurobi_error = false;
                                                             double per_opt_runtime_ms = 0.0;
                                                             const bool ok = solver->generateNewTrajectory(gurobi_error, per_opt_runtime_ms, factor);
                                                             const bool success = ok && (!gurobi_error);

                                                             std::string msg = success ? "SUCCESS" : (gurobi_error ? "GRB_ERROR" : "NO_SOLUTION");
                                                             return {success, gurobi_error, per_opt_runtime_ms, factor, msg};
                                                         }
                                                         catch (const std::exception &e)
                                                         {
                                                             return {false, true, 0.0, factor, std::string("EXCEPTION: ") + e.what()};
                                                         }
                                                         catch (...)
                                                         {
                                                             return {false, true, 0.0, factor, "UNKNOWN_EXCEPTION"};
                                                         }
                                                     }));
                    }

                    if (planner_name_ == "dynus" || planner_name_ == "faster")
                    {

                        // Stop-at-first-success mode
                        int success_idx = -1;
                        bool any_gurobi_error = false;
                        std::string last_fail_msg;

                        std::vector<bool> got(futures.size(), false);
                        size_t remaining = futures.size();

                        while (remaining > 0 && success_idx < 0)
                        {
                            bool progressed = false;
                            for (size_t i = 0; i < futures.size(); ++i)
                            {
                                if (got[i])
                                    continue;
                                if (futures[i].wait_for(0ms) == std::future_status::ready)
                                {
                                    progressed = true;
                                    got[i] = true;
                                    --remaining;

                                    auto [succ, gurobi_error, gurobi_ms, factor, msg] = futures[i].get();
                                    any_gurobi_error = any_gurobi_error || gurobi_error;
                                    last_fail_msg = msg;

                                    if (succ && success_idx < 0)
                                    {
                                        success_idx = (int)i;
                                        r.per_opt_runtime_ms = gurobi_ms;
                                        r.factor_used = factor;

                                        for (size_t j = 0; j < whole_traj_solver_ptrs_.size(); ++j)
                                        {
                                            if ((int)j == success_idx)
                                                continue;
                                            try
                                            {
                                                whole_traj_solver_ptrs_[j]->stopExecution();
                                            }
                                            catch (...)
                                            {
                                            }
                                        }
                                        break;
                                    }
                                }
                            }
                            if (!progressed)
                                std::this_thread::sleep_for(1ms);
                        }

                        const auto t1 = steady_clock::now();
                        r.total_opt_runtime_ms = 1e3 * duration<double>(t1 - t0).count();

                        if (success_idx < 0)
                        {
                            r.success = false;
                            r.gurobi_error = any_gurobi_error;
                            r.status = any_gurobi_error ? ("GRB_ERROR: " + last_fail_msg) : ("NO_SOLUTION: " + last_fail_msg);
                        }
                        else
                        {
                            auto &solver = whole_traj_solver_ptrs_[success_idx];
                            solver->getTotalTrajTime(r.total_traj_time_sec);
                            solver->fillGoalSetPoints();

                            std::vector<state> goal_setpoints;
                            solver->getGoalSetpoints(goal_setpoints);

                            const auto crep = analyzeConstraintsSampled(
                                goal_setpoints, l_constraints, par_.dc,
                                par_.v_max, par_.a_max, par_.j_max);
                            applyConstraintReport(r, crep);

                            r.opt_traj_ma = stateVector2ColoredMarkerArray(goal_setpoints, /*type=*/1, par_.v_max, this->now());
                            r.success = true;
                            r.gurobi_error = false;
                            r.cost_value = solver->getObjectiveValue();
                            r.status = "OK (factor=" + std::to_string(r.factor_used) + ")";

                            // NEW: dump trajectory
                            maybeDumpTrajectory(fname, goal_setpoints, r);
                        }
                    }
                    else if (planner_name_ == "dynus_star" || planner_name_ == "faster_star")
                    {
                        // Wait-for-all then pick smallest factor among successes
                        bool any_gurobi_error = false;
                        std::string last_fail_msg;
                        int best_idx = -1;
                        double best_factor = std::numeric_limits<double>::infinity();

                        for (size_t i = 0; i < futures.size(); ++i)
                        {
                            auto [succ, gurobi_error, gurobi_ms, factor, msg] = futures[i].get();
                            any_gurobi_error = any_gurobi_error || gurobi_error;
                            last_fail_msg = msg;

                            if (succ && factor < best_factor)
                            {
                                best_factor = factor;
                                best_idx = (int)i;
                                r.per_opt_runtime_ms = gurobi_ms;
                                r.factor_used = factor;
                            }
                        }

                        const auto t1 = steady_clock::now();
                        r.total_opt_runtime_ms = 1e3 * duration<double>(t1 - t0).count();

                        if (best_idx < 0)
                        {
                            r.success = false;
                            r.gurobi_error = any_gurobi_error;
                            r.status = any_gurobi_error ? ("GRB_ERROR: " + last_fail_msg)
                                                        : ("NO_SOLUTION: " + last_fail_msg);
                        }
                        else
                        {
                            auto &solver = whole_traj_solver_ptrs_[best_idx];
                            solver->getTotalTrajTime(r.total_traj_time_sec);
                            solver->fillGoalSetPoints();

                            std::vector<state> goal_setpoints;
                            solver->getGoalSetpoints(goal_setpoints);

                            const auto crep = analyzeConstraintsSampled(
                                goal_setpoints, l_constraints, par_.dc,
                                par_.v_max, par_.a_max, par_.j_max);
                            applyConstraintReport(r, crep);

                            r.opt_traj_ma = stateVector2ColoredMarkerArray(
                                goal_setpoints,
                                /*type=*/1,
                                par_.v_max,
                                this->now());

                            r.success = true;
                            r.gurobi_error = false;
                            r.cost_value = solver->getObjectiveValue();
                            r.status = "OK (factor=" + std::to_string(r.factor_used) + ")";

                            // NEW: dump trajectory
                            maybeDumpTrajectory(fname, goal_setpoints, r);
                        }
                    }
                }
            }
            catch (const std::exception &e)
            {
                r.success = false;
                r.status = std::string("EXCEPTION: ") + e.what();
            }

        }
    }

    void writeCsv() const
    {
        std::ofstream ofs(csv_out_);
        if (!ofs)
        {
            RCLCPP_WARN(get_logger(), "Failed to open csv_out: %s", csv_out_.c_str());
            return;
        }

        ofs << "planner_name,file,success,status,gurobi_error,per_opt_runtime_ms,total_opt_runtime_ms,factor_used,cost_value,total_traj_time_sec,"
               "corridor_max_min_violation,corridor_t_at_max,corridor_px,corridor_py,corridor_pz,corridor_best_poly_idx,corridor_violated,"
               "v_max_observed,v_max_excess,v_t_at_max,v_violated,"
               "a_max_observed,a_max_excess,a_t_at_max,a_violated,"
               "j_max_observed,j_max_excess,j_t_at_max,j_violated,"
               "jerk_smoothness_l1,jerk_rms,"
               "traj_length_m,"
               "start_x,start_y,start_z,goal_x,goal_y,goal_z\n";

        for (const auto &r : results_)
        {
            ofs << planner_name_ << ","
                << fs::path(r.file).filename().string() << ","
                << (r.success ? 1 : 0) << ","
                << "\"" << r.status << "\"" << ","
                << (r.gurobi_error ? 1 : 0) << ","
                << std::fixed << std::setprecision(3)
                << r.per_opt_runtime_ms << ","
                << r.total_opt_runtime_ms << ","
                << r.factor_used << ","
                << r.cost_value << ","
                << r.total_traj_time_sec << ","
                << std::setprecision(9)
                << r.corridor_max_min_violation << ","
                << r.corridor_t_at_max << ","
                << r.corridor_p_at_max.x() << "," << r.corridor_p_at_max.y() << "," << r.corridor_p_at_max.z() << ","
                << r.corridor_best_poly_idx_at_max << ","
                << (r.corridor_violated ? 1 : 0) << ","
                << r.v_max_observed << ","
                << r.v_max_excess << ","
                << r.v_t_at_max << ","
                << (r.v_violated ? 1 : 0) << ","
                << r.a_max_observed << ","
                << r.a_max_excess << ","
                << r.a_t_at_max << ","
                << (r.a_violated ? 1 : 0) << ","
                << r.j_max_observed << ","
                << r.j_max_excess << ","
                << r.j_t_at_max << ","
                << (r.j_violated ? 1 : 0) << ","
                << r.jerk_smoothness_l1 << ","
                << r.jerk_rms << ","
                << r.traj_length_m << ","
                << std::setprecision(3)
                << r.start.x() << "," << r.start.y() << "," << r.start.z() << ","
                << r.goal.x() << "," << r.goal.y() << "," << r.goal.z()
                << "\n";
        }

        ofs.flush();
        RCLCPP_INFO(get_logger(), "Wrote CSV: %s", csv_out_.c_str());
    }

    void publishNext()
    {
        auto &r = results_[play_idx_];
        auto stamp = now();

        // Clear the trajectory topic
        {
            visualization_msgs::msg::MarkerArray clear_msg;
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = stamp;
            m.action = visualization_msgs::msg::Marker::DELETEALL;
            clear_msg.markers.push_back(m);
            pub_traj_committed_colored_->publish(clear_msg);
            pub_dgp_path_marker_->publish(clear_msg);
        }

        // Publish global path markers
        for (auto &mk : r.global_path_ma.markers)
        {
            mk.header.frame_id = frame_id_;
            mk.header.stamp = stamp;
        }
        pub_dgp_path_marker_->publish(r.global_path_ma);

        // Publish committed traj markers (if success)
        if (r.success)
        {
            for (auto &mk : r.opt_traj_ma.markers)
            {
                mk.header.frame_id = frame_id_;
                mk.header.stamp = stamp;
            }
            pub_traj_committed_colored_->publish(r.opt_traj_ma);
        }

        // publish corridor polyhedra
        r.poly_msg.header.stamp = stamp;
        pub_poly_->publish(r.poly_msg);

        play_idx_ = (play_idx_ + 1) % results_.size();
    }

private:
    std::string planner_name_{"dynus"};

    // I/O
    std::string sfc_dir_;
    std::string file_ext_;
    std::string frame_id_;
    std::string poly_topic_;
    std::string traj_committed_topic_;
    std::string dgp_path_topic_;
    std::string csv_out_;

    bool use_single_threaded_{false};

    bool visualize_{true};
    double playback_period_sec_{1.0};
    bool latched_{true};

    double assumed_last_replan_time_sec_{0.05};

    parameters par_;

    std::vector<BenchResult> results_;
    size_t play_idx_{0};

    std::vector<double> factors_;
    std::vector<std::shared_ptr<SolverGurobi>> whole_traj_solver_ptrs_;

    double poly_seed_eps_{1e-6};
    bool debug_poly_check_{true};

    std::vector<EllipsoidDecomp3D> ellip_workers_;

    // NEW: trajectory dump control
    bool traj_dump_enable_{true};
    std::string traj_dump_root_dir_;
    double traj_dump_dt_{-1.0};

    bool traj_dump_enable_this_run_{false};
    std::string traj_dump_run_dir_;

    // ROS
    rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_committed_colored_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dgp_path_marker_;
    rclcpp::TimerBase::SharedPtr playback_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalTrajBenchmarkNode>());
    rclcpp::shutdown();
    return 0;
}
