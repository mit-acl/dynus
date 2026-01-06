#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <decomp_rviz_plugins/data_ros_utils.hpp> // DecompROS::polyhedron_array_to_ros
#include <decomp_util/seed_decomp.h>              // Polyhedron / Hyperplane

#include <Eigen/Dense>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <limits>
#include <cctype>
#include <cmath>

namespace fs = std::filesystem;

// ------------------------ types ------------------------

using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Matrix<double, 3, 1>;

struct TrajPoint
{
    double t{0.0};
    double x{0.0}, y{0.0}, z{0.0};
};

struct TrajCsv
{
    std::string planner_name; // NOTE: we use this as the "planner variant key" (e.g., dynus_N4)
    std::string case_file;
    std::string frame_id;
    std::vector<TrajPoint> pts;
};

struct CaseBundle
{
    std::string case_file; // basename: "sfc_g000.mysco2"
    fs::path mysco2_path;

    decomp_ros_msgs::msg::PolyhedronArray poly_msg;
    visualization_msgs::msg::MarkerArray guide_path_ma;

    // planner_variant_key -> traj
    std::map<std::string, TrajCsv> planner_to_traj;
};

// ------------------------ small helpers ------------------------

static inline std::string trim(std::string s)
{
    auto notSpace = [](int ch)
    { return !std::isspace(ch); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    return s;
}

static inline bool startsWith(const std::string &s, const std::string &pref)
{
    return s.size() >= pref.size() && s.compare(0, pref.size(), pref) == 0;
}

static inline bool endsWith(const std::string &s, const std::string &suf)
{
    return s.size() >= suf.size() && s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

static inline std::vector<std::string> splitCsvLine(const std::string &line)
{
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ','))
        out.push_back(trim(tok));
    return out;
}

static inline bool isFinite3(double x, double y, double z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

static inline std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

// Deterministic hash -> [0,1)
static inline double hash01(const std::string &s)
{
    std::uint64_t h = 1469598103934665603ull;
    for (unsigned char c : s)
    {
        h ^= c;
        h *= 1099511628211ull;
    }
    return (h % 1000000) / 1000000.0;
}

// HSV in [0,1] -> RGB [0,1]
static inline void hsv2rgb(double h, double s, double v, double &r, double &g, double &b)
{
    if (s <= 1e-9)
    {
        r = g = b = v;
        return;
    }
    h = std::fmod(h, 1.0);
    if (h < 0)
        h += 1.0;

    const double i = std::floor(h * 6.0);
    const double f = h * 6.0 - i;
    const double p = v * (1.0 - s);
    const double q = v * (1.0 - s * f);
    const double t = v * (1.0 - s * (1.0 - f));

    switch (static_cast<int>(i) % 6)
    {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    case 5:
        r = v;
        g = p;
        b = q;
        break;
    }
}

static inline geometry_msgs::msg::Point toPoint(double x, double y, double z)
{
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

static inline visualization_msgs::msg::Marker deleteAllMarker(const std::string &frame_id, const rclcpp::Time &stamp)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    return m;
}

// ------------------------ .mysco2 reader (corridor + guide path) ------------------------

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

// Build Polyhedron<3> from A x <= b, ensuring seed is inside by flipping violating planes.
static Polyhedron<3> polyFromHalfspacesSeeded(Eigen::MatrixXd A, Eigen::VectorXd b,
                                              const Vec3f &seed, double eps)
{
    for (int r = 0; r < A.rows(); ++r)
    {
        const double v = A.row(r).dot(seed) - b(r);
        if (v > eps)
        {
            A.row(r) *= -1.0;
            b(r) *= -1.0;
        }
    }

    Polyhedron<3> poly;
    for (int r = 0; r < A.rows(); ++r)
    {
        Vec3f a = A.row(r).transpose();
        const double norm = a.norm();
        if (norm < 1e-12)
            continue;

        const Vec3f n = a / norm;
        const double d = b(r) / norm;
        const Vec3f p0 = n * d; // point on plane

        poly.add(Hyperplane<3>(p0, n));
    }
    return poly;
}

static void loadMysco2CorridorAndPath(const fs::path &file,
                                      Vec3d &start, Vec3d &goal,
                                      std::vector<Vec3f> &path_pts,
                                      vec_E<Polyhedron<3>> &polys,
                                      double poly_seed_eps)
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
        throw std::runtime_error("Unsupported .mysco2 version: " + file.string());

    start.x() = readD(ifs);
    start.y() = readD(ifs);
    start.z() = readD(ifs);
    goal.x() = readD(ifs);
    goal.y() = readD(ifs);
    goal.z() = readD(ifs);

    const uint32_t num_path_pts = readU32(ifs);
    path_pts.clear();
    path_pts.reserve(num_path_pts);
    for (uint32_t i = 0; i < num_path_pts; ++i)
    {
        Vec3f p;
        p.x() = readD(ifs);
        p.y() = readD(ifs);
        p.z() = readD(ifs);
        path_pts.push_back(p);
    }

    const uint32_t num_seg = readU32(ifs);
    // seg_end_times not needed here; consume it
    for (uint32_t i = 0; i < num_seg; ++i)
        (void)readD(ifs);

    if (path_pts.size() < 2 || (path_pts.size() - 1) != num_seg)
        throw std::runtime_error("File inconsistent: path.size()-1 != num_seg in " + file.string());

    polys.clear();
    polys.resize(num_seg);

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

        const Vec3f seed = 0.5 * (path_pts[si] + path_pts[si + 1]);
        polys[si] = polyFromHalfspacesSeeded(A, b, seed, poly_seed_eps);
    }
}

static visualization_msgs::msg::MarkerArray makeGuidePathMarkers(const std::vector<Vec3f> &path,
                                                                 const std::string &frame_id,
                                                                 const rclcpp::Time &stamp,
                                                                 const std_msgs::msg::ColorRGBA &color,
                                                                 double line_width,
                                                                 double point_diam)
{
    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(deleteAllMarker(frame_id, stamp));

    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id;
    line.header.stamp = stamp;
    line.ns = "guide_path";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = line_width;
    line.color = color;

    visualization_msgs::msg::Marker pts = line;
    pts.ns = "guide_path_pts";
    pts.id = 2;
    pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    pts.scale.x = point_diam;
    pts.scale.y = point_diam;
    pts.scale.z = point_diam;

    for (const auto &p : path)
    {
        line.points.push_back(toPoint(p.x(), p.y(), p.z()));
        pts.points.push_back(toPoint(p.x(), p.y(), p.z()));
    }

    arr.markers.push_back(line);
    arr.markers.push_back(pts);
    return arr;
}

// ------------------------ trajectory CSV scan + parse ------------------------

// Tries to parse our dumped format:
// # planner_name: ...
// # case_file: ...
// then header with t,x,y,z,...
static bool parseTrajCsv(const fs::path &csv_path, TrajCsv &out)
{
    std::ifstream ifs(csv_path);
    if (!ifs)
        return false;

    out = TrajCsv{};
    std::string line;
    bool header_seen = false;
    std::unordered_map<std::string, int> col;

    while (std::getline(ifs, line))
    {
        line = trim(line);
        if (line.empty())
            continue;

        if (startsWith(line, "#"))
        {
            // metadata
            const auto pos = line.find(':');
            if (pos != std::string::npos)
            {
                const std::string key = trim(line.substr(1, pos - 1));
                const std::string val = trim(line.substr(pos + 1));
                if (key == "planner_name")
                    out.planner_name = val;
                else if (key == "case_file" || key == "source_file" || key == "file")
                    out.case_file = val;
                else if (key == "frame_id")
                    out.frame_id = val;
            }
            continue;
        }

        // First non-comment line: header
        if (!header_seen)
        {
            header_seen = true;
            const auto toks = splitCsvLine(line);
            for (int i = 0; i < (int)toks.size(); ++i)
                col[toks[i]] = i;

            // minimally require x,y,z (t optional)
            if (col.find("x") == col.end() ||
                col.find("y") == col.end() ||
                col.find("z") == col.end())
                return false;

            continue;
        }

        // Data
        const auto toks = splitCsvLine(line);
        auto getD = [&](const std::string &name, double def) -> double
        {
            auto it = col.find(name);
            if (it == col.end())
                return def;
            const int i = it->second;
            if (i < 0 || i >= (int)toks.size())
                return def;
            try
            {
                return std::stod(toks[(size_t)i]);
            }
            catch (...)
            {
                return def;
            }
        };

        TrajPoint p;
        p.t = getD("t", out.pts.empty() ? 0.0 : out.pts.back().t);
        p.x = getD("x", std::numeric_limits<double>::quiet_NaN());
        p.y = getD("y", std::numeric_limits<double>::quiet_NaN());
        p.z = getD("z", std::numeric_limits<double>::quiet_NaN());

        if (isFinite3(p.x, p.y, p.z))
            out.pts.push_back(p);
    }

    // Fallback inference from filename / directory if not present (or to enrich with N):
    // Expected filename (recommended): traj_<planner>_N<k>__<case>.csv
    // Also supported: traj_<planner>__<case>.csv
    const std::string fname = csv_path.filename().string();
    const std::string parent_dir = csv_path.parent_path().filename().string();

    auto stripCsvExt = [](std::string s) -> std::string
    {
        if (endsWith(s, ".csv"))
            s = s.substr(0, s.size() - 4);
        return s;
    };

    auto inferVariantFromFilename = [&]() -> std::string
    {
        // "traj_dynus_N4__sfc_g000.mysco2.csv" -> "dynus_N4"
        std::string s = fname;
        if (startsWith(s, "traj_"))
            s = s.substr(5);
        const auto pos2 = s.find("__");
        if (pos2 == std::string::npos)
            return std::string();
        return stripCsvExt(s.substr(0, pos2));
    };

    auto inferCaseFromFilename = [&]() -> std::string
    {
        // "traj_dynus_N4__sfc_g000.mysco2.csv" -> "sfc_g000.mysco2"
        const auto pos = fname.find("__");
        if (pos == std::string::npos)
            return std::string();
        return stripCsvExt(fname.substr(pos + 2));
    };

    auto inferVariantFromDir = [&]() -> std::string
    {
        // If parent_dir is "dynus_N4" or "faster_N6" -> keep it.
        // We treat anything containing "_N" followed by digits as a variant.
        const auto pos = parent_dir.find("_N");
        if (pos == std::string::npos)
            return std::string();

        bool has_digit = false;
        for (size_t i = pos + 2; i < parent_dir.size(); ++i)
        {
            if (std::isdigit(static_cast<unsigned char>(parent_dir[i])))
            {
                has_digit = true;
                break;
            }
        }
        return has_digit ? parent_dir : std::string();
    };

    // Case file inference
    if (out.case_file.empty())
    {
        out.case_file = inferCaseFromFilename();
    }
    else
    {
        // Sometimes header contains absolute path; normalize to basename.
        out.case_file = fs::path(out.case_file).filename().string();
    }

    // Planner variant inference (prefer filename, then directory).
    const std::string variant_fname = inferVariantFromFilename();
    const std::string variant_dir = inferVariantFromDir();
    const std::string variant = !variant_fname.empty() ? variant_fname : variant_dir;

    // Use variant as grouping key (so dynus_N4, dynus_N5, ... do not overwrite each other).
    if (!variant.empty())
    {
        out.planner_name = variant;
    }
    else if (out.planner_name.empty())
    {
        // Last resort: take substring up to "__" (without truncating at "_N")
        std::string s = fname;
        if (startsWith(s, "traj_"))
            s = s.substr(5);
        const auto pos2 = s.find("__");
        if (pos2 != std::string::npos)
            out.planner_name = stripCsvExt(s.substr(0, pos2));
        else
            out.planner_name = "unknown";
    }

    if (out.frame_id.empty())
        out.frame_id = "map";

    return !out.pts.empty() && !out.case_file.empty();
}

// ------------------------ marker building for trajectories ------------------------

static visualization_msgs::msg::MarkerArray makeTrajOverlayMarkers(
    const std::map<std::string, TrajCsv> &planner_to_traj,
    const std::string &frame_id,
    const rclcpp::Time &stamp,
    bool show_points,
    bool show_labels,
    double line_width,
    double point_diam,
    double label_height,
    double label_z_offset)
{
    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(deleteAllMarker(frame_id, stamp));

    int id = 1;

    // label spacing rule:
    const int num_planners = static_cast<int>(planner_to_traj.size());
    const double label_spacing_m = 1.0;

    std::unordered_map<std::string, int> planner_to_index;
    planner_to_index.reserve((size_t)num_planners);

    int idx = 0;
    for (const auto &kv : planner_to_traj)
        planner_to_index[kv.first] = idx++;

    const double mid = (num_planners - 1) / 2.0;

    for (const auto &kv : planner_to_traj)
    {
        const std::string &planner = kv.first; // this is now planner variant (e.g., dynus_N4)
        const TrajCsv &tr = kv.second;

        // deterministic per planner-variant color
        const double h = hash01(planner);
        double rr, gg, bb;
        hsv2rgb(h, 0.85, 0.95, rr, gg, bb);
        const auto col = makeColor((float)rr, (float)gg, (float)bb, 1.0f);

        visualization_msgs::msg::Marker line;
        line.header.frame_id = frame_id;
        line.header.stamp = stamp;
        line.ns = "traj/" + planner;
        line.id = id++;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = line_width;
        line.color = col;

        line.points.reserve(tr.pts.size());
        for (const auto &p : tr.pts)
            line.points.push_back(toPoint(p.x, p.y, p.z));
        arr.markers.push_back(line);

        if (show_points)
        {
            visualization_msgs::msg::Marker pts = line;
            pts.id = id++;
            pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            pts.scale.x = point_diam;
            pts.scale.y = point_diam;
            pts.scale.z = point_diam;
            pts.points = line.points;
            arr.markers.push_back(pts);
        }

        if (show_labels && !tr.pts.empty())
        {
            const int i = planner_to_index.at(planner);
            const double y_label = (mid - (double)i) * label_spacing_m;

            visualization_msgs::msg::Marker text;
            text.header.frame_id = frame_id;
            text.header.stamp = stamp;
            text.ns = "traj_label";
            text.id = id++;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.orientation.w = 1.0;

            text.pose.position.x = tr.pts.front().x - 1.0;
            text.pose.position.y = y_label;
            text.pose.position.z = tr.pts.front().z + label_z_offset;

            text.scale.z = label_height;
            text.color = col;
            text.text = planner; // show variant (dynus_N4, ...)
            arr.markers.push_back(text);
        }
    }

    return arr;
}

// ------------------------ node ------------------------

class VisualizeLocalTrajsNode final : public rclcpp::Node
{
public:
    VisualizeLocalTrajsNode() : Node("visualize_local_trajs")
    {
        // Inputs
        sfc_dir_ = declare_parameter<std::string>("sfc_dir", "/home/kkondo/code/dynus_ws/src/dynus/data");
        file_ext_ = declare_parameter<std::string>("file_ext", ".mysco2");
        traj_dump_root_dir_ = declare_parameter<std::string>("traj_dump_root_dir", "/home/kkondo/code/dynus_ws/src/dynus/benchmark_data/single_thread/traj_dump");

        // New: multiple roots
        traj_dump_root_dirs_ = declare_parameter<std::vector<std::string>>(
            "traj_dump_root_dirs",
            std::vector<std::string>{
                traj_dump_root_dir_,                               // dynus/faster dumps
                "/media/kkondo/kota_elements/super/traj_dump_ros1" // SUPER dumps
            });

        // Topics
        frame_id_ = declare_parameter<std::string>("frame_id", "map");
        poly_topic_ = declare_parameter<std::string>("poly_topic", "/NX01/poly_safe");
        traj_topic_ = declare_parameter<std::string>("traj_overlay_topic", "/local_traj_overlay");
        guide_path_topic_ = declare_parameter<std::string>("guide_path_topic", "/dgp_path_marker");

        // Playback
        playback_period_sec_ = declare_parameter<double>("playback_period_sec", 0.5);
        visualize_ = declare_parameter<bool>("visualize", true);

        // What to show
        show_guide_path_ = declare_parameter<bool>("show_guide_path", true);
        show_points_ = declare_parameter<bool>("show_points", true);
        show_labels_ = declare_parameter<bool>("show_labels", true);

        // Styling
        traj_line_width_ = declare_parameter<double>("traj_line_width", 0.06);
        traj_point_diam_ = declare_parameter<double>("traj_point_diam", 0.10);
        label_height_ = declare_parameter<double>("label_height", 0.25);
        label_z_offset_ = declare_parameter<double>("label_z_offset", 0.25);

        // Corridor load
        poly_seed_eps_ = declare_parameter<double>("poly_seed_eps", 1e-6);

        // QoS
        latched_ = declare_parameter<bool>("latched", true);

        if (sfc_dir_.empty())
        {
            RCLCPP_ERROR(get_logger(), "Parameter sfc_dir is empty.");
            return;
        }
        if (traj_dump_root_dir_.empty())
        {
            RCLCPP_ERROR(get_logger(), "Parameter traj_dump_root_dir is empty.");
            return;
        }

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliable();
        if (latched_)
            qos.transient_local();

        pub_poly_ = create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(poly_topic_, qos);
        pub_traj_ = create_publisher<visualization_msgs::msg::MarkerArray>(traj_topic_, qos);
        pub_guide_path_ = create_publisher<visualization_msgs::msg::MarkerArray>(guide_path_topic_, qos);

        loadAllCasesAndTrajs();

        if (cases_.empty())
        {
            RCLCPP_WARN(get_logger(), "No cases loaded. Check sfc_dir and file_ext.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu cases with trajectories. Starting playback.", cases_.size());

        if (visualize_)
        {
            const double period = std::max(0.05, playback_period_sec_);
            timer_ = create_wall_timer(std::chrono::duration<double>(period),
                                       std::bind(&VisualizeLocalTrajsNode::publishNext, this));
        }
        else
        {
            publishCase(0);
        }
    }

private:
    void loadAllCasesAndTrajs()
    {
        // 1) Load cases (.mysco2)
        std::vector<fs::path> mysco2_files;
        for (const auto &ent : fs::directory_iterator(sfc_dir_))
        {
            if (!ent.is_regular_file())
                continue;
            const auto p = ent.path();
            if (p.extension() == file_ext_)
                mysco2_files.push_back(p);
        }
        std::sort(mysco2_files.begin(), mysco2_files.end());

        std::unordered_map<std::string, size_t> case_index; // case_basename -> idx
        cases_.clear();
        cases_.reserve(mysco2_files.size());

        for (const auto &p : mysco2_files)
        {
            CaseBundle cb;
            cb.case_file = p.filename().string();
            cb.mysco2_path = p;
            cases_.push_back(std::move(cb));
            case_index[cases_.back().case_file] = cases_.size() - 1;
        }

        // 2) Load trajectories by scanning traj_dump_root_dir recursively
        size_t traj_count = 0;

        for (const auto &root_str : traj_dump_root_dirs_)
        {
            if (root_str.empty())
                continue;

            const fs::path root(root_str);
            std::error_code ec;
            if (!fs::exists(root, ec))
            {
                RCLCPP_WARN(get_logger(), "traj_dump_root_dirs entry does not exist: %s", root_str.c_str());
                continue;
            }

            for (auto it = fs::recursive_directory_iterator(root, ec);
                 it != fs::recursive_directory_iterator(); ++it)
            {
                if (ec)
                    break;
                if (!it->is_regular_file())
                    continue;

                const auto p = it->path();
                if (p.extension() != ".csv")
                    continue;

                TrajCsv tr;
                if (!parseTrajCsv(p, tr))
                    continue;

                // normalize to basename (important if SUPER writes full paths)
                tr.case_file = fs::path(tr.case_file).filename().string();

                // match to sfc_dir cases
                auto itc = case_index.find(tr.case_file);
                if (itc == case_index.end())
                {
                    // If your mysco2 files in sfc_dir are named "sfc_g000.mysco2" and SUPER logs "sfc_g000"
                    // you can optionally try appending extension here.
                    if (!endsWith(tr.case_file, file_ext_) &&
                        case_index.find(tr.case_file + file_ext_) != case_index.end())
                    {
                        tr.case_file = tr.case_file + file_ext_;
                        itc = case_index.find(tr.case_file);
                    }
                }
                if (itc == case_index.end())
                    continue;

                tr.frame_id = frame_id_;

                // Use tr.planner_name as the “variant key” (dynus_N4, faster_N6, super, etc.)
                cases_[itc->second].planner_to_traj[tr.planner_name] = std::move(tr);
                traj_count++;
            }
        }

        // 3) For each case, load corridor polytope and guide path markers
        std::vector<CaseBundle> filtered;
        filtered.reserve(cases_.size());

        for (auto &cb : cases_)
        {
            if (cb.planner_to_traj.empty())
                continue;

            try
            {
                Vec3d start, goal;
                std::vector<Vec3f> path_pts;
                vec_E<Polyhedron<3>> polys;

                loadMysco2CorridorAndPath(cb.mysco2_path, start, goal, path_pts, polys, poly_seed_eps_);

                auto msg = DecompROS::polyhedron_array_to_ros(polys);
                msg.header.frame_id = frame_id_;
                msg.header.stamp = now();
                msg.lifetime = rclcpp::Duration::from_seconds(1.0);
                cb.poly_msg = msg;

                cb.guide_path_ma = makeGuidePathMarkers(
                    path_pts, frame_id_, now(),
                    makeColor(0.6f, 0.6f, 0.6f, 1.0f),
                    0.04, 0.08);
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(get_logger(), "Failed loading mysco2 %s: %s",
                            cb.mysco2_path.string().c_str(), e.what());
                continue;
            }

            filtered.push_back(std::move(cb));
        }

        cases_.swap(filtered);

        RCLCPP_INFO(get_logger(),
                    "Scan complete: %zu cases in sfc_dir, %zu cases with trajectories, %zu trajectory CSVs loaded.",
                    mysco2_files.size(), cases_.size(), traj_count);
    }

    void publishNext()
    {
        if (cases_.empty())
            return;
        publishCase(play_idx_);
        play_idx_ = (play_idx_ + 1) % cases_.size();
    }

    void publishCase(size_t idx)
    {
        if (idx >= cases_.size())
            return;

        const auto stamp = now();
        auto &cb = cases_[idx];

        // Clear
        {
            visualization_msgs::msg::MarkerArray clear;
            clear.markers.push_back(deleteAllMarker(frame_id_, stamp));
            pub_traj_->publish(clear);
            pub_guide_path_->publish(clear);
        }

        // Corridor
        cb.poly_msg.header.stamp = stamp;
        pub_poly_->publish(cb.poly_msg);

        // Guide path
        if (show_guide_path_)
        {
            for (auto &mk : cb.guide_path_ma.markers)
            {
                mk.header.frame_id = frame_id_;
                mk.header.stamp = stamp;
            }
            pub_guide_path_->publish(cb.guide_path_ma);
        }

        // Overlay trajectories for this case (now includes dynus_N4, dynus_N5, ...)
        const auto traj_ma = makeTrajOverlayMarkers(
            cb.planner_to_traj, frame_id_, stamp,
            show_points_, show_labels_,
            traj_line_width_, traj_point_diam_,
            label_height_, label_z_offset_);

        pub_traj_->publish(traj_ma);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Visualizing case %zu/%zu: %s (variants=%zu)",
                             idx + 1, cases_.size(), cb.case_file.c_str(), cb.planner_to_traj.size());
    }

private:
    // params
    std::string sfc_dir_;
    std::string file_ext_;
    std::vector<std::string> traj_dump_root_dirs_;
    std::string traj_dump_root_dir_;

    std::string frame_id_;
    std::string poly_topic_;
    std::string traj_topic_;
    std::string guide_path_topic_;

    bool visualize_{true};
    double playback_period_sec_{0.5};
    bool latched_{true};

    bool show_guide_path_{true};
    bool show_points_{true};
    bool show_labels_{true};

    double traj_line_width_{0.06};
    double traj_point_diam_{0.10};
    double label_height_{0.25};
    double label_z_offset_{0.25};

    double poly_seed_eps_{1e-6};

    // state
    std::vector<CaseBundle> cases_;
    size_t play_idx_{0};

    // ros
    rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_guide_path_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizeLocalTrajsNode>());
    rclcpp::shutdown();
    return 0;
}
