#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

// Your project headers (adjust include paths to match your repo)
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

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using namespace mighty;

// Type‐aliases
using Vec3 = Eigen::Vector3d;
using Vec3f = Eigen::Matrix<double, 3, 1>;
using MatXd = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

static Vec3f vec3FromStd(const std::vector<double> &v, const char *name)
{
    if (v.size() != 3)
        throw std::runtime_error(std::string("Expected ") + name + " to have size 3.");
    return Vec3f(v[0], v[1], v[2]);
}

static std::vector<Vec3f> goalsFromFlat(const std::vector<double> &flat)
{
    if (flat.empty())
        return {};
    if (flat.size() % 3 != 0)
        throw std::runtime_error("goals_flat must have length multiple of 3.");

    std::vector<Vec3f> goals;
    goals.reserve(flat.size() / 3);
    for (size_t i = 0; i < flat.size(); i += 3)
        goals.emplace_back(flat[i + 0], flat[i + 1], flat[i + 2]);
    return goals;
}

static vec_Vec3f pclToVec3f(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    vec_Vec3f out;
    out.reserve(cloud.size());
    for (const auto &p : cloud.points)
        out.emplace_back(p.x, p.y, p.z);
    return out;
}

// Cumulative end-time per segment for dynamic-obstacle inflation.
// If you don’t care about dynamic inflation for corridor generation, this is still fine.
static std::vector<double> computeSegEndTimesFromPath(
    const vec_Vecf<3> &path,
    double nominal_speed_mps,
    double min_dt = 1e-3)
{
    std::vector<double> seg_end_times;
    if (path.size() < 2)
        return seg_end_times;
    const size_t num_seg = path.size() - 1;

    seg_end_times.reserve(num_seg);

    double t = 0.0;
    const double v = std::max(nominal_speed_mps, 1e-3);

    for (size_t i = 0; i < num_seg; ++i)
    {
        const double dist = (path[i + 1] - path[i]).norm();
        const double dt = std::max(dist / v, min_dt);
        t += dt;
        seg_end_times.push_back(t); // cumulative time at end of segment i
    }
    return seg_end_times;
}

// Dependency-free binary serialization for reuse.
// File format (little-endian):
//   magic[8]="MYSCO2\0\0"
//   u32 version=1
//   start(3*double), goal(3*double)
//   u32 num_path_pts; path_pts[num_path_pts](3*double)
//   u32 num_seg; seg_end_times[num_seg](double)
//   for each seg:
//     u32 num_planes
//     A[num_planes][3] (double)
//     b[num_planes] (double)
static void saveCorridorBinary(
    const fs::path &filepath,
    const Vec3f &start,
    const Vec3f &goal,
    const vec_Vecf<3> &path,
    const std::vector<double> &seg_end_times,
    const std::vector<LinearConstraint3D> &l_constraints)
{
    std::ofstream ofs(filepath, std::ios::binary);
    if (!ofs)
        throw std::runtime_error("Failed to open file for writing: " + filepath.string());

    const char magic[8] = {'M', 'Y', 'S', 'C', 'O', '2', '\0', '\0'};
    const uint32_t version = 1;

    auto writeU32 = [&](uint32_t v)
    { ofs.write(reinterpret_cast<const char *>(&v), sizeof(v)); };
    auto writeD = [&](double v)
    { ofs.write(reinterpret_cast<const char *>(&v), sizeof(v)); };

    ofs.write(magic, 8);
    writeU32(version);

    // start / goal
    writeD(start.x());
    writeD(start.y());
    writeD(start.z());
    writeD(goal.x());
    writeD(goal.y());
    writeD(goal.z());

    // path
    writeU32(static_cast<uint32_t>(path.size()));
    for (const auto &p : path)
    {
        writeD(p.x());
        writeD(p.y());
        writeD(p.z());
    }

    // seg_end_times
    const uint32_t num_seg = static_cast<uint32_t>(seg_end_times.size());
    writeU32(num_seg);
    for (double t : seg_end_times)
        writeD(t);

    // constraints
    if (l_constraints.size() != num_seg)
        throw std::runtime_error("l_constraints size mismatch with seg_end_times.");

    for (uint32_t i = 0; i < num_seg; ++i)
    {
        const auto &A = l_constraints[i].A_;
        const auto &b = l_constraints[i].b_;

        if (A.cols() != 3)
            throw std::runtime_error("Constraint A must have 3 columns.");

        if (A.rows() != b.size())
            throw std::runtime_error("Constraint A rows must match b size.");

        const uint32_t num_planes = static_cast<uint32_t>(A.rows());
        writeU32(num_planes);

        // A row-major
        for (uint32_t r = 0; r < num_planes; ++r)
            for (int c = 0; c < 3; ++c)
                writeD(A(r, c));

        // b
        for (uint32_t r = 0; r < num_planes; ++r)
            writeD(b(r));
    }

    ofs.flush();
}

class CorridorGeneratorNode final : public rclcpp::Node
{
public:
    CorridorGeneratorNode() : Node("corridor_generator_node")
    {
        declare_parameter<std::string>("map_topic", "/map_generator/global_cloud");
        declare_parameter<std::vector<double>>("start", {-4.0, 0.0, 1.0});

        // Map window to read into VoxelMapUtil (make this cover all your goals for fairness)
        declare_parameter<std::vector<double>>("map_center", {0.0, 0.0, 1.0});
        declare_parameter<double>("wdx", 20.0);
        declare_parameter<double>("wdy", 20.0);
        declare_parameter<double>("wdz", 10.0);

        // Output
        declare_parameter<std::string>("output_dir", "/home/kkondo/code/dynus_ws/src/dynus/data");
        declare_parameter<std::string>("output_prefix", "sfc");

        // Planner / decomp essentials
        declare_parameter<std::string>("global_planner", "sjps");
        declare_parameter<bool>("global_planner_verbose", false);
        declare_parameter<double>("global_planner_huristic_weight", 1.0);
        declare_parameter<int>("dgp_timeout_duration_ms", 100000);

        // Corridor generation / inflation params
        declare_parameter<double>("res", 0.15);
        declare_parameter<double>("factor_dgp", 1.0);
        declare_parameter<double>("inflation_dgp", 0.45);
        declare_parameter<double>("drone_radius", 0.1);
        declare_parameter<double>("obst_max_vel", 0.0);           // set >0 only if you want dynamic inflation
        declare_parameter<double>("corridor_nominal_speed", 2.0); // used to compute seg_end_times
        declare_parameter<double>("v_max", 2.0);
        declare_parameter<double>("a_max", 3.0);
        declare_parameter<double>("j_max", 10.0);

        // Decomp tuning
        declare_parameter<std::vector<double>>("local_box_size", {4.0, 4.0, 3.0});
        declare_parameter<bool>("use_shrinked_box", false);
        declare_parameter<double>("shrinked_box_size", 0.2);
        declare_parameter<double>("max_dist_vertexes", 10.0);

        // Map bounds for VoxelMapUtil ctor
        declare_parameter<double>("x_min", -100.0);
        declare_parameter<double>("x_max", 100.0);
        declare_parameter<double>("y_min", -100.0);
        declare_parameter<double>("y_max", 100.0);
        declare_parameter<double>("z_min", 0.0);
        declare_parameter<double>("z_max", 2.0);

        // For publisher
        declare_parameter<std::string>("frame_id", "map");
        declare_parameter<std::string>("poly_topic", "/NX01/poly_safe");
        declare_parameter<std::string>("path_topic", "/NX01/dgp_path_marker");
        declare_parameter<bool>("publish_ellipsoids", false);
        declare_parameter<std::string>("ellip_topic", "/mighty/sfc_ellip");
        declare_parameter<bool>("keep_alive", true);            // keep node running for RViz
        declare_parameter<double>("republish_period_sec", 1.0); // 0 disables periodic republish

        // Read parameters
        map_topic_ = get_parameter("map_topic").as_string();
        output_dir_ = get_parameter("output_dir").as_string();
        output_prefix_ = get_parameter("output_prefix").as_string();

        start_ = vec3FromStd(get_parameter("start").as_double_array(), "start");
        goals_.clear();
        for (double y = -5.0; y <= 5.0 + 1e-3; y += 0.1)
        {
            goals_.emplace_back(4.0, y, 1.0);
        }
        if (goals_.empty())
        {
            RCLCPP_WARN(get_logger(), "No goals provided (goals_flat is empty). Node will idle.");
        }

        map_center_ = vec3FromStd(get_parameter("map_center").as_double_array(), "map_center");
        wdx_ = get_parameter("wdx").as_double();
        wdy_ = get_parameter("wdy").as_double();
        wdz_ = get_parameter("wdz").as_double();

        corridor_nominal_speed_ = get_parameter("corridor_nominal_speed").as_double();

        // Fill the subset of `parameters` that DGPManager and cvxEllipsoidDecomp use.
        par_.res = get_parameter("res").as_double();
        par_.factor_dgp = get_parameter("factor_dgp").as_double();
        par_.inflation_dgp = get_parameter("inflation_dgp").as_double();
        par_.drone_radius = get_parameter("drone_radius").as_double();
        par_.obst_max_vel = get_parameter("obst_max_vel").as_double();

        par_.local_box_size = get_parameter("local_box_size").as_double_array(); // should be vector<double> in your struct
        par_.use_shrinked_box = get_parameter("use_shrinked_box").as_bool();
        par_.shrinked_box_size = get_parameter("shrinked_box_size").as_double();
        par_.max_dist_vertexes = get_parameter("max_dist_vertexes").as_double();

        par_.x_min = get_parameter("x_min").as_double();
        par_.x_max = get_parameter("x_max").as_double();
        par_.y_min = get_parameter("y_min").as_double();
        par_.y_max = get_parameter("y_max").as_double();
        par_.z_min = get_parameter("z_min").as_double();
        par_.z_max = get_parameter("z_max").as_double();

        global_planner_ = get_parameter("global_planner").as_string();
        global_planner_verbose_ = get_parameter("global_planner_verbose").as_bool();
        weight_ = get_parameter("global_planner_huristic_weight").as_double();
        dgp_timeout_ms_ = get_parameter("dgp_timeout_duration_ms").as_int();

        v_max_ = get_parameter("v_max").as_double();
        a_max_ = get_parameter("a_max").as_double();
        j_max_ = get_parameter("j_max").as_double();

        // Init DGPManager with parameters
        dgp_.setParameters(par_);

        // Subscribe to point cloud
        sub_map_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            map_topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
            std::bind(&CorridorGeneratorNode::mapCb, this, std::placeholders::_1));

        // Periodic trigger to run once map is ready
        timer_ = create_wall_timer(200ms, std::bind(&CorridorGeneratorNode::tick, this));

        // For publisher
        frame_id_ = get_parameter("frame_id").as_string();
        poly_topic_ = get_parameter("poly_topic").as_string();
        path_topic_ = get_parameter("path_topic").as_string();
        keep_alive_ = get_parameter("keep_alive").as_bool();
        republish_period_sec_ = get_parameter("republish_period_sec").as_double();

        auto qos_latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        pub_poly_ = create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(poly_topic_, qos_latched);
        pub_path_ = create_publisher<visualization_msgs::msg::MarkerArray>(path_topic_, qos_latched);

        if (republish_period_sec_ > 1e-6)
        {
            repub_timer_ = create_wall_timer(
                std::chrono::duration<double>(republish_period_sec_),
                std::bind(&CorridorGeneratorNode::republishCachedMsgs, this));
        }

        // RCLCPP_INFO(get_logger(),
        //             "CorridorGeneratorNode up. Waiting for point cloud on [%s].", map_topic_.c_str());
    }

private:
    void mapCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        {
            std::lock_guard<std::mutex> lk(mtx_);
            last_cloud_ = cloud;
            map_received_ = true;
        }

        // Update DGP voxel map with a fixed window for fairness
        // Dynamic obstacles: pass empty here unless you explicitly want inflation-in-map-update.
        vec_Vecf<3> obst_pos_empty;
        const double traj_max_time = 0.0;

        // Create an empty point cloud for unknowns
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unk(new pcl::PointCloud<pcl::PointXYZ>());

        dgp_.updateMap(wdx_, wdy_, wdz_, map_center_, cloud, cloud_unk, obst_pos_empty, traj_max_time);

        // Also store occupied vector for decomp obstacle set
        dgp_.updateVecOccupied(pclToVec3f(*cloud));
    }

    void tick()
    {
        if (done_)
            return;
        if (!map_received_)
            return;
        if (goals_.empty())
            return;
        if (!dgp_.isMapInitialized())
            return;

        try
        {
            runAllGoalsOnce();
            done_ = true;
            RCLCPP_INFO(get_logger(), "All corridors generated.");

            timer_->cancel(); // stop calling tick()

            if (!keep_alive_)
            {
                RCLCPP_INFO(get_logger(), "keep_alive:=false -> shutting down.");
                rclcpp::shutdown();
            }
            else
            {
                RCLCPP_INFO(get_logger(), "keep_alive:=true -> staying alive for RViz.");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed: %s", e.what());
            rclcpp::shutdown();
        }
    }

    void runAllGoalsOnce()
    {
        fs::create_directories(output_dir_);
        // RCLCPP_INFO(get_logger(), "Saving corridors to: %s", output_dir_.c_str());

        // Setup planner snapshot (locks current map_util_ into map_util_for_planning_)
        dgp_.setupDGPPlanner(
            global_planner_,
            global_planner_verbose_,
            par_.res,
            v_max_,
            a_max_,
            j_max_,
            dgp_timeout_ms_,
            /*w_unknown*/ 0.0, /*w_align*/ 0.0, /*decay_len_cells*/ 100.0, /*w_side*/ 0.0,
            /*los_cells*/ 0, /*min_len*/ 0.5, /*min_turn*/ 0.0);

        // Base obstacle set for decomposition: occupied (or unknown+occupied if you maintain it)
        vec_Vec3f base_uo;
        dgp_.getVecOccupied(base_uo);

        // If you want unknown+occupied corridors (gazebo case), you would instead do:
        // dgp_.getVecUnknownOccupied(base_uo);

        for (size_t gi = 0; gi < goals_.size(); ++gi)
        {
            const Vec3f goal = goals_[gi];

            // Direction hint (magnitude irrelevant; used as direction)
            Vec3f dir = goal - start_;
            if (dir.norm() > 1e-6)
                dir = dir / dir.norm();
            else
                dir = Vec3f(1.0, 0.0, 0.0);

            double final_g = 0.0;
            vec_Vecf<3> path;
            vec_Vecf<3> raw_path;

            const double tnow = now().seconds();
            const bool ok = dgp_.solveDGP(start_, dir, goal, final_g, weight_, tnow, path, raw_path);

            if (!ok || path.size() < 2)
            {
                RCLCPP_WARN(get_logger(), "Goal %zu: global planner failed. Skipping.", gi);
                continue;
            }

            // Segment end times for per-segment dynamic inflation
            const auto seg_end_times = computeSegEndTimesFromPath(path, corridor_nominal_speed_);

            // Convex decomposition -> linear constraints + polys
            EllipsoidDecomp3D ellip;
            std::vector<LinearConstraint3D> l_constraints;
            vec_E<Polyhedron<3>> poly_out;

            vec_Vecf<3> obst_pos_empty; // set this if you want dynamic obstacle inflation in corridor
            const bool decomp_ok = dgp_.cvxEllipsoidDecomp(
                ellip, path, base_uo, obst_pos_empty, seg_end_times, l_constraints, poly_out);

            if (!decomp_ok)
            {
                RCLCPP_WARN(get_logger(), "Goal %zu: cvxEllipsoidDecomp failed. Skipping.", gi);
                continue;
            }

            publishCorridorAndPath(gi, path, poly_out, ellip);

            // Save
            const fs::path out = fs::path(output_dir_) /
                                 (output_prefix_ + "_g" + pad_int(gi, 3) + ".mysco2");

            saveCorridorBinary(out, start_, goal, path, seg_end_times, l_constraints);

            RCLCPP_INFO(get_logger(), "Saved goal %zu corridor (segments=%zu)", gi, l_constraints.size());
        }
    }

    static std::string pad_int(int v, int width)
    {
        std::ostringstream oss;
        oss << std::setw(width) << std::setfill('0') << v;
        return oss.str();
    }

    void publishCorridorAndPath(
        size_t goal_idx,
        const vec_Vecf<3> &path,
        const vec_E<Polyhedron<3>> &poly_out,
        const EllipsoidDecomp3D &ellip)
    {
        // 1) Path
        visualization_msgs::msg::MarkerArray path_msg;
        geometry_msgs::msg::Point last_point;
        last_point.x = path[0].x();
        last_point.y = path[0].y();
        last_point.z = path[0].z();

        for (size_t i = 1; i < path.size(); ++i)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = now();
            marker.header.frame_id = frame_id_;
            marker.ns = "dgp_path";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.points.push_back(last_point);
            geometry_msgs::msg::Point curr_point;
            curr_point.x = path[i].x();
            curr_point.y = path[i].y();
            curr_point.z = path[i].z();
            marker.points.push_back(curr_point);
            last_point = curr_point;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

            path_msg.markers.push_back(marker);
        }

        // 2) Polyhedra corridor (decomp_rviz_plugins-compatible)
        decomp_ros_msgs::msg::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly_out);
        poly_msg.header.stamp = now();
        poly_msg.header.frame_id = frame_id_;
        poly_msg.lifetime = rclcpp::Duration::from_seconds(1.0); // infinite

        // Publish
        pub_path_->publish(path_msg);
        pub_poly_->publish(poly_msg);

        // Cache for periodic republish
        cached_path_msg_ = path_msg;
        cached_poly_msg_ = poly_msg;
        have_cached_path_ = true;
        have_cached_poly_ = true;

        // RCLCPP_INFO(get_logger(),
        //             "Published corridor+path for goal %zu: path_pts=%zu polys=%zu",
        //             goal_idx, path.size(), poly_out.size());
    }

    void republishCachedMsgs()
    {
        if (!done_)
            return; // only republish after generation

        if (have_cached_path_)
            pub_path_->publish(cached_path_msg_);
        if (have_cached_poly_)
            pub_poly_->publish(cached_poly_msg_);
    }

private:
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    std::mutex mtx_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud_;
    bool map_received_{false};
    bool done_{false};

    // Config
    std::string map_topic_;
    std::string output_dir_;
    std::string output_prefix_;

    Vec3f start_;
    std::vector<Vec3f> goals_;

    Vec3f map_center_;
    double wdx_{200.0}, wdy_{200.0}, wdz_{10.0};

    double corridor_nominal_speed_{2.0};

    // Planner config
    parameters par_;
    DGPManager dgp_;

    std::string global_planner_{"sjps"};
    bool global_planner_verbose_{false};
    double weight_{1.0};
    int dgp_timeout_ms_{1000};

    double v_max_{2.0}, a_max_{3.0}, j_max_{10.0};

    // RViz publishing
    std::string frame_id_{"world"};
    std::string poly_topic_{"/mighty/sfc_poly"};
    std::string path_topic_{"/mighty/sfc_path"};
    bool keep_alive_{true};
    double republish_period_sec_{1.0};

    rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_path_;
    rclcpp::Publisher<decomp_ros_msgs::msg::EllipsoidArray>::SharedPtr pub_ellip_;
    rclcpp::TimerBase::SharedPtr repub_timer_;

    // Cached messages
    visualization_msgs::msg::MarkerArray cached_path_msg_;
    decomp_ros_msgs::msg::PolyhedronArray cached_poly_msg_;
    decomp_ros_msgs::msg::EllipsoidArray cached_ellip_msg_;
    bool have_cached_path_{false};
    bool have_cached_poly_{false};
    bool have_cached_ellip_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
