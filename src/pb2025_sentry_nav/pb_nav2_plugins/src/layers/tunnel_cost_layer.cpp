 #include "pb_nav2_plugins/tunnel_cost_layer.hpp"
 #include <pluginlib/class_list_macros.hpp>
 #include <cmath>
 #include <rclcpp/rclcpp.hpp>
 namespace pb_nav2_costmap_2d {
 
 void TunnelCostLayer::onInitialize() {
   auto nh = node_->shared_from_this();  // use ROS2 node
   std::vector<int64_t> fwd, rev;
   // direction: 0=entry_to_exit (blocked on exit side), 1=exit_to_entry (blocked on entry side)
   nh->get_parameter(name_ + ".forward_oneway", fwd);
   nh->get_parameter(name_ + ".reverse_oneway", rev);
 
   auto L = [&](const std::string &k) { return nh->get_parameter(name_ + "." + k).as_double_array(); };
   auto tv_xmin = L("tunnel_x_mins"); auto tv_xmax = L("tunnel_x_maxs");
   auto tv_ymin = L("tunnel_y_mins"); auto tv_ymax = L("tunnel_y_maxs");
   auto en_x = L("entry_xs"); auto en_y = L("entry_ys");
   auto ex_x = L("exit_xs");  auto ex_y = L("exit_ys");
 
   size_t n = std::min({tv_xmin.size(), tv_xmax.size(), tv_ymin.size(), tv_ymax.size(),
                        en_x.size(), en_y.size(), ex_x.size(), ex_y.size()});
   for (size_t i = 0; i < n; i++) {
     int dir = 0;  // 0=bidirectional
     if (std::find(fwd.begin(), fwd.end(), (int64_t)i) != fwd.end()) dir = 1;  // forward one-way
     if (std::find(rev.begin(), rev.end(), (int64_t)i) != rev.end()) dir = -1; // reverse one-way
     tunnels_.push_back({tv_xmin[i], tv_xmax[i], tv_ymin[i], tv_ymax[i],
                         en_x[i], en_y[i], ex_x[i], ex_y[i], dir});
   }
   RCLCPP_INFO(rclcpp::get_logger("TunnelCostLayer"), "Loaded %zu tunnels (fwd=%zu rev=%zu)", n, fwd.size(), rev.size());
 }
 
 void TunnelCostLayer::updateBounds(double robot_x, double robot_y, double,
                                    double*, double*, double*, double*) {
   updated_ = false;
   for (auto &t : tunnels_) {
     if (t.oneway_dir == 0) continue;
     double d_en = std::hypot(robot_x - t.entry_x, robot_y - t.entry_y);
     double d_ex = std::hypot(robot_x - t.exit_x,  robot_y - t.exit_y);
     bool on_wrong_side = false;
     if (t.oneway_dir == 1) on_wrong_side = (d_ex < d_en);   // entry→exit, wrong on exit side
     if (t.oneway_dir == -1) on_wrong_side = (d_en < d_ex);  // exit→entry, wrong on entry side
     if (on_wrong_side) {
       mark_x_min_ = t.tx_min - 2; mark_x_max_ = t.tx_max + 2;
       mark_y_min_ = t.ty_min - 2; mark_y_max_ = t.ty_max + 2;
       updated_ = true;
     }
   }
 }
 
 void TunnelCostLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                   int, int, int, int) {
   if (!updated_) return;
   unsigned char lethal = nav2_costmap_2d::LETHAL_OBSTACLE;
   for (auto &t : tunnels_) {
     if (t.oneway_dir == 0) continue;
     unsigned int sx, sy, ex, ey;
     if (!master_grid.worldToMap(t.tx_min, t.ty_min, sx, sy)) continue;
     if (!master_grid.worldToMap(t.tx_max, t.ty_max, ex, ey)) continue;
     for (unsigned int x = sx; x <= ex && x < master_grid.getSizeInCellsX(); x++)
       for (unsigned int y = sy; y <= ey && y < master_grid.getSizeInCellsY(); y++)
         master_grid.setCost(x, y, lethal);
   }
 }
 }
 PLUGINLIB_EXPORT_CLASS(pb_nav2_costmap_2d::TunnelCostLayer, nav2_costmap_2d::Layer)
