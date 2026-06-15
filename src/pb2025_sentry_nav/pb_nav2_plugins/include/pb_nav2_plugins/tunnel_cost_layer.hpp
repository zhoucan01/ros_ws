 #pragma once
 #include "nav2_costmap_2d/layer.hpp"
 namespace pb_nav2_costmap_2d {
 class TunnelCostLayer : public nav2_costmap_2d::Layer {
 public:
   TunnelCostLayer() = default;
   virtual void onInitialize() override;
   virtual void updateBounds(
     double robot_x, double robot_y, double robot_yaw,
     double *min_x, double *min_y, double *max_x, double *max_y) override;
   virtual void updateCosts(
     nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;
   virtual void reset() override {}
   virtual bool isClearable() override { return false; }
 private:
   struct TunnelDef {
     double tx_min, tx_max, ty_min, ty_max;
     double entry_x, entry_y, exit_x, exit_y;
     int oneway_dir;  // 0=bidirectional, 1=entry_exit, -1=exit_entry
   };
   std::vector<TunnelDef> tunnels_;
   bool updated_ = false;
   double mark_x_min_, mark_x_max_, mark_y_min_, mark_y_max_;
 };
 }
