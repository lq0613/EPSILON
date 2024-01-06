#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>

#include "behavior_planner/map_interface.h"
#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"
#include "route_planner/route_planner.h"

struct Obstacle {
    std::vector<double> st;
    std::vector<double> lt;
};

struct Environment {
    std::vector<Obstacle> obstacles;
    double road_left_barrier;
    double road_right_barrier;
};

struct Parameters {
    double s0;
    double l0;
    // Add other parameters as needed
};

class RRTStar {
public:
    RRTStar(const Parameters& param, const Environment& env);

    ErrorType SearchCoarseTrajectoryViaRRTStar();

public:
  using State = common::State;
//   struct State {
//   decimal_t time_stamp{0.0};
//   Vecf<2> vec_position{Vecf<2>::Zero()};
//   decimal_t angle{0.0};
//   decimal_t curvature{0.0};
//   decimal_t velocity{0.0};
//   decimal_t acceleration{0.0};
//   decimal_t steer{0.0};
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using LateralBehavior = common::LateralBehavior;

    double rand_uniform();
    double SpecifyLongValueRandomly(double time);
    double SpecifyLatValueRandomly(double s_ego, double time);
    std::vector<int> FindParentAndChild(const std::vector<double>& q_new_slt);
    double ComputeDistance(const State& node_a, const State& node_b);
    bool IsV1CollidingWithV2(double s_ego, double l_ego, double s_other, double l_other);
    bool IsSteerCollisionFree(const State& a, const State& b);
    double MeasureDistancePure(const State& node_a, const State& node_b);

private:
    Parameters param_;
    Environment env_;
    std::vector<double> center_right_barrier;
    std::vector<double> center_left_barrier;
    std::vector<double> node_list_slt;
    std::vector<State> node_list;
    planning::BehaviorPlannerMapItf* map_itf_{nullptr};
      // debug
  vec_E<vec_E<common::Vehicle>> forward_trajs_;
  std::vector<LateralBehavior> forward_behaviors_;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;
};

RRTStar::RRTStar(const Parameters& param, const Environment& env)
    : param_(param), env_(env) {
    // Initialize other members as needed
}

ErrorType RRTStar::SearchCoarseTrajectoryViaRRTStar() {
    // Implementation of the RRT* algorithm
      // * get relevant information
  common::SemanticVehicleSet semantic_vehicle_set;
  if (map_itf_->GetKeySemanticVehicles(&semantic_vehicle_set) != kSuccess) {
    printf("[MPDM]fail to get key vehicles.\n");
    return kWrongStatus;
  }

  common::Vehicle ego_vehicle;
  if (map_itf_->GetEgoVehicle(&ego_vehicle) != kSuccess) {
    printf("[MPDM]fail to get ego vehicle.\n");
    return kWrongStatus;
  }
    std::vector<std::vector<double>> path;
    // Your implementation here
    return kSuccess;
}

double RRTStar::rand_uniform() {
    return rand() / static_cast<double>(RAND_MAX);
}

double RRTStar::SpecifyLongValueRandomly(double time) {
    // Implementation of SpecifyLongValueRandomly function
    return 0.0;  // Placeholder, replace with actual logic
}

double RRTStar::SpecifyLatValueRandomly(double s_ego, double time) {
    // Implementation of SpecifyLatValueRandomly function
    return 0.0;  // Placeholder, replace with actual logic
}

std::vector<int> RRTStar::FindParentAndChild(const std::vector<double>& q_new_slt) {
    // Implementation of FindParentAndChild function
    return {};  // Placeholder, replace with actual logic
}

double RRTStar::ComputeDistance(const State& node_a, const State& node_b) {
    // Implementation of ComputeDistance function
    return 0.0;  // Placeholder, replace with actual logic
}

bool RRTStar::IsV1CollidingWithV2(double s_ego, double l_ego, double s_other, double l_other) {
    // Implementation of IsV1CollidingWithV2 function
    return false;  // Placeholder, replace with actual logic
}

bool RRTStar::IsSteerCollisionFree(const State& a, const State& b) {
    // Implementation of IsSteerCollisionFree function
    return false;  // Placeholder, replace with actual logic
}

double RRTStar::MeasureDistancePure(const State& node_a, const State& node_b) {
    // Implementation of MeasureDistancePure function
    return 0.0;  // Placeholder, replace with actual logic
}

int main() {
    Parameters param;
    // Initialize parameters

    Environment env;
    // Initialize environment

    RRTStar rrt(param, env);
   if(rrt.SearchCoarseTrajectoryViaRRTStar() != kSuccess){
        printf("get coarse traj failed");
   }

    // Process the resulting path as needed

    return 0;
}
