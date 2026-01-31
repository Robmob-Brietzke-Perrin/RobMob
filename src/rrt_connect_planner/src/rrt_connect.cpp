#include "rrt_connect_planner/rrt_connect.hpp"

// ---------------- Tree ----------------

std::shared_ptr<TreeNode> Tree::add_node(Pose pose,
                                         std::shared_ptr<TreeNode> parent) {
  auto node = std::make_shared<TreeNode>(pose, parent);
  nodes_.push_back(node);
  return node;
}

// ---------------- RRTConnect ----------------

Pose RRTConnect::random_config() {
  Pose q_rand;

  // Calculer des facteurs aléatoires entre 0.0 et 1.0
  float rand_factor_x = static_cast<float>(rand()) / RAND_MAX;
  float rand_factor_y = static_cast<float>(rand()) / RAND_MAX;

  // Récupérer les dimensions via le helper
  double min_x = map_helper_.get_min_x();
  double max_x = map_helper_.get_max_x();
  double min_y = map_helper_.get_min_y();
  double max_y = map_helper_.get_max_y();

  // Formule : Min + (Delta * Random)
  // Cela couvre toute la carte, peu importe où est l'origine (même si elle est
  // négative)
  q_rand.position.x = min_x + (max_x - min_x) * rand_factor_x;
  q_rand.position.y = min_y + (max_y - min_y) * rand_factor_y;

  return q_rand;
}

std::shared_ptr<TreeNode> RRTConnect::nearest_neighbor(
    const Pose &point, const Tree &tree)  // checks the nearest neighbor
{
  std::shared_ptr<TreeNode> nearest = nullptr;
  float min_dist = std::numeric_limits<float>::max();

  for (const auto &node : tree.get_nodes())  // checks on all nodes
  {
    float d =
        cart_dist(node->get_pose(), point);  // mesures the length between the
                                             // current point and the TreeNode
    if (d < min_dist) {
      min_dist = d;
      nearest = node;
    }
  }
  return nearest;
}

bool RRTConnect::new_state(
    const Pose &q_target, const Pose &q_near,
    Pose &q_new_out)  // return if the point is in the map or not
{
  float dist = cart_dist(q_target, q_near);

  if (dist < 0.001f) return false;  // Trop proche

  // Si la cible est plus proche que le pas maximum, on l'atteint directement
  if (dist <= STEP_SIZE) {
    q_new_out = q_target;
  } else {
    // Sinon, on avance de STEP_SIZE dans la direction de la cible
    float ratio = STEP_SIZE / dist;
    q_new_out.position.x =
        q_near.position.x + (q_target.position.x - q_near.position.x) * ratio;
    q_new_out.position.y =
        q_near.position.y + (q_target.position.y - q_near.position.y) * ratio;
  }

  return is_valid(q_new_out);
}

RRTConnect::State RRTConnect::extend(Tree &tree, const Pose &q_target,
                                     std::shared_ptr<TreeNode> &new_node_out) {
  auto q_near_node = nearest_neighbor(q_target, tree);
  Pose q_new_pose;

  if (new_state(q_target, q_near_node->get_pose(),
                q_new_pose))  // test if reachable
  {
    new_node_out = tree.add_node(
        q_new_pose, q_near_node);  // adds the point if it is reachable

    if (cart_dist(q_new_pose, q_target) <
        0.1f) {  // tests if it can reach the goal
      return REACHED;
    }
    return ADVANCED;
  }
  return TRAPPED;
}

RRTConnect::State RRTConnect::connect(
    Tree &tree, const Pose &q_target,
    std::shared_ptr<TreeNode> &last_node_out) {
  State s = ADVANCED;
  std::shared_ptr<TreeNode> tmp_node;

  // Boucle avec heuristique gloutonne -> on continue d'étendre tant qu'on
  // avance
  while (s == ADVANCED) {
    s = extend(tree, q_target, tmp_node);
    if (s != TRAPPED) last_node_out = tmp_node;
  }
  return s;
}

nav_msgs::msg::Path RRTConnect::get_path(
    std::shared_ptr<TreeNode> node_start_side,
    std::shared_ptr<TreeNode> node_goal_side) {
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id =
      "map";  // FIXME: is this the right one? Or was only good for testing?

  std::vector<Pose> raw_path;

  // First part: Start to Jonction
  auto curr = node_start_side;
  while (curr !=
         nullptr) {  // tests if we have reached the beginning (no parent)
    raw_path.push_back(curr->get_pose());  // adds the point to the path
    curr = curr->get_parent();             // passes on the next point
  }
  std::reverse(raw_path.begin(),
               raw_path.end());  // puts the path in the right order

  // Second part: Jonction to Goal
  curr = node_goal_side;
  while (curr !=
         nullptr) {  // tests if we have reached the beginning (no parent)
    raw_path.push_back(curr->get_pose());  // adds the point to the path
    curr = curr->get_parent();             // passes on the next point
  }
  // FIXME: is the jonction repeated twice? Would it be problematic? don't think
  // so

  for (const auto &p :
       raw_path) {  // turns the path (list of TreeNodes) in a list of
                    // PoseStamped (correct form for usage)
    PoseStamped ps;
    ps.pose = p;
    path_msg.poses.push_back(ps);
  }

  return path_msg;
}