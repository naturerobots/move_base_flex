/*
 *  Copyright 2023, Rapyuta Robotics Co., Ltd., Renan Salles
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  free_pose_search_test.cpp
 *
 *  authors:
 *    Renan Salles <renan028@gmail.com>
 *
 */

// ros
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/testing_helper.h>

// mbf
#include "mbf_costmap_nav/free_pose_search.h"
#include "mbf_costmap_nav/costmap_navigation_server.h"

#include <tf2/utils.h>
namespace mbf_costmap_nav::test
{
class SearchHelperTest : public ::testing::Test
{
protected:
  boost::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
  tf2_ros::TransformListener tf_listener;
  costmap_2d::Costmap2D costmap;
  ros::Subscriber map_sub;
  nav_msgs::OccupancyGrid map;
  ros::Publisher map_pub;

  SearchHelperTest()
    : tf_buffer_ptr(boost::make_shared<tf2_ros::Buffer>())
    , tf_listener(*tf_buffer_ptr)
    , costmap(10, 10, 1.0, 0.0, 0.0, 0)
  {
    tf_buffer_ptr->setUsingDedicatedThread(true);
  }

  void SetUp() override
  {
    if (!tf_buffer_ptr->canTransform("base_link", "map", ros::Time::now(), ros::Duration(5.0)))
    {
      FAIL() << "Cannot transform from base_link to map";
    }
    map_sub = ros::NodeHandle().subscribe("map", 1, &SearchHelperTest::mapCb, this);
    map_pub = ros::NodeHandle().advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  }

  void mapCb(const nav_msgs::OccupancyGrid& m)
  {
    map = m;
  }

  geometry_msgs::Point toPoint(double x, double y)
  {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    return point;
  }

  geometry_msgs::Pose2D toPose2D(double x, double y, double theta)
  {
    geometry_msgs::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    return pose;
  }

  void addObstacle(costmap_2d::Costmap2DROS& cm, double x, double y)
  {
    boost::shared_ptr<costmap_2d::ObstacleLayer> olayer;
    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = cm.getLayeredCostmap()->getPlugins();
    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin();
         pluginp != plugins->end(); ++pluginp)
    {
      boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
      if (plugin->getName().find("obstacles") != std::string::npos)
      {
        olayer = boost::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
        addObservation(&(*olayer), x, y, MAX_Z / 2, 0, 0, MAX_Z / 2);
      }
    }

    cm.updateMap();
    printMap(*(cm.getCostmap()));
  }
};

TEST_F(SearchHelperTest, getNeighbors)
{
  Cell cell{ 0, 0 };
  auto neighbors = FreePoseSearch::getNeighbors(costmap, cell);
  ASSERT_EQ(neighbors.size(), 3);
  EXPECT_EQ(neighbors[0].x, 0);
  EXPECT_EQ(neighbors[0].y, 1);
  EXPECT_EQ(neighbors[1].x, 1);
  EXPECT_EQ(neighbors[1].y, 0);
  EXPECT_EQ(neighbors[2].x, 1);
  EXPECT_EQ(neighbors[2].y, 1);

  cell = { 9, 9 };
  neighbors = FreePoseSearch::getNeighbors(costmap, cell);
  ASSERT_EQ(neighbors.size(), 3);
  EXPECT_EQ(neighbors[0].x, 8);
  EXPECT_EQ(neighbors[0].y, 8);
  EXPECT_EQ(neighbors[1].x, 8);
  EXPECT_EQ(neighbors[1].y, 9);
  EXPECT_EQ(neighbors[2].x, 9);
  EXPECT_EQ(neighbors[2].y, 8);

  cell = { 5, 5 };
  neighbors = FreePoseSearch::getNeighbors(costmap, cell);
  ASSERT_EQ(neighbors.size(), 8);
  EXPECT_EQ(neighbors[0].x, 4);
  EXPECT_EQ(neighbors[0].y, 4);
  EXPECT_EQ(neighbors[1].x, 4);
  EXPECT_EQ(neighbors[1].y, 5);
  EXPECT_EQ(neighbors[2].x, 4);
  EXPECT_EQ(neighbors[2].y, 6);
  EXPECT_EQ(neighbors[3].x, 5);
  EXPECT_EQ(neighbors[3].y, 4);
  EXPECT_EQ(neighbors[4].x, 5);
  EXPECT_EQ(neighbors[4].y, 6);
  EXPECT_EQ(neighbors[5].x, 6);
  EXPECT_EQ(neighbors[5].y, 4);
  EXPECT_EQ(neighbors[6].x, 6);
  EXPECT_EQ(neighbors[6].y, 5);
  EXPECT_EQ(neighbors[7].x, 6);
  EXPECT_EQ(neighbors[7].y, 6);

  cell = { 0, 5 };
  neighbors = FreePoseSearch::getNeighbors(costmap, cell);
  ASSERT_EQ(neighbors.size(), 5);
  EXPECT_EQ(neighbors[0].x, 0);
  EXPECT_EQ(neighbors[0].y, 4);
  EXPECT_EQ(neighbors[1].x, 0);
  EXPECT_EQ(neighbors[1].y, 6);
  EXPECT_EQ(neighbors[2].x, 1);
  EXPECT_EQ(neighbors[2].y, 4);
  EXPECT_EQ(neighbors[3].x, 1);
  EXPECT_EQ(neighbors[3].y, 5);
  EXPECT_EQ(neighbors[4].x, 1);
  EXPECT_EQ(neighbors[4].y, 6);

  cell = { 5, 0 };
  neighbors = FreePoseSearch::getNeighbors(costmap, cell);
  ASSERT_EQ(neighbors.size(), 5);
  EXPECT_EQ(neighbors[0].x, 4);
  EXPECT_EQ(neighbors[0].y, 0);
  EXPECT_EQ(neighbors[1].x, 4);
  EXPECT_EQ(neighbors[1].y, 1);
  EXPECT_EQ(neighbors[2].x, 5);
  EXPECT_EQ(neighbors[2].y, 1);
  EXPECT_EQ(neighbors[3].x, 6);
  EXPECT_EQ(neighbors[3].y, 0);
  EXPECT_EQ(neighbors[4].x, 6);
  EXPECT_EQ(neighbors[4].y, 1);
}

TEST_F(SearchHelperTest, safetyPadding)
{
  costmap_2d::Costmap2DROS cm("unpadded", *tf_buffer_ptr);

  auto footprint = FreePoseSearch::safetyPadding(cm, true, 0.1);
  EXPECT_EQ(3, footprint.size());
  EXPECT_NEAR(1.1f, footprint[0].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[0].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[1].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[1].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].x, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].y, 1e-6);

  footprint = FreePoseSearch::safetyPadding(cm, false, 0.1);
  EXPECT_EQ(3, footprint.size());
  EXPECT_NEAR(1.1f, footprint[0].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[0].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[1].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[1].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].x, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].y, 1e-6);

  costmap_2d::Costmap2DROS cm2("padded", *tf_buffer_ptr);
  footprint = FreePoseSearch::safetyPadding(cm2, true, 0.1);
  EXPECT_EQ(3, footprint.size());
  EXPECT_NEAR(1.6f, footprint[0].x, 1e-6);
  EXPECT_NEAR(1.6f, footprint[0].y, 1e-6);
  EXPECT_NEAR(-1.6f, footprint[1].x, 1e-6);
  EXPECT_NEAR(1.6f, footprint[1].y, 1e-6);
  EXPECT_NEAR(-1.6f, footprint[2].x, 1e-6);
  EXPECT_NEAR(-1.6f, footprint[2].y, 1e-6);

  footprint = FreePoseSearch::safetyPadding(cm2, false, 0.1);
  EXPECT_EQ(3, footprint.size());
  EXPECT_NEAR(1.1f, footprint[0].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[0].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[1].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[1].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].x, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].y, 1e-6);
}

TEST_F(SearchHelperTest, getFootprintState)
{
  std::vector<geometry_msgs::Point> footprint = { toPoint(-0.5, -0.5), toPoint(0.5, -0.5), toPoint(0.5, 0.5),
                                                  toPoint(-0.5, 0.5) };

  auto test = [&](std::uint8_t state) {
    EXPECT_EQ(FreePoseSearch::getFootprintState(costmap, footprint, toPose2D(5, 5, 0)).state, state);
    EXPECT_EQ(FreePoseSearch::getFootprintState(costmap, footprint, toPose2D(4.6, 5, 0)).state, state);
    EXPECT_EQ(FreePoseSearch::getFootprintState(costmap, footprint, toPose2D(4.6, 4.6, 0)).state, state);
    EXPECT_EQ(FreePoseSearch::getFootprintState(costmap, footprint, toPose2D(4.6, 4.6, M_PI)).state, state);
    EXPECT_EQ(FreePoseSearch::getFootprintState(costmap, footprint, toPose2D(4.5, 4.5, 0)).state, state);
    EXPECT_EQ(FreePoseSearch::getFootprintState(costmap, footprint, toPose2D(4.4, 4.4, 0)).state, SearchState::FREE);
    EXPECT_EQ(FreePoseSearch::getFootprintState(costmap, footprint, toPose2D(4.5, 4.5, M_PI_4)).state, state);
    EXPECT_EQ(FreePoseSearch::getFootprintState(costmap, footprint, toPose2D(3, 5, 0)).state, SearchState::FREE);
  };

  // Test LETHAL
  costmap.setCost(5, 5, costmap_2d::LETHAL_OBSTACLE);
  test(SearchState::LETHAL);

  // Test NO_INFORMATION
  costmap.setCost(5, 5, costmap_2d::NO_INFORMATION);
  test(SearchState::UNKNOWN);

  // Test INSCRIBED
  costmap.setCost(5, 5, costmap_2d::FREE_SPACE);
  costmap.setCost(5, 6, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  costmap.setCost(6, 5, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  costmap.setCost(5, 4, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  costmap.setCost(4, 5, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(FreePoseSearch::getFootprintState(costmap, footprint, toPose2D(5, 5, 0)).state, SearchState::INSCRIBED);

  // Test OUTSIDE
  EXPECT_EQ(FreePoseSearch::getFootprintState(costmap, footprint, toPose2D(0, 0, 0)).state, SearchState::OUTSIDE);
}

TEST_F(SearchHelperTest, findValidOrientation)
{
  costmap.setCost(5, 5, costmap_2d::LETHAL_OBSTACLE);
  std::optional<FreePoseSearchViz> viz;

  // square
  std::vector<geometry_msgs::Point> footprint = { toPoint(-0.5, -0.5), toPoint(0.5, -0.5), toPoint(0.5, 0.5),
                                                  toPoint(-0.5, 0.5) };
  auto sol = FreePoseSearch::findValidOrientation(costmap, footprint, toPose2D(4.6, 4.6, 0), { M_PI_4 / 2, M_PI }, viz);
  EXPECT_EQ(sol.search_state.state, SearchState::LETHAL);

  sol = FreePoseSearch::findValidOrientation(costmap, footprint, toPose2D(4.6, 4.6, 0), { M_PI, 2 * M_PI }, viz);
  EXPECT_EQ(sol.search_state.state, SearchState::LETHAL);

  // rectangle
  footprint = { toPoint(-0.5, -0.4), toPoint(1.0, -0.4), toPoint(1.0, 0.4), toPoint(-0.5, 0.4) };

  sol = FreePoseSearch::findValidOrientation(costmap, footprint, toPose2D(4.5, 5.5, 0), { M_PI_4, M_PI }, viz);
  EXPECT_EQ(sol.search_state.state, SearchState::FREE);
  EXPECT_NEAR(sol.pose.theta, M_PI_2, 1e-6);

  sol = FreePoseSearch::findValidOrientation(costmap, footprint, toPose2D(4.5, 5.5, -M_PI_2), { M_PI_4, M_PI }, viz);
  EXPECT_EQ(sol.search_state.state, SearchState::FREE);
  EXPECT_NEAR(sol.pose.theta, -M_PI_2, 1e-6);

  // inscribed
  costmap.setCost(4, 4, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  costmap.setCost(4, 6, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  sol = FreePoseSearch::findValidOrientation(costmap, footprint, toPose2D(4.5, 5.5, 0), { M_PI_4, M_PI_2 }, viz);
  EXPECT_EQ(sol.search_state.state, SearchState::INSCRIBED);
  EXPECT_NEAR(sol.pose.theta, M_PI_2, 1e-6);
}

TEST_F(SearchHelperTest, search)
{
  ros::NodeHandle nh;
  costmap_2d::Costmap2DROS cm("search/global", *tf_buffer_ptr);
  FreePoseSearchViz viz(nh, cm.getGlobalFrameID());

  printMap(*(cm.getCostmap()));
  addObstacle(cm, 5.5, 5.5);
  map.header.stamp = ros::Time::now();
  map.data[cm.getCostmap()->getIndex(5, 5)] = 100;
  map_pub.publish(map);

  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  0    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0     0     0    0     x     0     0
  4.5 |  0    0    0     0     0     0    0     0     0     0
  5.5 |  0    0    0     0    254    G    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0     0    0    254   254   254
  8.5 |  0    0    0     0     0     0    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */

  SearchConfig config{ M_PI_4, M_PI, 5.0, false, 0.0, toPose2D(5.5, 5.5, 0) };
  FreePoseSearch sh(cm, config, std::nullopt, viz);

  auto sol = sh.search();
  EXPECT_EQ(sol.search_state.state, SearchState::FREE);
  EXPECT_NEAR(sol.pose.x, 7.5, 1e-6);
  EXPECT_NEAR(sol.pose.y, 3.5, 1e-6);
  EXPECT_NEAR(sol.pose.theta, 0, 1e-6);

  addObstacle(cm, 7.5, 3.5);
  map.header.stamp = ros::Time::now();
  map.data[cm.getCostmap()->getIndex(7, 3)] = 100;
  map_pub.publish(map);

  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  0    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0    254    0    0    254    0     0
  4.5 |  0    0    0     0     0     0    0     0     0     0
  5.5 |  0    0    x     0    254    G    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0     0    0    254   254   254
  8.5 |  0    0    0     0     0     0    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */

  sol = sh.search();
  EXPECT_EQ(sol.search_state.state, SearchState::FREE);
  EXPECT_NEAR(sol.pose.x, 2.5, 1e-6);
  EXPECT_NEAR(sol.pose.y, 5.5, 1e-6);
  EXPECT_NEAR(sol.pose.theta, M_PI_2, 1e-6);

  addObstacle(cm, 2.5, 5.5);
  map.header.stamp = ros::Time::now();
  map.data[cm.getCostmap()->getIndex(2, 5)] = 100;
  map_pub.publish(map);

  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  0    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0    254    0    0    254    0     0
  4.5 |  0    0    0     0     0     0    0     0     0     0
  5.5 |  0    0   254    0    254    G    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0     0    0    254   254   254
  8.5 |  0    0    0     0     0     x    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */

  sol = sh.search();
  EXPECT_EQ(sol.search_state.state, SearchState::FREE);
  EXPECT_NEAR(sol.pose.x, 5.5, 1e-6);
  EXPECT_NEAR(sol.pose.y, 8.5, 1e-6);
  EXPECT_NEAR(sol.pose.theta, M_PI, 1e-6);

  addObstacle(cm, 5.5, 8.5);
  map.header.stamp = ros::Time::now();
  map.data[cm.getCostmap()->getIndex(5, 8)] = 100;
  map_pub.publish(map);

  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  0    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0    254    0    0    254    0     0
  4.5 |  0    0    0     0     0     0    0     0     0     0
  5.5 |  0    0   254    0    254    G    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0     0    0    254   254   254
  8.5 |  0    0    0     x     0    254   0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */

  sol = sh.search();
  EXPECT_EQ(sol.search_state.state, SearchState::FREE);
  EXPECT_NEAR(sol.pose.x, 3.5, 1e-6);
  EXPECT_NEAR(sol.pose.y, 8.5, 1e-6);
  EXPECT_NEAR(sol.pose.theta, -3.0 / 4 * M_PI, 1e-6);

  // Non-zero orientation
  SearchConfig config2{ M_PI_4, M_PI, 5.0, false, 0.0, toPose2D(1.5, 4.5, M_PI_4) };
  FreePoseSearch sh2(cm, config2, std::nullopt, viz);
  sol = sh2.search();
  EXPECT_EQ(sol.search_state.state, SearchState::FREE);
  EXPECT_NEAR(sol.pose.x, 1.5, 1e-6);
  EXPECT_NEAR(sol.pose.y, 3.5, 1e-6);
  EXPECT_NEAR(sol.pose.theta, 3.0 / 2 * M_PI, 1e-6);
}

TEST_F(SearchHelperTest, service_test)
{
  CostmapNavigationServer server(tf_buffer_ptr);

  ros::ServiceClient client = ros::NodeHandle("~").serviceClient<mbf_msgs::FindValidPose>("find_valid_pose");
  mbf_msgs::FindValidPose::Request req;
  mbf_msgs::FindValidPose::Response res;

  req.angle_tolerance = M_PI_4;
  req.dist_tolerance = 1.0;
  req.use_padded_fp = false;
  req.costmap = mbf_msgs::FindValidPose::Request::GLOBAL_COSTMAP;
  req.pose.header.frame_id = "map";
  req.pose.header.stamp = ros::Time::now();
  req.pose.pose.position.x = 1.525;
  req.pose.pose.position.y = 4.525;
  req.pose.pose.position.z = 0.0;
  req.pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI_4);

  ASSERT_TRUE(client.call(req, res));
  EXPECT_EQ(res.state, mbf_msgs::FindValidPose::Response::FREE);
  EXPECT_NEAR(res.pose.pose.position.x, 1.525, 1e-6);
  EXPECT_NEAR(res.pose.pose.position.y, 4.525, 1e-6);
  EXPECT_NEAR(tf::getYaw(res.pose.pose.orientation), M_PI_4, 1e-6);
  server.stop();
}

TEST_F(SearchHelperTest, enforce_bounds_no_tolerance)
{
  ros::NodeHandle nh;
  costmap_2d::Costmap2DROS cm("search/global", *tf_buffer_ptr);
  FreePoseSearchViz viz(nh, cm.getGlobalFrameID());

  printMap(*(cm.getCostmap()));

  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ----G-------------------------------------------------------
  0.5 |  0    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0     0     0    0     0     0     0
  4.5 |  0    0    0     0     0     0    0     0     0     0
  5.5 |  0    0    0     0    254    0    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0     0    0    254   254   254
  8.5 |  0    0    0     0     0     0    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */

  SearchConfig config{ M_PI_4, M_PI, 0.5, false, 0.0, toPose2D(-1, -1, 0) };
  FreePoseSearch sh(cm, config, std::nullopt, viz);

  auto sol = sh.search();
  EXPECT_EQ(sol.search_state.state, SearchState::LETHAL);
}

TEST_F(SearchHelperTest, enforce_bounds)
{
  ros::NodeHandle nh;
  costmap_2d::Costmap2DROS cm("search/global", *tf_buffer_ptr);
  FreePoseSearchViz viz(nh, cm.getGlobalFrameID());

  printMap(*(cm.getCostmap()));

  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  G    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0     0     0    0     0     0     0
  4.5 |  0    0    0     0     0     0    0     0     0     0
  5.5 |  0    0    0     0    254    0    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0     0    0    254   254   254
  8.5 |  0    0    0     0     0     0    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */

  SearchConfig config{ M_PI_4, M_PI, std::sqrt(2 * 1.5 * 1.5), false, 0.0, toPose2D(-1, -1, 0) };
  FreePoseSearch sh(cm, config, std::nullopt, viz);

  auto sol = sh.search();
  EXPECT_EQ(sol.search_state.state, SearchState::OUTSIDE);
  EXPECT_NEAR(sol.pose.x, 0.5, 1e-6);
  EXPECT_NEAR(sol.pose.y, 0.5, 1e-6);
  EXPECT_NEAR(sol.pose.theta, 0, 1e-6);
}

TEST_F(SearchHelperTest, enforce_bounds_within_tolerance)
{
  ros::NodeHandle nh;
  costmap_2d::Costmap2DROS cm("search/global", *tf_buffer_ptr);
  FreePoseSearchViz viz(nh, cm.getGlobalFrameID());

  printMap(*(cm.getCostmap()));
  addObstacle(cm, 0.5, 2.5);
  addObstacle(cm, 2.5, 0.5);
  addObstacle(cm, 1.5, 1.5);

  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  G    0   254    0     0     0    0    254   254   254
  1.5 |  0   254   0     0     0     0    0    254   254   254
  2.5 |  254  0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0     0     0    0     0     0     0
  4.5 |  0    0    0     0     0     0    0     0     0     0
  5.5 |  0    0    0     0    254    0    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0     0    0    254   254   254
  8.5 |  0    0    0     0     0     0    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */

  SearchConfig config{ M_PI_4, M_PI, 3.0, false, 0.0, toPose2D(-1, -1, 0) };
  FreePoseSearch sh(cm, config, std::nullopt, viz);

  auto sol = sh.search();
  EXPECT_EQ(sol.search_state.state, SearchState::OUTSIDE);
  EXPECT_NEAR(sol.pose.x, 0.5, 1e-6);
  EXPECT_NEAR(sol.pose.y, 0.5, 1e-6);
  EXPECT_NEAR(sol.pose.theta, 0, 1e-6);
}

TEST_F(SearchHelperTest, goal_not_centered)
{
  ros::NodeHandle nh;
  costmap_2d::Costmap2DROS cm("search/global", *tf_buffer_ptr);
  FreePoseSearchViz viz(nh, cm.getGlobalFrameID());

  printMap(*(cm.getCostmap()));
  map.header.stamp = ros::Time::now();
  map_pub.publish(map);

  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  0    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0     0     0    0     0     0     0
  4.5 |  0    0    0     0     0     0    0     0     0     0
  5.5 |  0    0    0     0    254    0    0    254   254   254
  6.5 |  0    G    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0     0    0    254   254   254
  8.5 |  0    0    0     0     0     0    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */

  SearchConfig config{ M_PI_4, M_PI, 5.0, false, 0.0, toPose2D(1.345, 6.66, 0) };
  FreePoseSearch sh(cm, config, std::nullopt, viz);

  auto sol = sh.search();
  EXPECT_EQ(sol.search_state.state, SearchState::FREE);
  EXPECT_EQ(sol.pose.x, 1.345);
  EXPECT_EQ(sol.pose.y, 6.66);
  EXPECT_EQ(sol.pose.theta, 0);
}

TEST_F(SearchHelperTest, goal_not_centered_small_tolerance)
{
  ros::NodeHandle nh;
  costmap_2d::Costmap2DROS cm("search/global", *tf_buffer_ptr);
  FreePoseSearchViz viz(nh, cm.getGlobalFrameID());

  printMap(*(cm.getCostmap()));
  map.header.stamp = ros::Time::now();
  map_pub.publish(map);

  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  0    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0     0     0    0     0     0     0
  4.5 |  0    0    0     0     0     0    0     0     0     0
  5.5 |  0    0    0     0    254    0    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0     G    0    254   254   254
  8.5 |  0    0    0     0     0     0    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */

  // tolerance is less than cell resolution
  SearchConfig config{ M_PI_4, M_PI, 0.1, false, 0.0, toPose2D(5.5, 7.1, 0) };
  FreePoseSearch sh(cm, config, std::nullopt, viz);

  // goal pose is not valid, but the goal cell is valid (5.5, 7.5, 0.785398); however that is above tolerance (0.1)
  auto sol = sh.search();
  EXPECT_EQ(sol.search_state.state, SearchState::LETHAL);
}

TEST_F(SearchHelperTest, service_zero_tolerance_test)
{
  CostmapNavigationServer server(tf_buffer_ptr);

  ros::ServiceClient client = ros::NodeHandle("~").serviceClient<mbf_msgs::FindValidPose>("find_valid_pose");
  mbf_msgs::FindValidPose::Request req;
  mbf_msgs::FindValidPose::Response res;

  req.angle_tolerance = 0;
  req.dist_tolerance = 0;
  req.use_padded_fp = false;
  req.costmap = mbf_msgs::FindValidPose::Request::GLOBAL_COSTMAP;
  req.pose.header.frame_id = "map";
  req.pose.header.stamp = ros::Time::now();
  req.pose.pose.position.x = 1.5345;
  req.pose.pose.position.y = 4.666;
  req.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.001);

  ASSERT_TRUE(client.call(req, res));
  EXPECT_EQ(res.state, mbf_msgs::FindValidPose::Response::FREE);
  EXPECT_EQ(res.pose.pose.position.x, 1.5345);
  EXPECT_EQ(res.pose.pose.position.y, 4.666);
  EXPECT_EQ(tf2::getYaw(res.pose.pose.orientation), tf2::getYaw(req.pose.pose.orientation));
  server.stop();
}
}  // namespace mbf_costmap_nav::test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "search_helper_test");
  // enable debug logging for tests
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  ros::AsyncSpinner spinner(0);
  spinner.start();
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  spinner.stop();
  return result;
}
