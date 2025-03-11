/**
 * @file cloud_ui_publisher.cc
 * @author Jiazheng Wu (wujiazheng2020@163.com)
 * @brief for cloud ui test
 * @version 0.1
 * @date 2023-08-17
 *
 *
 */

#include <google/protobuf/util/json_util.h>

#include <fstream>

#include "tros_base/tros_base.h"

void SetTransformation(tros::tros_base::geometry::TransformationFrame* tf,
                       double x, double y, double z) {
  tf->set_time_stamp(tros::tros_base::GetTimeSystem()
                         .GetRelativeTime());  // set current time as timestamp
  tf->set_parent_frame("world");  // assuming "world" as the parent frame, can
                                  // be modified as needed
  tf->set_now_frame("shape_frame");  // can be set dynamically if each shape has
                                     // a unique frame

  // Create a 4x4 transformation matrix with translation
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation(0, 3) = x;
  transformation(1, 3) = y;
  transformation(2, 3) = z;

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      tf->add_tf_to_parent(transformation(i, j));
    }
  }
}

void SetColor(tros::tros_base::geometry::Shape* shape, float a, float r,
              float g, float b) {
  shape->set_apparent(a);
  shape->add_color_rgb(r);
  shape->add_color_rgb(g);
  shape->add_color_rgb(b);
}

constexpr bool kReadFromFile = true;
constexpr bool kReadFromJson = false;
constexpr bool kReCreateScene = false;

int main() {
  tros::tros_base::TROSInit("../tests/data/param_file.yaml",
                            "../tests/data/data_graph_setting.yaml",
                            "../tests/data/network_cloud_ui.yaml", 0);

  tros::tros_base::NetworkBridge::Instance().ConfigPublishChannel(
      "scene_channel", false);
  tros::tros_base::NetworkBridge::Instance().ConfigPublishChannel(
      "scene_channel", true);

  for (size_t i = 0; i < 5; ++i) {
    std::shared_ptr<tros::tros_base::geometry::Scene> my_scene =
        std::make_shared<tros::tros_base::geometry::Scene>();

    if (kReadFromFile) {
      if (kReadFromJson) {
        std::ifstream ifs("scene.json");
        std::string input((std::istreambuf_iterator<char>(ifs)),
                          std::istreambuf_iterator<char>());
        google::protobuf::util::JsonStringToMessage(input, my_scene.get());
      } else {
        std::ifstream ifs("scene.bin", std::ios::binary);
        my_scene->ParseFromIstream(&ifs);
        ifs.close();
      }
    }

    if (kReCreateScene) {
      // Point shape
      tros::tros_base::geometry::Shape shape_point;
      shape_point.mutable_point()->mutable_data()->Add(1.0);
      shape_point.mutable_point()->mutable_data()->Add(2.0);
      shape_point.mutable_point()->mutable_data()->Add(3.0);
      shape_point.mutable_point()->set_width(0.1);
      SetTransformation(shape_point.mutable_transformation_frame(), 0.0, 0.0,
                        5.0);
      SetColor(&shape_point, 1.0, 1.0, 0.0, 0.0);

      my_scene->add_shapes()->CopyFrom(shape_point);

      // Line shape
      tros::tros_base::geometry::Shape shape_curve;
      for (int i = -5; i < 5; ++i) {
        auto point = shape_curve.mutable_line()->add_points()->mutable_data();
        point->Add(3 * std::sin(i * 0.1));
        point->Add(3 * std::cos(i * 0.1));
        point->Add(0.0);
      }
      shape_curve.mutable_line()->set_width(0.1);

      SetTransformation(shape_curve.mutable_transformation_frame(), 4.0, -3.0,
                        0.0);
      SetColor(&shape_curve, 1.0, 0.0, 1.0, 0.0);

      my_scene->add_shapes()->CopyFrom(shape_curve);

      tros::tros_base::geometry::Shape shape_line;
      for (int i = -2; i < 2; ++i) {
        auto point = shape_line.mutable_line()->add_points()->mutable_data();
        point->Add(i);
        point->Add(0.0);
        point->Add(0.0);
      }
      shape_line.mutable_line()->set_width(0.1);

      SetTransformation(shape_line.mutable_transformation_frame(), 0.0, 4.0,
                        0.0);
      SetColor(&shape_line, 1.0, 1.0, 0.0, 1.0);

      my_scene->add_shapes()->CopyFrom(shape_line);

      // Polygon shape
      tros::tros_base::geometry::Shape shape_polygon;
      shape_polygon.set_name("pillar");
      auto poly_point1 = shape_polygon.mutable_polygen()->add_points();
      poly_point1->mutable_data()->Add(0.0);
      poly_point1->mutable_data()->Add(0.0);
      poly_point1->mutable_data()->Add(0.0);

      auto poly_point2 = shape_polygon.mutable_polygen()->add_points();
      poly_point2->mutable_data()->Add(1.0);
      poly_point2->mutable_data()->Add(0.0);
      poly_point2->mutable_data()->Add(0.0);

      auto poly_point3 = shape_polygon.mutable_polygen()->add_points();
      poly_point3->mutable_data()->Add(1.0);
      poly_point3->mutable_data()->Add(1.0);
      poly_point3->mutable_data()->Add(0.0);
      SetTransformation(shape_polygon.mutable_transformation_frame(), 4.0, 4.0,
                        0.0);
      SetColor(&shape_polygon, 1.0, 1.0, 1.0, 0.0);

      my_scene->add_shapes()->CopyFrom(shape_polygon);

      // Oval shape
      tros::tros_base::geometry::Shape shape_oval;
      shape_oval.mutable_oval()->set_a(1.0);
      shape_oval.mutable_oval()->set_b(1.5);
      SetTransformation(shape_oval.mutable_transformation_frame(), -4.0, 0.0,
                        0.0);
      SetColor(&shape_oval, 1.0, 0.8, 0.8, 1.0);

      my_scene->add_shapes()->CopyFrom(shape_oval);

      // Sphere shape
      tros::tros_base::geometry::Shape shape_sphere;
      shape_sphere.mutable_sphere()->set_a(1.0);
      shape_sphere.mutable_sphere()->set_b(1.0);
      shape_sphere.mutable_sphere()->set_c(2.0);
      SetTransformation(shape_sphere.mutable_transformation_frame(), 0.0, 0.0,
                        0.0);
      SetColor(&shape_sphere, 1.0, 0.8, 0.4, 0.2);

      my_scene->add_shapes()->CopyFrom(shape_sphere);

      // Cone shape with an oval base
      tros::tros_base::geometry::Shape shape_cone_oval;
      shape_cone_oval.mutable_cone()->mutable_oval()->set_a(1.0);
      shape_cone_oval.mutable_cone()->mutable_oval()->set_b(1.5);
      shape_cone_oval.mutable_cone()->set_height(2.0);
      SetTransformation(shape_cone_oval.mutable_transformation_frame(), -4.0,
                        4.0, 0.0);
      SetColor(&shape_cone_oval, 1.0, 0.0, 0.9, 0.5);

      my_scene->add_shapes()->CopyFrom(shape_cone_oval);

      // Cone shape with an polygen base
      tros::tros_base::geometry::Shape shape_cone_polygen;
      auto cone_poly_point1 =
          shape_cone_polygen.mutable_cone()->mutable_polygen()->add_points();
      cone_poly_point1->mutable_data()->Add(-1.0);
      cone_poly_point1->mutable_data()->Add(0.0);
      cone_poly_point1->mutable_data()->Add(0.0);

      auto cone_poly_point2 =
          shape_cone_polygen.mutable_cone()->mutable_polygen()->add_points();
      cone_poly_point2->mutable_data()->Add(1.0);
      cone_poly_point2->mutable_data()->Add(0.0);
      cone_poly_point2->mutable_data()->Add(0.0);

      auto cone_poly_point3 =
          shape_cone_polygen.mutable_cone()->mutable_polygen()->add_points();
      cone_poly_point3->mutable_data()->Add(0.0);
      cone_poly_point3->mutable_data()->Add(2.0);
      cone_poly_point3->mutable_data()->Add(0.0);
      shape_cone_polygen.mutable_cone()->set_height(2.0);
      SetTransformation(shape_cone_polygen.mutable_transformation_frame(), 0.0,
                        -4.0, 0.0);
      SetColor(&shape_cone_polygen, 1.0, 1.0, 0.5, 0.5);

      my_scene->add_shapes()->CopyFrom(shape_cone_polygen);

      // Extrusion shape with oval base
      tros::tros_base::geometry::Shape shape_extrusion_oval;
      shape_extrusion_oval.mutable_extrusion()->mutable_oval()->set_a(1.0);
      shape_extrusion_oval.mutable_extrusion()->mutable_oval()->set_b(1.5);
      shape_extrusion_oval.mutable_extrusion()->set_height(2.0);
      SetTransformation(shape_extrusion_oval.mutable_transformation_frame(),
                        -4.0, -4.0, 0.0);
      SetColor(&shape_extrusion_oval, 1.0, 1.0, 0.2, 0.2);

      my_scene->add_shapes()->CopyFrom(shape_extrusion_oval);

      // Extrusion shape with polygen base
      tros::tros_base::geometry::Shape shape_extrusion_polygen;
      auto extrusion_poly_point1 = shape_extrusion_polygen.mutable_extrusion()
                                       ->mutable_polygen()
                                       ->add_points();
      extrusion_poly_point1->mutable_data()->Add(-1.0);
      extrusion_poly_point1->mutable_data()->Add(0.0);
      extrusion_poly_point1->mutable_data()->Add(0.0);

      auto extrusion_poly_point2 = shape_extrusion_polygen.mutable_extrusion()
                                       ->mutable_polygen()
                                       ->add_points();
      extrusion_poly_point2->mutable_data()->Add(1.0);
      extrusion_poly_point2->mutable_data()->Add(0.0);
      extrusion_poly_point2->mutable_data()->Add(0.0);

      auto extrusion_poly_point3 = shape_extrusion_polygen.mutable_extrusion()
                                       ->mutable_polygen()
                                       ->add_points();
      extrusion_poly_point3->mutable_data()->Add(0.0);
      extrusion_poly_point3->mutable_data()->Add(2.0);
      extrusion_poly_point3->mutable_data()->Add(0.0);
      shape_extrusion_polygen.mutable_extrusion()->set_height(2.0);
      SetTransformation(shape_extrusion_polygen.mutable_transformation_frame(),
                        4.0, -4.0, 0.0);
      SetColor(&shape_extrusion_polygen, 1.0, 0.2, 0.2, 0.9);

      my_scene->add_shapes()->CopyFrom(shape_extrusion_polygen);

      // for point cloud
      tros::tros_base::geometry::Shape shape_pointcloud;
      size_t num_points = 1000;
      double radius = 5;
      for (size_t i = 0; i < num_points; ++i) {
        double angle = 2 * M_PI * i / num_points;
        double x = radius * std::cos(angle);
        double y = radius * std::sin(angle);
        double z = 0.0;  // or whatever height you want

        auto point = shape_pointcloud.mutable_point_cloud()
                         ->add_points()
                         ->mutable_data();
        point->Add(x);
        point->Add(y);
        point->Add(z);
      }
      SetTransformation(shape_pointcloud.mutable_transformation_frame(), 0.0,
                        0.0,
                        6.0);  // set the transformation if required
      SetColor(&shape_pointcloud, 1.0, 1.0, 0.0, 0.0);
      my_scene->add_shapes()->CopyFrom(shape_pointcloud);
    }

    tros::tros_base::ToAny("scene_channel", my_scene);

    // nav msg
    std::shared_ptr<tros::tros_base::navigation::NavigationMsg> nav_msg =
        std::make_shared<tros::tros_base::navigation::NavigationMsg>();

    tros::tros_base::navigation::OccupancyGrid* proto_grid =
        nav_msg->mutable_occupancy_grid();
    proto_grid->set_resolution(0.05);
    proto_grid->set_width(100);
    proto_grid->set_height(100);

    for (size_t y = 0; y < proto_grid->height(); y++) {
      for (size_t x = 0; x < proto_grid->width(); x++) {
        // Assuming that 'data' field is bytes type in proto.
        proto_grid->mutable_data()->push_back(static_cast<char>(y));
      }
    }
    SetTransformation(nav_msg->mutable_transformation_frame(), -2, -2, 2);
    nav_msg->mutable_transformation_frame()->set_now_frame("grid_map");
    tros::tros_base::ToAny("occupany_grid_channel", nav_msg);

    std::shared_ptr<tros::tros_base::navigation::NavigationMsg> nav_msg2 =
        std::make_shared<tros::tros_base::navigation::NavigationMsg>();

    tros::tros_base::navigation::OccupancyVoxel* proto_voxel =
        nav_msg2->mutable_occupancy_voxel();
    proto_voxel->set_resolution(0.1);
    proto_voxel->set_x_width(100);
    proto_voxel->set_y_width(100);
    proto_voxel->set_z_width(100);

    for (size_t z = 0; z < proto_voxel->z_width(); z++) {
      for (size_t y = 0; y < proto_voxel->y_width(); y++) {
        for (size_t x = 0; x < proto_voxel->x_width(); x++) {
          proto_voxel->mutable_data()->push_back(static_cast<char>(z));
        }
      }
    }
    SetTransformation(nav_msg2->mutable_transformation_frame(), -5, -5, -20);
    nav_msg2->mutable_transformation_frame()->set_now_frame("voxel_map");
    tros::tros_base::ToAny("occupany_voxel_channel", nav_msg2);

    // save
    if (i == 0) {
      std::string output;
      google::protobuf::util::MessageToJsonString(*my_scene, &output);
      std::ofstream ofs("scene.json");
      ofs << output;
      ofs.close();

      std::ofstream ofis("scene.bin", std::ios::binary);
      my_scene->SerializeToOstream(&ofis);
      ofis.close();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  int div_part = 1000;

  for (int i = 0; i < div_part; ++i) {
    Eigen::Isometry3d tf_translation = Eigen::Isometry3d::Identity();
    double theta = 2 * i * M_PI / div_part;
    double x = 8 * std::cos(theta);
    double y = 8 * std::sin(theta);
    tf_translation.translate(Eigen::Vector3d(x, y, 0));

    Eigen::Vector3d tangent(-std::sin(theta), std::cos(theta), 0);
    Eigen::Vector3d up(0, 0, 1);  // Up direction
    Eigen::Quaterniond rotation =
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1, 0, 0), tangent);
    tf_translation.rotate(rotation);

    std::shared_ptr<tros::tros_base::TransformationFrame> tf_frame =
        std::make_shared<tros::tros_base::TransformationFrame>(
            0, "world", "robot", tf_translation);
    tros::tros_base::ToAny("tf_channel", tf_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::this_thread::sleep_for(std::chrono::seconds(50));
  tros::tros_base::NetworkBridge::Instance().Stop();
}