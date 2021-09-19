#include "Ego.h"

#include <iostream>

Ego::Ego(std::shared_ptr<lazyECS::RenderingSystem> render_sys_in, 
         std::shared_ptr<lazyECS::PhysicsSystem> physics_sys_in,
         const lazyECS::Transform3D& transform_in) : 
    renderSys(render_sys_in), physicsSys(physics_sys_in), ego_trans_(transform_in), lidar_pos_offset{0.0, 0.0, 0.5}
{
    lidar_hor_count = LIDAR_HOR_FOV / LIDAR_HOR_RES + 1;
    lidar_vert_count = LIDAR_VERT_FOV / LIDAR_VERT_RES + 1;
}

void Ego::step() {
    sensor_step();

}

// Bx = 0 + L * cos (Yaw Angle) * sin (Pitch Angle)
// By = 0 + L * sin (Yaw Angle)
// Bz = 0 + L * cos (Yaw Angle) * cos (Pitch Angle)
void Ego::sensor_step() {

    rp3d::Vector3 ray_end;
    float ego_yaw, ego_pitch, ego_roll;
    auto ego_pos = ego_trans_.rp3d_transform.getPosition();

    std::tie(ego_yaw, ego_pitch, ego_roll) = ego_trans_.GetEulerOrientation(); 

    // Update sensor's position based on the ego it's attached
    // We assume the sensor is a point mass without a 3D volume so that the rays emerging from it's origin doesn't hit itself
    const rp3d::Transform sensor_trans(ego_pos + rp3d::Vector3(sin(ego_yaw)*lidar_pos_offset.z,0,cos(ego_yaw)*lidar_pos_offset.z), ego_trans_.rp3d_transform.getOrientation());
    const auto sensor_pos = sensor_trans.getPosition();

    for(int j = 0; j < lidar_vert_count; j++) {
        float vert_ray_angle = -LIDAR_VERT_FOV/2.0f + j * LIDAR_VERT_RES;
        for(int i = 0; i < lidar_hor_count; i++) {
            float hor_ray_angle = -LIDAR_HOR_FOV/2.0f + i * LIDAR_HOR_RES;

            ray_end.x = sensor_pos.x + LIDAR_RANGE * sin(hor_ray_angle*rp3d::PI/180.0f + ego_yaw) * cos(vert_ray_angle*rp3d::PI/180.0f);
            ray_end.y = sensor_pos.y + 1.0 + LIDAR_RANGE * sin(vert_ray_angle*rp3d::PI/180.0f);
            ray_end.z = sensor_pos.z + LIDAR_RANGE * cos(hor_ray_angle*rp3d::PI/180.0f + ego_yaw) * cos(vert_ray_angle*rp3d::PI/180.0f);

            rp3d::Ray ray(sensor_pos, ray_end);
            renderSys->mDebugRays.emplace_back(ray);
            physicsSys->RayCast(ray);
        }
    }

    for(const auto& hit_point : physicsSys->raycast_manager_.hit_points_) {
        // std::cout << hit_point.x << " " << hit_point.y << " " << hit_point.z << std::endl;
        renderSys->mDebugSpheres.emplace_back(lazyECS::RenderingSystem::DebugSphere(hit_point,
                                                            0.05, rp3d::DebugRenderer::DebugColor::BLACK));                
    }

    // TODO:
    // Pack the hit points into pointcloud and publish
    // Abstract away the ROS parts, so that it can run also stand-alone (shared mem?)



    // for(int j = 0; j < 3; j++) {
    //     for(int i = 0; i < 30; i++) {
    //         // rp3d::Ray ray(rp3d::Vector3(0,0,0), rp3d::Vector3(-15+i,0,-30));
    //         rp3d::Ray ray(rp3d::Vector3(ego_pos_.x, ego_pos_.y, ego_pos_.z - 0.3), 
    //                         rp3d::Vector3(ego_pos_.x -15 + i, ego_pos_.y + j*1.0, ego_pos_.z - 0.3 - 30));
    //         renderSys->mDebugRays.emplace_back(ray);
    //         physicsSys->RayCast(ray);
    //     }
    // }
    // for(const auto& hit_point : physicsSys->raycast_manager_.hit_points_) {
    //     // std::cout << hit_point.x << " " << hit_point.y << " " << hit_point.z << std::endl;
    //     renderSys->mDebugSpheres.emplace_back(lazyECS::RenderingSystem::DebugSphere(hit_point,
    //                                                         0.05, rp3d::DebugRenderer::DebugColor::BLACK));                
    // }
}

// void WbLidar::updatePointCloud(int minWidth, int maxWidth) {

//   const int resolution = actualHorizontalResolution();
//   const double w = width();
//   const double time = WbSimulationState::instance()->time() / 1000.0;

//   for (int i = 0; i < actualNumberOfLayers(); ++i) {
//     double phi = 0;
//     if (actualNumberOfLayers() > 1)  // to avoid division by zero
//       phi = verticalFieldOfView() / 2 - i * (verticalFieldOfView() / (actualNumberOfLayers() - 1));
//     const double sinPhi = sin(phi + mCurrentTiltAngle);
//     const double cosPhi = cos(phi + mCurrentTiltAngle);
//     for (int j = minWidth; j < maxWidth; ++j) {
//       double theta = actualFieldOfView() / 2 - j * (actualFieldOfView() / (w - 1));
//       if (isRotating())
//         theta = -((double)j / (double)resolution) * 2 * M_PI;
//       const int index = resolution * i + j;
//       const double r = image[index];
//       lidarPoints[index].x = -r * sin(theta) * cosPhi;
//       lidarPoints[index].y = r * sinPhi;
//       lidarPoints[index].z = -r * cos(theta) * cosPhi;
//       lidarPoints[index].time = (j / w) * (time - mRefreshRate / 1000.0) + (1 - (j / w)) * time;
//       lidarPoints[index].layer_id = i;
//     }
//   }
// }