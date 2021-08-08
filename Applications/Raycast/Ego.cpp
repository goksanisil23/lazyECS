#include "Ego.h"

#include <iostream>

Ego::Ego(std::shared_ptr<lazyECS::RenderingSystem> render_sys_in, 
         std::shared_ptr<lazyECS::PhysicsSystem> physics_sys_in,
         const rp3d::Vector3& ego_pos_in, const rp3d::Quaternion& ego_rot_in) : 
    renderSys(render_sys_in), physicsSys(physics_sys_in),
    ego_pos_(ego_pos_in), ego_rot_(ego_rot_in)
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

    std::tie(ego_yaw, ego_pitch, ego_roll) = quaternion_to_euler(ego_rot_); 

    // std::cout << "yaw: " << ego_yaw/rp3d::PI*180 << " pitch " << ego_pitch/rp3d::PI*180 <<  " roll " << ego_roll/rp3d::PI*180 << std::endl;

    // ego_rotation_.

    for(int j = 0; j < lidar_vert_count; j++) {
        float vert_ray_angle = -LIDAR_VERT_FOV/2.0f + j * LIDAR_VERT_RES;
        for(int i = 0; i < lidar_hor_count; i++) {
            float hor_ray_angle = -LIDAR_HOR_FOV/2.0f + i * LIDAR_HOR_RES;

            ray_end.x = ego_pos_.x + LIDAR_RANGE * sin(hor_ray_angle*rp3d::PI/180.0f) * cos(vert_ray_angle*rp3d::PI/180.0f);
            ray_end.y = ego_pos_.y + LIDAR_RANGE * sin(vert_ray_angle*rp3d::PI/180.0f);
            ray_end.z = ego_pos_.z - 0.3 + LIDAR_RANGE * cos(hor_ray_angle*rp3d::PI/180.0f) * cos(vert_ray_angle*rp3d::PI/180.0f);

            rp3d::Ray ray(rp3d::Vector3(ego_pos_.x, ego_pos_.y, ego_pos_.z - 0.3), ray_end);
            renderSys->mDebugRays.emplace_back(ray);
            physicsSys->RayCast(ray);
        }
    }

    for(const auto& hit_point : physicsSys->raycast_manager_.hit_points_) {
        // std::cout << hit_point.x << " " << hit_point.y << " " << hit_point.z << std::endl;
        renderSys->mDebugSpheres.emplace_back(lazyECS::RenderingSystem::DebugSphere(hit_point,
                                                            0.05, rp3d::DebugRenderer::DebugColor::BLACK));                
    }    



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


std::tuple<float, float, float> Ego::quaternion_to_euler(const rp3d::Quaternion& q) {
    float yaw, pitch, roll;

    float test = q.x*q.y + q.z*q.w;

    if(test > 0.499) { // singularity 
        yaw = 2.0f * atan2(q.x,q.w);
        roll = 0;
        pitch = rp3d::PI/2.0f;
    }
    else if (test < -0.499) {
        yaw = -2.0f * atan2(q.x,q.w);
        roll = 0;
        pitch = -rp3d::PI/2.0f;        
    }
    else {
        yaw = atan2(2*q.y*q.w-2*q.x*q.z , 1 - 2*q.y*q.y - 2*q.z*q.z);
        pitch = asin(2*q.x*q.y + 2*q.z*q.w);
        roll = atan2(2*q.x*q.w-2*q.y*q.z , 1 - 2*q.x*q.x - 2*q.z*q.z);
    }

    return std::make_tuple<float,float,float>(std::move(yaw),std::move(pitch),std::move(roll));

}