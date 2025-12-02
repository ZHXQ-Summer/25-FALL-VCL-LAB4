#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/4-Animation/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"
#include <fstream>

namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            ik.JointGlobalRotation[i]=ik.JointGlobalRotation[i-1]*ik.JointLocalRotation[i];
            ik.JointGlobalPosition[i]=ik.JointGlobalPosition[i-1]+ik.JointGlobalRotation[i-1]*ik.JointLocalOffset[i];
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            int n=ik.NumJoints();
            for (int i = n-2; i >=0; i--) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
                glm::vec3 v=glm::normalize(ik.JointGlobalPosition[n-1]-ik.JointGlobalPosition[i]);
                glm::vec3 m=glm::normalize(EndPosition-ik.JointGlobalPosition[i]);
                glm::quat q=glm::rotation(v,m);

                ik.JointGlobalRotation[i] = q * ik.JointGlobalRotation[i];
                if (i == 0) {
                    ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
                }
                else {
                    ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i-1]) * ik.JointGlobalRotation[i];
                }

                ForwardKinematics(ik,i);
        }
        }
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                glm::vec3 t=glm::normalize(ik.JointGlobalPosition[i]-next_position);
                backward_positions[i]=next_position+t*ik.JointOffsetLength[i+1];
                next_position=backward_positions[i];
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                glm::vec3 t=glm::normalize(backward_positions[i+1]-now_position);
                forward_positions[i+1]=now_position+t*ik.JointOffsetLength[i+1];
                now_position=forward_positions[i+1];
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
       /*初始版本：// get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums = 5000;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int index = 0;
        for (int i = 0; i < nums; i++) {
            float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
            float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
            if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
            (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
        }
        custom->resize(index);
        return custom;*/ 
        //subtask 4.2
        /*using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr());
    
        // 读取Python生成的轨迹文件
        std::ifstream file("trajectory.txt");
        if (!file.is_open()) {
        spdlog::error("cant find trajectory.txt");
        return custom;
        }
    
        // 读取点的数量
        int num_points;
        file >> num_points;
        
        // 参数设置
        float scale = 1.0f;        // 缩放因子
        float center_x = 0.0f;     // X方向中心
        float center_z = 0.0f;     // Z方向中心
        float y_height = 0.0f;     // Y方向高度
        
        // 读取每个点
        for (int i = 0; i < num_points; i++) {
            float x, y;
            file >> x >> y;
            
            // 转换坐标：
            // - 归一化坐标 [0,1] 转换为 [-0.5, 0.5]
            // - 缩放并平移到工作空间
            float world_x = center_x - (x - 0.5f) * scale;  // 水平翻转
            float world_z = center_z - (y - 0.5f) * scale;
            
            custom->push_back(glm::vec3(world_x, y_height, world_z));
        }
        
        file.close();
        
        spdlog::info("成功加载 {} 个轨迹点", custom->size());
        return custom;*/
    //Sub-Task 4.1

    using Vec3Arr = std::vector<glm::vec3>;
    
    // 步骤1：密集采样并检测断裂
    std::vector<std::vector<glm::vec3>> segments;  // 多段曲线
    std::vector<glm::vec3> current_segment;
    
    int dense = 50000;
    glm::vec3 last_point(0);
    bool has_last = false;
    float max_gap = 0.05f;  // 最大允许间隔，超过就认为是断裂
    
    for (int i = 0; i < dense; i++) {
        float t = 92.0f * glm::pi<float>() * i / dense;
        float x_val = 1.5e-3f * custom_x(t);
        float y_val = 1.5e-3f * custom_y(t);
        
        // 跳过接近原点的点
        if (std::abs(x_val) < 1e-3 && std::abs(y_val) < 1e-3) {
            // 如果当前段不为空，保存它
            if (!current_segment.empty()) {
                segments.push_back(current_segment);
                current_segment.clear();
                has_last = false;
            }
            continue;
        }
        
        glm::vec3 current_point(1.6f - x_val, 0.0f, y_val - 0.2f);
        
        // 检测是否与上一点间隔过大（可能是曲线跳跃）
        if (has_last && glm::distance(current_point, last_point) > max_gap) {
            // 保存当前段，开始新段
            if (!current_segment.empty()) {
                segments.push_back(current_segment);
                current_segment.clear();
            }
        }
        
        current_segment.push_back(current_point);
        last_point = current_point;
        has_last = true;
    }
    
    // 保存最后一段
    if (!current_segment.empty()) {
        segments.push_back(current_segment);
    }
    
    spdlog::info("检测到 {} 段连续曲线", segments.size());
    
    // 步骤2：对每段独立重采样
    std::shared_ptr<Vec3Arr> custom(new Vec3Arr());
    int target_per_segment = 5000 / segments.size();  // 平均分配采样点
    
    for (const auto& segment : segments) {
        if (segment.size() < 2) continue;
        
        // 计算该段的累积距离
        std::vector<float> cumulative_dist = {0.0f};
        for (size_t i = 1; i < segment.size(); i++) {
            float dist = glm::distance(segment[i], segment[i-1]);
            cumulative_dist.push_back(cumulative_dist.back() + dist);
        }
        float total_dist = cumulative_dist.back();
        
        if (total_dist < 1e-6f) continue;
        
        // 重采样该段
        for (int i = 0; i < target_per_segment; i++) {
            float target_dist = total_dist * i / (target_per_segment - 1);
            
            size_t j = 0;
            while (j < cumulative_dist.size() - 1 && cumulative_dist[j+1] < target_dist) {
                j++;
            }
            
            if (j >= segment.size() - 1) {
                custom->push_back(segment.back());
            } else {
                float t = (target_dist - cumulative_dist[j]) / 
                          (cumulative_dist[j+1] - cumulative_dist[j]);
                glm::vec3 p = glm::mix(segment[j], segment[j+1], t);
                custom->push_back(p);
            }
        }
    }
    
    spdlog::info("均匀重采样得到 {} 点", custom->size());
    return custom;
    }

    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }
    static Eigen::VectorXf grav(MassSpringSystem & system){
        std::vector<glm::vec3> gravity(system.Positions.size(), glm::vec3(0, system.Gravity, 0));
        return glm2eigen(gravity);
    }

    static Eigen::VectorXf damp_force(MassSpringSystem & system, Eigen::VectorXf const & x, Eigen::VectorXf const & v){
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
        std::vector<glm::vec3> vec_x = eigen2glm(x);
        std::vector<glm::vec3> vec_v = eigen2glm(v);
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = vec_x[p1] - vec_x[p0];
            glm::vec3 const v01 = vec_v[p1] - vec_v[p0];
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3 f = system.Damping * glm::dot(v01, e01) * e01;
            forces[p0] += f;
            forces[p1] -= f;
        }
        return glm2eigen(forces);
    }

    static Eigen::VectorXf grad_E(MassSpringSystem & system, Eigen::VectorXf const & x){
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
        std::vector<glm::vec3> vec_x = eigen2glm(x);
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = vec_x[p1] - vec_x[p0];
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3 f = system.Stiffness * (spring.RestLength - glm::length(x01)) * e01;
            forces[p0] += f;
            forces[p1] -= f;
        }
        return glm2eigen(forces);
    }

    static Eigen::VectorXf y(MassSpringSystem & system, Eigen::VectorXf const & x, Eigen::VectorXf const & v, float const ddt){
        return x + ddt * v + (ddt * ddt / system.Mass) * damp_force(system, x, v) - ddt * ddt * grav(system);
    }

    static Eigen::VectorXf grad_g(MassSpringSystem & system, Eigen::VectorXf const & x, Eigen::VectorXf const & v, float const ddt){
        return (system.Mass / (ddt * ddt)) * (x - y(system, x, v, ddt)) + grad_E(system, x);
    }

    static Eigen::SparseMatrix<float> Hessian(MassSpringSystem & system, Eigen::VectorXf const & x, float const ddt){
        std::vector<Eigen::Triplet<float>> triplets;
        std::vector<glm::vec3> vec_x = eigen2glm(x);
        float m=system.Mass/(ddt*ddt);
        std::vector<glm::mat3> diagonal(system.Positions.size(),glm::mat3(m, 0.0f, 0.0f, 0.0f, m, 0.0f, 0.0f, 0.0f, m));        
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = vec_x[p1] - vec_x[p0];
            float len=glm::length(x01);
            float p;
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    if(i==j) p=system.Stiffness*((x01[i]*x01[j])/(len*len)+(1.0-spring.RestLength/len)*(1.0-(x01[i]*x01[j])/(len*len)));
                    else p=system.Stiffness*((x01[i]*x01[j])/(len*len)+(1.0-spring.RestLength/len)*(0.0-(x01[i]*x01[j])/(len*len)));                    
                    diagonal[p0][i][j]+=p;
                    diagonal[p1][i][j]+=p;
                    triplets.emplace_back(Eigen::Triplet<float>(3*p0+i,3*p1+j,-p));
                    triplets.emplace_back(Eigen::Triplet<float>(3*p1+i,3*p0+j,-p));
                }
            }
        }
        for(std::size_t k=0;k<system.Positions.size();k++){
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    triplets.emplace_back(Eigen::Triplet<float>(3*k+i,3*k+j,diagonal[k][i][j]));
                }
            }
        }
        return CreateEigenSparseMatrix(3*system.Positions.size(),triplets);
    }


    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here: rewrite following code
        int const steps = 10;
        float const ddt = dt / steps; 
        /*
        for (std::size_t s = 0; s < steps; s++) {
            std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
            for (auto const spring : system.Springs) {
                auto const p0 = spring.AdjIdx.first;
                auto const p1 = spring.AdjIdx.second;
                glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
                glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
                glm::vec3 const e01 = glm::normalize(x01);
                glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
                forces[p0] += f;
                forces[p1] -= f;
            }
            for (std::size_t i = 0; i < system.Positions.size(); i++) {
                if (system.Fixed[i]) continue;
                system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
                system.Positions[i] += system.Velocities[i] * ddt;
            }
        }*/
        Eigen::VectorXf x=glm2eigen(system.Positions);
        Eigen::VectorXf v=glm2eigen(system.Velocities);
        for (std::size_t s = 0; s < steps; s++){
            x += ComputeSimplicialLLT(Hessian(system,x,ddt),-grad_g(system,x,v,ddt));
            v += ((damp_force(system,x,v)-grad_E(system,x))/system.Mass-grav(system))*ddt;
        }
        auto newx=eigen2glm(x),newv=eigen2glm(v);
        for (std::size_t i = 0; i < system.Positions.size(); i++) {
            if (system.Fixed[i]) continue;
            system.Velocities[i] = newv[i];
            system.Positions[i]  = newx[i];
        }
    }
 
}
