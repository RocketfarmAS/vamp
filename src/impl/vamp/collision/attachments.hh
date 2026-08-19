#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <type_traits>
#include <vamp/collision/shapes.hh>
#include <vector>

namespace vamp::collision
{
    template <typename DataT>
    struct Attachment
    {
        Attachment(const Eigen::Transform<DataT, 3, Eigen::Isometry> &tf) noexcept : tf(std::move(tf))
        {
        }

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Eigen::Transform<float, 3, Eigen::Isometry> &tf) noexcept
          : Attachment(tf.cast<DataT>())
        {
        }

        Attachment(const Attachment &) = default;

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Attachment<float> &o) noexcept : Attachment(o.tf)
        {
            spheres.reserve(o.spheres.size());
            for (const auto &sphere : o.spheres)
            {
                spheres.emplace_back(sphere);
            }

            cuboids.reserve(o.cuboids.size());
            for (const auto &cuboid : o.cuboids)
            {
                cuboids.emplace_back(cuboid);
            }
        }

        std::vector<Sphere<DataT>> spheres;
        // Attached geometry that a sphere fill only approximates — a carried box covers its
        // own corners exactly. Leaving this empty keeps the attachment sphere-only.
        std::vector<Cuboid<DataT>> cuboids;
        // HACK: To get around passing the environment as const but needing to re-pose the
        // attachments
        mutable std::vector<Sphere<DataT>> posed_spheres;
        mutable std::vector<Cuboid<DataT>> posed_cuboids;
        Eigen::Transform<DataT, 3, Eigen::Isometry> tf;

        inline void pose(const Eigen::Transform<DataT, 3, Eigen::Isometry> &p_tf) const noexcept
        {
            const auto &n_tf = p_tf * tf;

            posed_spheres.resize(spheres.size());
            for (auto i = 0U; i < spheres.size(); ++i)
            {
                const auto &s = spheres[i];
                Eigen::Matrix<DataT, 3, 1> sp(s.x, s.y, s.z);
                auto tfs = n_tf * sp;
                posed_spheres[i] = Sphere<DataT>(tfs[0], tfs[1], tfs[2], s.r);
            }

            posed_cuboids.resize(cuboids.size());
            for (auto i = 0U; i < cuboids.size(); ++i)
            {
                const auto &c = cuboids[i];
                Eigen::Matrix<DataT, 3, 1> center(c.x, c.y, c.z);
                Eigen::Matrix<DataT, 3, 1> a_1(c.axis_1_x, c.axis_1_y, c.axis_1_z);
                Eigen::Matrix<DataT, 3, 1> a_2(c.axis_2_x, c.axis_2_y, c.axis_2_z);
                Eigen::Matrix<DataT, 3, 1> a_3(c.axis_3_x, c.axis_3_y, c.axis_3_z);

                const auto tfc = n_tf * center;
                const auto tfa_1 = n_tf.linear() * a_1;
                const auto tfa_2 = n_tf.linear() * a_2;
                const auto tfa_3 = n_tf.linear() * a_3;

                // Assigned field-wise rather than constructed: the constructor would also
                // compute a distance to the world origin, which means nothing for geometry
                // that rides along with the robot and is never sorted against the world.
                auto &posed = posed_cuboids[i];
                posed.x = tfc[0];
                posed.y = tfc[1];
                posed.z = tfc[2];
                posed.axis_1_x = tfa_1[0];
                posed.axis_1_y = tfa_1[1];
                posed.axis_1_z = tfa_1[2];
                posed.axis_2_x = tfa_2[0];
                posed.axis_2_y = tfa_2[1];
                posed.axis_2_z = tfa_2[2];
                posed.axis_3_x = tfa_3[0];
                posed.axis_3_y = tfa_3[1];
                posed.axis_3_z = tfa_3[2];
                posed.axis_1_r = c.axis_1_r;
                posed.axis_2_r = c.axis_2_r;
                posed.axis_3_r = c.axis_3_r;
            }
        }
    };
}  // namespace vamp::collision
