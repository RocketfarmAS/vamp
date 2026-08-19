#pragma once

#include <array>

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>

namespace vamp::collision
{
    // Separating-axis test between two oriented boxes, given their axes (indexed
    // [axis][component]), half-extents and the vector between their centers. Returns the
    // widest separation over the 15 candidate axes, so a lane is in collision when the
    // result is negative — the sign convention sphere_cuboid uses, under which boxes that
    // touch exactly (0) read as free.
    //
    // The nine edge-edge axes are left unnormalised, so their separations carry a factor
    // |a_i x b_j|. Only the sign of the maximum is read, and scaling an axis by a positive
    // factor cannot change which axes are positive.
    template <typename DataT>
    inline constexpr auto obb_separation(
        const std::array<DataT, 3> &ax,
        const std::array<DataT, 3> &ay,
        const std::array<DataT, 3> &az,
        const std::array<DataT, 3> &ar,
        const std::array<DataT, 3> &bx,
        const std::array<DataT, 3> &by,
        const std::array<DataT, 3> &bz,
        const std::array<DataT, 3> &br,
        const DataT &tx,
        const DataT &ty,
        const DataT &tz) noexcept -> DataT
    {
        // Parallel edges collapse an edge-edge axis to zero length, taking every term of
        // its test down into float noise where the sign is meaningless — a genuine overlap
        // could read as separated. A vanished axis is not a separating axis, so it is
        // dropped from the maximum rather than propped up by a tolerance: dropping a
        // candidate can only report overlap where there is none, never the reverse, and
        // the face axes still separate parallel boxes exactly. The threshold sits an order
        // of magnitude above the float32 noise floor of 1 - (a_i . b_j)^2; what it discards
        // is edges within ~1 mrad of parallel.
        constexpr float kMinAxisLengthSq = 1.0e-6F;
        constexpr float kAxisDropped = -1.0e30F;

        std::array<std::array<DataT, 3>, 3> r;
        std::array<std::array<DataT, 3>, 3> abs_r;
        for (auto i = 0U; i < 3; ++i)
        {
            for (auto j = 0U; j < 3; ++j)
            {
                r[i][j] = dot_3(ax[i], ay[i], az[i], bx[j], by[j], bz[j]);
                abs_r[i][j] = r[i][j].abs();
            }
        }

        // The center offset resolved in each box's own frame.
        std::array<DataT, 3> ta;
        std::array<DataT, 3> tb;
        for (auto i = 0U; i < 3; ++i)
        {
            ta[i] = dot_3(tx, ty, tz, ax[i], ay[i], az[i]);
            tb[i] = dot_3(tx, ty, tz, bx[i], by[i], bz[i]);
        }

        // Face normals of A.
        auto widest =
            ta[0].abs() - (ar[0] + br[0] * abs_r[0][0] + br[1] * abs_r[0][1] + br[2] * abs_r[0][2]);
        for (auto i = 1U; i < 3; ++i)
        {
            const auto sep =
                ta[i].abs() -
                (ar[i] + br[0] * abs_r[i][0] + br[1] * abs_r[i][1] + br[2] * abs_r[i][2]);
            widest = widest.max(sep);
        }

        // Face normals of B.
        for (auto j = 0U; j < 3; ++j)
        {
            const auto sep =
                tb[j].abs() -
                (br[j] + ar[0] * abs_r[0][j] + ar[1] * abs_r[1][j] + ar[2] * abs_r[2][j]);
            widest = widest.max(sep);
        }

        // Edge-edge axes a_i x b_j.
        for (auto i = 0U; i < 3; ++i)
        {
            const auto i1 = (i + 1) % 3;
            const auto i2 = (i + 2) % 3;
            for (auto j = 0U; j < 3; ++j)
            {
                const auto j1 = (j + 1) % 3;
                const auto j2 = (j + 2) % 3;

                const auto ra = ar[i1] * abs_r[i2][j] + ar[i2] * abs_r[i1][j];
                const auto rb = br[j1] * abs_r[i][j2] + br[j2] * abs_r[i][j1];
                const auto dist = (ta[i2] * r[i1][j] - ta[i1] * r[i2][j]).abs();

                const auto sep = dist - (ra + rb);
                const auto length_sq = DataT::fill(1.0F) - (r[i][j] * r[i][j]);
                widest =
                    widest.max(sep.blend(DataT::fill(kAxisDropped), length_sq <= kMinAxisLengthSq));
            }
        }

        return widest;
    }

    template <typename DataT>
    inline constexpr auto cuboid_cuboid(const Cuboid<DataT> &a, const Cuboid<DataT> &b) noexcept
        -> DataT
    {
        return obb_separation<DataT>(
            {a.axis_1_x, a.axis_2_x, a.axis_3_x},
            {a.axis_1_y, a.axis_2_y, a.axis_3_y},
            {a.axis_1_z, a.axis_2_z, a.axis_3_z},
            {a.axis_1_r, a.axis_2_r, a.axis_3_r},
            {b.axis_1_x, b.axis_2_x, b.axis_3_x},
            {b.axis_1_y, b.axis_2_y, b.axis_3_y},
            {b.axis_1_z, b.axis_2_z, b.axis_3_z},
            {b.axis_1_r, b.axis_2_r, b.axis_3_r},
            b.x - a.x,
            b.y - a.y,
            b.z - a.z);
    }

    // z-aligned cuboids only populate the xy components of axes 1 and 2 plus the three
    // half-extents; axis 3 is the world z and the remaining fields carry nothing, so they
    // are materialised here rather than read.
    template <typename DataT>
    inline constexpr auto cuboid_z_aligned_cuboid(const Cuboid<DataT> &a, const Cuboid<DataT> &b) noexcept
        -> DataT
    {
        const auto zero = DataT::fill(0.0F);
        const auto one = DataT::fill(1.0F);

        return obb_separation<DataT>(
            {a.axis_1_x, a.axis_2_x, a.axis_3_x},
            {a.axis_1_y, a.axis_2_y, a.axis_3_y},
            {a.axis_1_z, a.axis_2_z, a.axis_3_z},
            {a.axis_1_r, a.axis_2_r, a.axis_3_r},
            {b.axis_1_x, b.axis_2_x, zero},
            {b.axis_1_y, b.axis_2_y, zero},
            {zero, zero, one},
            {b.axis_1_r, b.axis_2_r, b.axis_3_r},
            b.x - a.x,
            b.y - a.y,
            b.z - a.z);
    }
}  // namespace vamp::collision
