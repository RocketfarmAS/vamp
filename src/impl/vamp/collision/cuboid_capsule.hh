#pragma once

#include <array>

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>

namespace vamp::collision
{
    // Separating-axis test between an oriented box and a capsule, over the three box face
    // normals, the segment direction, and the three box-edge/segment cross axes. The
    // capsule is given as the offset from the box center to its segment midpoint, the half
    // segment vector, and the radius. Negative in a lane means collision, matching
    // sphere_capsule's sign convention.
    //
    // These seven axes do not span every separating direction for a box and a capsule — the
    // directions from a box corner to the segment are missing — so a box passing near the
    // rounded corner of the capsule can be reported as touching while a gap remains. That is
    // the safe direction to be wrong in: an axis that reports separation is always a genuine
    // separating axis, so no overlap is ever missed. Exact segment-to-box distance would
    // tighten this without changing the interface.
    template <typename DataT>
    inline constexpr auto obb_capsule_separation(
        const std::array<DataT, 3> &ax,
        const std::array<DataT, 3> &ay,
        const std::array<DataT, 3> &az,
        const std::array<DataT, 3> &ar,
        const DataT &tx,
        const DataT &ty,
        const DataT &tz,
        const DataT &hx,
        const DataT &hy,
        const DataT &hz,
        const DataT &radius) noexcept -> DataT
    {
        // As in obb_separation: an axis that has collapsed to zero length carries only float
        // noise and is dropped from the maximum instead of being propped up by a tolerance.
        // The cross-axis threshold is relative to the segment length, so it too discards
        // only edges within ~1 mrad of parallel; the segment axis is dropped for capsules
        // shorter than a micron, where the face axes already bound the (spherical) shape.
        constexpr float kMinAxisLengthSq = 1.0e-6F;
        constexpr float kMinSegmentLengthSq = 1.0e-12F;
        constexpr float kAxisDropped = -1.0e30F;

        // Face normals of the box: the segment contributes its extent along the normal, the
        // ball its radius.
        auto widest = dot_3(tx, ty, tz, ax[0], ay[0], az[0]).abs() -
                      (ar[0] + dot_3(hx, hy, hz, ax[0], ay[0], az[0]).abs() + radius);
        for (auto i = 1U; i < 3; ++i)
        {
            const auto sep = dot_3(tx, ty, tz, ax[i], ay[i], az[i]).abs() -
                             (ar[i] + dot_3(hx, hy, hz, ax[i], ay[i], az[i]).abs() + radius);
            widest = widest.max(sep);
        }

        // Segment direction, left unnormalised so every term carries a factor |h|.
        const auto h_sq = dot_3(hx, hy, hz, hx, hy, hz);
        const auto h_len = sqrt(h_sq);
        {
            auto box_extent = ar[0] * dot_3(hx, hy, hz, ax[0], ay[0], az[0]).abs();
            for (auto i = 1U; i < 3; ++i)
            {
                box_extent = box_extent + ar[i] * dot_3(hx, hy, hz, ax[i], ay[i], az[i]).abs();
            }

            const auto sep = dot_3(tx, ty, tz, hx, hy, hz).abs() -
                             (box_extent + h_sq + radius * h_len);
            widest =
                widest.max(sep.blend(DataT::fill(kAxisDropped), h_sq <= kMinSegmentLengthSq));
        }

        // Box edges crossed with the segment. The segment is parallel to every such axis, so
        // only the capsule's radius bounds it there.
        for (auto i = 0U; i < 3; ++i)
        {
            const auto lx = ay[i] * hz - az[i] * hy;
            const auto ly = az[i] * hx - ax[i] * hz;
            const auto lz = ax[i] * hy - ay[i] * hx;

            const auto l_sq = dot_3(lx, ly, lz, lx, ly, lz);

            auto box_extent = ar[0] * dot_3(lx, ly, lz, ax[0], ay[0], az[0]).abs();
            for (auto k = 1U; k < 3; ++k)
            {
                box_extent = box_extent + ar[k] * dot_3(lx, ly, lz, ax[k], ay[k], az[k]).abs();
            }

            const auto sep =
                dot_3(tx, ty, tz, lx, ly, lz).abs() - (box_extent + radius * sqrt(l_sq));
            widest = widest.max(sep.blend(
                DataT::fill(kAxisDropped), l_sq <= h_sq * DataT::fill(kMinAxisLengthSq)));
        }

        return widest;
    }

    template <typename DataT>
    inline constexpr auto cuboid_capsule(const Cuboid<DataT> &a, const Capsule<DataT> &c) noexcept
        -> DataT
    {
        const auto half = DataT::fill(0.5F);
        const auto hx = c.xv * half;
        const auto hy = c.yv * half;
        const auto hz = c.zv * half;

        return obb_capsule_separation<DataT>(
            {a.axis_1_x, a.axis_2_x, a.axis_3_x},
            {a.axis_1_y, a.axis_2_y, a.axis_3_y},
            {a.axis_1_z, a.axis_2_z, a.axis_3_z},
            {a.axis_1_r, a.axis_2_r, a.axis_3_r},
            (c.x1 + hx) - a.x,
            (c.y1 + hy) - a.y,
            (c.z1 + hz) - a.z,
            hx,
            hy,
            hz,
            c.r);
    }

    // z-aligned capsules carry their extent in zv alone; the other two components of the
    // segment hold nothing and are materialised here rather than read.
    template <typename DataT>
    inline constexpr auto cuboid_z_aligned_capsule(
        const Cuboid<DataT> &a,
        const Capsule<DataT> &c) noexcept -> DataT
    {
        const auto zero = DataT::fill(0.0F);
        const auto hz = c.zv * DataT::fill(0.5F);

        return obb_capsule_separation<DataT>(
            {a.axis_1_x, a.axis_2_x, a.axis_3_x},
            {a.axis_1_y, a.axis_2_y, a.axis_3_y},
            {a.axis_1_z, a.axis_2_z, a.axis_3_z},
            {a.axis_1_r, a.axis_2_r, a.axis_3_r},
            c.x1 - a.x,
            c.y1 - a.y,
            (c.z1 + hz) - a.z,
            zero,
            zero,
            hz,
            c.r);
    }
}  // namespace vamp::collision
