#pragma once

#include <cmath>
#include <cassert>

namespace cdt
{

    template <class T>
    struct Vector2
    {
        T x;
        T y;

        Vector2() = default;
        template <class T1, class T2>
        Vector2(T1 x, T2 y)
            : x(x), y(y) {}

        template <class T1>
        Vector2(const struct Vector2<T1> &v) : x(v.x), y(v.y) {}

        Vector2 operator+(const Vector2 &v) const
        {
            return {x + v.x, y + v.y};
        }

        Vector2 operator/(float i) const
        {
            return {x / i, y / i};
        }
        Vector2 operator*(float i) const
        {
            return {x * i, y * i};
        }

        template <class T1>
        void operator+=(const cdt::Vector2<T1> &v)
        {
            x += v.x;
            y += v.y;
        }

        template <class T1>
        void operator/=(T1 i)
        {
            x /= i;
            y /= i;
        }

        template <class T1>
        void operator*=(T1 i)
        {
            x *= i;
            y *= i;
        }

        template <class Scalar>
        Vector2 operator*(Scalar i) const
        {
            return {x * i, y * i};
        }

        template <class T1>
        Vector2 operator-(const Vector2<T1> &v) const
        {
            return {x - v.x, y - v.y};
        }

        // bool operator==(const Vector2<T>& v) const;

    };

    bool inline approx_equal(float a, float b, float epsilon = std::numeric_limits<float>::epsilon())
    {
        return std::abs(a - b) <= 1000. * std::max(std::abs(a), std::abs(b)) * epsilon;
    }
    bool inline approx_equal_zero(float a, float epsilon = std::numeric_limits<float>::epsilon())
    {
        return std::abs(a) <= 1000. * epsilon;
    }
    bool inline strictly_less(float a, float b, float epsilon = std::numeric_limits<float>::epsilon())
    {
        return (b - a) > std::max(std::abs(a), std::abs(b)) * 10000. * epsilon;
    }

    template <typename T>
    bool operator==(const Vector2<T>& v1, const Vector2<T> &v2)
    {
        return v1.x == v2.x && v1.y == v2.y;
    }

    bool operator==(const Vector2<float>& v1, const Vector2<float> &v2) = delete;

    template <class T, class Scalar>
    Vector2<T> inline operator*(Scalar i, const Vector2<T> &v)
    {
        return v * i;
    }

    using Vector2f = Vector2<float>;
    using Vector2i = Vector2<int>;

    template <typename T>
    inline float dot(const T &a, const T &b) { return a.x * b.x + a.y * b.y; }
    template <typename T>
    inline float dot(const T &&a, const T &&b) { return a.x * b.x + a.y * b.y; }

    template <typename T>
    inline float norm2(const T &a) { return dot(a, a); }
    template <typename T>
    inline float norm(const T &a) { return std::sqrt(norm2(a)); }
    template <typename T>
    inline float dist(const T &a, const T &b) { return std::sqrt(dot(a - b, a - b)); }

    template <class VecType>
    float inline cross(const VecType a, const VecType &b)
    {
        return a.x * b.y - a.y * b.x;
    }

    template <class VecType>
    float inline orient(const VecType &a, const VecType &b, const VecType &c)
    {
        return cross(b - a, c - a);
    }

    template <class VecType>
    float inline orient2(const VecType &a, const VecType &b, const VecType &c)
    {
        return -cross(b - a, c - a);
    }

    constexpr float TOLERANCE = 0.0001f;
    inline bool vequal(const cdt::Vector2f &a, const cdt::Vector2f &b) { return dist(a, b) < TOLERANCE; }

    bool inline segmentsIntersect(cdt::Vector2f a, cdt::Vector2f b, cdt::Vector2f c, cdt::Vector2f d, cdt::Vector2f &hit_point)
    {
        float oa = orient(c, d, a),
              ob = orient(c, d, b),
              oc = orient(a, b, c),
              od = orient(a, b, d);
        // Proper intersection exists iff opposite signs
        bool ab_cond = strictly_less(oa * ob, 0); // || approx_equal_zero(oa) || approx_equal_zero(ob);
        bool cd_cond = strictly_less(oc * od, 0); // || approx_equal_zero(oc) || approx_equal_zero(od);
        if (ab_cond && cd_cond)
        {
            hit_point = (a * ob - b * oa) / (ob - oa);
            assert(!std::isnan(hit_point.x) && !std::isnan(hit_point.y));
            return true;
        }
        return false;
    }

    template <class VecType>
    bool inline segmentsIntersect(const VecType &a, const VecType &b, const VecType &c, const VecType &d)
    {
        float oa = orient(c, d, a),
              ob = orient(c, d, b),
              oc = orient(a, b, c),
              od = orient(a, b, d);
        // Proper intersection exists iff opposite signs
        bool ab_cond = strictly_less(oa * ob, 0); // || approx_equal_zero(oa) || approx_equal_zero(ob);
        bool cd_cond = strictly_less(oc * od, 0); // || approx_equal_zero(oc) || approx_equal_zero(od);
        return ab_cond && cd_cond;
    }

    template <class VecType>
    bool inline segmentsIntersectOrTouch(const VecType &a, const VecType &b, const VecType &c, const VecType &d)
    {
        float oa = orient(c, d, a),
              ob = orient(c, d, b),
              oc = orient(a, b, c),
              od = orient(a, b, d);
        // Proper intersection exists iff opposite signs
        bool ab_cond = strictly_less(oa * ob, 0) || approx_equal_zero(oa) || approx_equal_zero(ob);
        bool cd_cond = strictly_less(oc * od, 0) || approx_equal_zero(oc) || approx_equal_zero(od);
        return ab_cond && cd_cond;
    }

    template <class VecType>
    bool inline segmentsIntersectOrTouch(const VecType &a, const VecType &b, const VecType &c, const VecType &d, cdt::Vector2f &hit_point)
    {
        float oa = orient(c, d, a),
              ob = orient(c, d, b),
              oc = orient(a, b, c),
              od = orient(a, b, d);
        // Proper intersection exists iff opposite signs
        bool ab_cond = strictly_less(oa * ob, 0) || approx_equal_zero(oa) || approx_equal_zero(ob);
        bool cd_cond = strictly_less(oc * od, 0) || approx_equal_zero(oc) || approx_equal_zero(od);
        if (ab_cond && cd_cond)
        {
            hit_point = (a * ob - b * oa) / (ob - oa);
            assert(!std::isnan(hit_point.x) && !std::isnan(hit_point.y));
            return true;
        }
        return false;
    }

}

inline cdt::Vector2f asFloat(const cdt::Vector2i &r) { return static_cast<cdt::Vector2f>(r); }

#include <type_traits>

namespace std
{
    template <>
    struct common_type<cdt::Vector2i, cdt::Vector2f>
    {
        using type = cdt::Vector2f;
    };
}