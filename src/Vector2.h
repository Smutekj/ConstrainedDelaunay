#pragma once

#include <cmath>
#include <cassert>

namespace cdt
{

    struct Vector2i
    {
        int x;
        int y;

        Vector2i() = default;
        Vector2i(int x, int y) : x(x), y(y) {}

        template <class VecType>
        Vector2i(const VecType &v) : x(v.x), y(v.y){};
        // Vector2i(const struct Vector2f& v) : x(v.x), y(v.y) {}

        bool operator==(const Vector2i &v) const
        {
            return v.x == x && v.y == y;
        }

        Vector2i operator-(const Vector2i &v) const
        {
            return {x - v.x, y - v.y};
        }

        Vector2i operator+(const Vector2i &v) const
        {
            return {x + v.x, y + v.y};
        }
        Vector2i operator*(int i) const
        {
            return {x * i, y * i};
        }

        Vector2i operator/(int i)
        {
            return {x / i, y / i};
        }
        void operator*=(int i)
        {
            x *= i;
            y *= i;
        }
    };

    Vector2i inline operator*(int i, const Vector2i &v)
    {
        return v * i;
    }

    struct Vector2is
    {
        unsigned short x;
        unsigned short y;

        Vector2is() = default;
        Vector2is(unsigned short x, unsigned short y) : x(x), y(y) {}

        template <class VecType>
        Vector2is(const VecType &v) : x(v.x), y(v.y){};
        // Vector2i(const struct Vector2f& v) : x(v.x), y(v.y) {}

        bool operator==(const Vector2is &v) const
        {
            return v.x == x && v.y == y;
        }

        Vector2i operator-(const Vector2is &v) const
        {
            return {x - v.x, y - v.y};
        }

        Vector2i operator+(const Vector2is &v) const
        {
            return {x + v.x, y + v.y};
        }
        Vector2i operator*(unsigned short i) const
        {
            return {x * i, y * i};
        }

        Vector2i operator/(unsigned short i)
        {
            return {x / i, y / i};
        }
        void operator*=(unsigned short i)
        {
            x *= i;
            y *= i;
        }
    };

    Vector2is inline operator*(unsigned short i, const Vector2is &v)
    {
        return v * i;
    }

    struct Vector2f
    {
        float x;
        float y;

        Vector2f() = default;
        Vector2f(float x, float y) : x(x), y(y) {}
        Vector2f(const Vector2f &r) : x(r.x), y(r.y) {}

        Vector2f(const struct Vector2i &v) : x(v.x), y(v.y) {}

        Vector2f operator+(const Vector2f &v) const
        {
            return {x + v.x, y + v.y};
        }

        Vector2f operator/(float i) const
        {
            return {x / i, y / i};
        }
        Vector2f operator*(float i) const
        {
            return {x * i, y * i};
        }
        void operator+=(const cdt::Vector2f& v)
        {
            x += v.x;
            y += v.y;
        }

        void operator/=(float i)
        {
            x /= i;
            y /= i;
        }
        void operator*=(float i)
        {
            x *= i;
            y *= i;
        }

        template <class Scalar>
        Vector2f operator*(Scalar i) const
        {
            return {x * i, y * i};
        }

        Vector2f operator-(const Vector2f &v) const
        {
            return {x - v.x, y - v.y};
        }
    };

    template <class Scalar>
    Vector2f inline operator*(Scalar i, const Vector2f &v)
    {
        return v * i;
    }

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

    inline bool approx_equal(float a, float b, float epsilon = std::numeric_limits<float>::epsilon())
    {
        return std::abs(a - b) <= 1000. * std::max(std::abs(a), std::abs(b)) * epsilon;
    }
    inline bool approx_equal_zero(float a, float epsilon = std::numeric_limits<float>::epsilon())
    {
        return std::abs(a) <= 1000. * epsilon;
    }
    bool inline strictly_less(float a, float b, float epsilon = std::numeric_limits<float>::epsilon())
    {
        return (b - a) > std::max(std::abs(a), std::abs(b)) * 10000. * epsilon;
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
