// g++-14 -march=native -flto=auto -O3 -mstore-max=512 -ftree-vectorize -std=gnu++23 -pedantic -Wall -lm -fdiagnostics-all-candidates main.cpp -o main
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <ostream>
#include <string>
#include <string_view>
#include <format>
#include <sstream>
#include <memory>
#include <cassert>
#include <cstdlib>
#include <array>
#include <complex>
#include <numbers>
#include <numeric>
#include <algorithm>
#include <exception>
#include <cfenv>
#include <functional>
#include <limits>
#include <climits>
#include <cfloat>
#include <utility>
#include <chrono>
#include <variant>
#include <type_traits>

namespace conversion {
    template <typename T> T deg_vers_rad(const T& degree) {
        return degree * (std::numbers::pi) / T(180);
    }

    template <typename T> T rad_vers_deg(const T& radian) {
        return radian * T(180) / (std::numbers::pi);
    }
}

// struct Quaterreur : public std::exception {
//     const std::string laquelle() noexecpt {
//         return
//     }
// };

template <typename T = double> class Quaternion final {
protected:
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>, "FP32 ou FP64 uniquement !");
    using type = T;
    using ptr_type = ptrdiff_t;
    typedef T& ref;
    using vec3 = std::array<T, 3>;
    using mat3 = std::array<std::array<T, 3>, 3>;
    mutable bool zero = true;

private:
    T _a, _i, _j, _k;

public:
    // constexpr Quaternion() = default;
    Quaternion() noexcept : _a(1), _i(0), _j(0), _k(0) {
        [[maybe_unused]] T temp; //std::void_t
        std::cout << "Type : " << typeid(const T).name() << std::endl;
        std::cout << "Size max : " << std::numeric_limits<T>::max() << std::endl;

        // try {

        // }
        // catch (const std::bad_typeid& e) {
        // std::cout << " caught " << e.what() << '\n';
        // }
    }
    Quaternion(const Quaternion<T>& q) noexcept : _a(static_cast<T>(q._a)), _i(static_cast<T>(q._i)), _j(static_cast<T>(q._j)), _k(static_cast<T>(q._k)) {}
    constexpr Quaternion(const T& s) noexcept : _a(s), _i(s), _j(s), _k(s) {}
    constexpr Quaternion(const T& a, const T& x, const T& y, const T& z) noexcept : _a(a), _i(x), _j(y), _k(z) {}
    constexpr Quaternion(const T(&v)[4]) noexcept : _a(v[0]), _i(v[1]), _j(v[2]), _k(v[3]) {}
    constexpr Quaternion(const T& s, const T(&v)[3]) noexcept : _a(s), _i(v[0]), _j(v[1]), _k(v[2]) {}
    constexpr Quaternion(const std::array<T, 4>& v) noexcept : _a(std::get<0>(v)), _i(std::get<1>(v)), _j(std::get<2>(v)), _k(std::get<3>(v)) {}
    constexpr Quaternion(const T& s, const vec3& v) noexcept : _a(s), _i(std::get<0>(v)), _j(std::get<1>(v)), _k(std::get<2>(v)) {}
    // constexpr Quaternion(const mat3& v) noexcept : _a(s), _i(std::get<0>(v)), _j(std::get<1>(v)), _k(std::get<2>(v)) {}

    Quaternion(const Quaternion<T>&& q) noexcept
        : _a(q._a), _i(q._i), _j(q._j), _k(q._k) {
        *this = std::move(q);
    }

    // Quaternion(const Quaternion<T>&& q) = default;

    ~Quaternion() noexcept = default;

    inline void clear() noexcept {
        // std::iota(this.begin(), this.end(), 0);
        _a = 0; _i = 0; _j = 0; _k = 0;
    }

    [[nodiscard]] constexpr decltype(auto) get_a() const noexcept { return this->_a; }
    [[nodiscard]] constexpr decltype(auto) get_i() const noexcept { return this->_i; }
    [[nodiscard]] constexpr decltype(auto) get_j() const noexcept { return this->_j; }
    [[nodiscard]] constexpr decltype(auto) get_k() const noexcept { return this->_k; }

    [[nodiscard]] constexpr decltype(auto) get_real() const noexcept { return _a; }
    [[nodiscard]] constexpr decltype(auto) get_imag() const noexcept { return std::array<T, 3>{ _i, _j, _k }; }

    [[nodiscard]] constexpr decltype(auto) get_complex() const noexcept {
        std::complex<T> c1(_a, _i);
        std::complex<T> c2(_j, _k);

        return std::make_pair(c1, c2);
    }

    void set_a(const T& v) {
        if (v <= std::numeric_limits<T>::max())
            _a = v;
    }

    void set_i(const T& v) {
        if (v <= std::numeric_limits<T>::max())
            _i = v;
    }

    void set_j(const T& v) {
        if (v <= std::numeric_limits<T>::max())
            _j = v;
    }

    void set_k(const T& v) {
        if (v <= std::numeric_limits<T>::max())
            _k = v;
    }

    // T Quaternion<T>::operator [] (int n) const {
    //     return reinterpret_pointer_cast<const T>(this)[n];
    // }

    // T& Quaternion<T>::operator [] (int n) {
    //     return reinterpret_pointer_cast<T>(this)[n];
    // }

    Quaternion<T> operator - () const {
        return { -_a, -_i, -_j, -_k };
    }

    constexpr Quaternion<T>& operator = (Quaternion<T>&& q) noexcept {
        if (this == &q)
            return *this;

        if (this != &q) {
            _a = std::move(q._a);
            _i = std::move(q._i);
            _j = std::move(q._j);
            _k = std::move(q._k);
        }
        return *this;
    }

    constexpr Quaternion<T>& operator = (const Quaternion<T>& q) noexcept {
        if (this == &q)
            return *this;

        if (this != &q) {
            _a = q._a;
            _i = q._i;
            _j = q._j;
            _k = q._k;
        }
        return *this;
    }

    constexpr Quaternion<T>& operator += (const Quaternion& q) noexcept {
        _a += q._a;
        _i += q._i;
        _j += q._j;
        _k += q._k;

        return *this;
    }

    constexpr Quaternion<T>& operator -= (const Quaternion& q) noexcept {
        _a -= q._a;
        _i -= q._i;
        _j -= q._j;
        _k -= q._k;

        return *this;
    }

    constexpr Quaternion<T>& operator *= (const T& s) noexcept {
        _a *= s;
        _i *= s;
        _j *= s;
        _k *= s;

        return *this;
    }

    constexpr Quaternion<T>& operator *= (const Quaternion& q) noexcept {
        _a = _a * q._a - _i * q._i - _j * q._j - _k * q._k;
        _i = _a * q._i + _i * q._a + _j * q._k + _k * q._j;
        _j = _a * q._j + _j * q._a + _k * q._i + _i * q._k;
        _k = _a * q._k + _k * q._a + _i * q._j + _j * q._i;

        return *this;
    }

    constexpr Quaternion<T>& operator /= (const T& s) noexcept {
        _a /= s;
        _i /= s;
        _j /= s;
        _k /= s;

        return *this;
    }

    [[nodiscard]] constexpr Quaternion<T> operator + (const Quaternion<T>& q) const noexcept {
        return Quaternion<T>(_a + q._a, _i + q._i, _j + q._j, _k + q._k);
    }

    [[nodiscard]] constexpr Quaternion<T> operator - (const Quaternion<T>& q) const noexcept {
        return Quaternion<T>(_a - q._a, _i - q._i, _j - q._j, _k - q._k);
    }

    [[nodiscard]] constexpr Quaternion<T> operator * (const Quaternion<T>& q) const noexcept {
        return Quaternion<T>(
            _a * q._a - _i * q._i - _j * q._j - _k * q._k,
            _a * q._i + _i * q._a - _j * q._k + _k * q._j,
            _a * q._j + _i * q._k + _j * q._a - _k * q._i,
            _a * q._k - _i * q._j + _j * q._i + _k * q._a
            );
    }

    [[nodiscard]] constexpr Quaternion<T> operator * (const T& s) const noexcept {

        return Quaternion<T>(this->_a * s, this->_i * s, this->_j * s, this->_k * s);
    }

    [[nodiscard]] constexpr Quaternion<T> operator / (const Quaternion<T>& q) const noexcept {
        T r, s, t, u;

        T denom{ q._a * q._a + q._i * q._i + q._j * q._j + q._k * q._k };

        r = (q._a * _a + q._i * _i + q._j * _j + q._k * _k) / denom;
        s = (q._a * _i - q._i * _a - q._j * _k + q._k * _j) / denom;
        t = (q._a * _j + q._i * _k - q._j * _a - q._k * _i) / denom;
        u = (q._a * _k - q._i * _j + q._j * _i - q._k * _a) / denom;

        return Quaternion<T>(r, s, t, u);
    }

    [[nodiscard]] constexpr Quaternion<T> operator / (const T& s) const {
        if (s == 0.0) {
            throw std::runtime_error("Division par zéro !");
            return EXIT_FAILURE;
        }
        else
            return Quaternion<T>(this->_a / s, this->_i / s, this->_j / s, this->_k / s);
        // try {
        //     if (s == 0.0)
        //         throw std::runtime_error("Division par zéro !");
        // } catch (const std::exception& e) {
        //     std::cerr << e.what() << std::endl;
        // }
    }

    [[nodiscard]] constexpr bool operator <=> (const Quaternion<T>& q) {
        return std::tie(_a, _i, _j, _k) == std::tie(q._a, q._i, q._j, q._k);
    }

    [[nodiscard]] constexpr bool operator == (const Quaternion<T>& q) {
        return std::tie(_a, _i, _j, _k) == std::tie(q._a, q._i, q._j, q._k);
    }

    [[nodiscard]] constexpr bool operator != (const Quaternion<T>& q) {
        return !(*this == q);
    }

    bool is_nan() const noexcept {
        return std::isnan(get_a()) || std::isnan(get_i()) || std::isnan(get_j()) || std::isnan(get_k());
    }

    bool is_zero() const noexcept {
        if (std::fpclassify(get_a()) == FP_ZERO
            && std::fpclassify(get_i()) == FP_ZERO
            && std::fpclassify(get_j()) == FP_ZERO
            && std::fpclassify(get_k()) == FP_ZERO) [[likely]]
            zero = true;
        else [[unlikely]]
            zero = false;

        return zero;
    }

    bool is_normal() const {
        if (std::isnormal(get_a()) && std::isnormal(get_i()) && std::isnormal(get_j()) && std::isnormal(get_k())) [[likely]]
            return true;
        else [[unlikely]]
            return false;
    }

    bool is_finite() const {
        if (std::isfinite(get_a()) && std::isfinite(get_i()) && std::isfinite(get_j()) && std::isfinite(get_k())) [[likely]]
            return true;
        else [[unlikely]]
            return false;
    }

    constexpr Quaternion<T> conjugate() const noexcept {
        return Quaternion<T>(_a, -_i, -_j, -_k);
    }

    constexpr Quaternion<T> square() const noexcept {
        T s = _a * _a - _i * _i - _j * _j - _k * _k;
        return Quaternion<T>(s, 2 * _a * _i, 2 * _a * _j, 2 * _a * _k);
    }

    constexpr T norm() const noexcept {
        if (is_zero() == true)
            std::cerr << "Quaternion equal zero." << std::endl;

        return std::sqrt(_a * _a + _i * _i + _j * _j + _k * _k);
    }

    constexpr Quaternion<T> normalize() const {
        T n = this->norm();
        if (n == 0.0)
            return Quaternion<T>(1.0, 0.0, 0.0, 0.0);

        T invnorm = 1 / n;
        [[assume(invnorm > 0)]];

        // std::feclearexcept(FE_ALL_EXCEPT);
        // if (std::fetestexcept(FE_DIVBYZERO))
        //     std::cout << "Norme égale à 0 !\n";

        return *this * invnorm;
    }

    constexpr T dot(const Quaternion<T>& q) const noexcept {
        static_assert(typeid(q) != typeid(this), "dot() : q != Quaternion");

        T result = _a * q._a + _i * q._i + _j * q._j + _k * q._k;

        return result;
    }

    constexpr Quaternion<T> inverse() const {
        T square = this->dot(*this);
        T invsquare = 0.0;

        // if (std::abs(x - rhs.x) < std::numeric_limits<T>::epsilon())

        if (square == 0) [[likely]]
            std::cerr << "inverse() : sqr == 0 !" << std::endl;
        else [[unlikely]]
            invsquare = 1 / square;

            return this->conjugate() * invsquare;
    }

    constexpr Quaternion<T> cross(const Quaternion<T>& q) const noexcept {
        // if (_a != 0 && q._a != 0)
        //     std::cerr << "It's not a pure quaternion." << std::endl;

        return Quaternion<T>(
            0,
            _j * q._k - q._j * _k,
            _k * q._i - q._k * _i,
            _i * q._j - q._i * _j
        );
    }

    constexpr T angle(const Quaternion<T>& q) const {
        T theta = std::fabs(this->dot(q));

        return std::acos(theta / (this->norm() * q.norm()));
    }

    constexpr Quaternion<T> interpolation_lineaire(const Quaternion<T>& q, const T& t) const {
        Quaternion<T> lerp;
        T dp;

        // if (typeid(const T).name() == "f")
        // if (typeid(const T).name() == "d")
        // if (typeid(const T).name() == "l")
        // static_assert(t < DBL_MIN);
        if (t < DBL_MIN) [[likely]]
            throw std::runtime_error("Step equal to 0!");
        else [[unlikely]]
            dp = this->dot(q);
            if (dp >= 0.0)
                lerp = *this * (1 - t) + q * t;
            else
                lerp = *this * (1 - t) - q * t;

            // lerp = *this * (q - *this) * t;

        return Quaternion(lerp);
    }

    constexpr Quaternion<T> interpolation_spherique(const Quaternion<T>& q1, const Quaternion<T>& q2, const T& t) const {
        if (q1 == q2)
            return q1;

        if (std::islessequal(t, 0) == true)
            return q1;
        else
            q1.normalize();

        if (std::isgreaterequal(t, 1) == true)
            return q2;
        else
            q2.normalize();

        T dp = q1.dot(q2);
        if (std::isless(dp, FLT_EPSILON) == true) {
            dp = -dp;
            q1 = -q1;
        }

        if (std::isgreaterequal(dp, static_cast<T>(1 - std::numeric_limits<T>::epsilon)) == true)
            return linearInterpolation(q1, q2, t).normalize();

        // T omega = std::sqrt(1 - dp * dp);
        // T invomega = 1 / omega;

        std::clamp(dp, -1, 1);
        T theta = std::acos(dp) / (q1.norm() * q2.norm());
        T sintheta = std::sin(theta);
        [[assume(sintheta > 0)]];
        T invsintheta = 1 / sintheta;

        return (q1 * std::sin((1 - t) * theta)) * invsintheta  + (q2 * std::sin(t * theta)) * invsintheta;
        // return ((q1 * std::sin((1 - t) * theta) * invomega + (q2 * std::sin(t * theta)) / std::sin(theta)) * invomega).normalize();
    }

    constexpr T getRotAngle() const {
        return 2 * std::acos(_a);
    }

    [[noreturn]]void from_x(const T& theta) {
        T angle = theta * 0.5;
        T s = std::sin(angle);

        _a = std::cos(angle);
        _i = theta * s;
        _j = theta * s;
        _k = theta * s;
    }

    [[noreturn]]void from_y(const T& theta) {
        T angle = theta * 0.5;
        _a = std::cos(angle);
        _i = 0;
        _j = std::sin(angle);
        _k = 0;
    }

    [[noreturn]]void from_z(const T& theta) {
        T angle = theta * 0.5;
        _a = std::cos(angle);
        _i = 0.0;
        _j = 0.0;
        _k = std::sin(angle);
    }

    void depuis_axe_angle(const T& x, const T& y, const T& z, const T& theta) {
        if (std::fpclassify(theta) == FP_ZERO) {
            _a = 1.0;
            _i = 0.0;
            _j = 0.0;
            _k = 0.0;
        } else {
            // T angle = conversion::deg_vers_rad(theta * 0.5);
            T angle = theta * 0.5;
            T sintheta = std::sin(angle);

            if (theta <= 180)
                _a = std::cos(angle);
            else
                _a = 0.0;

            _i = x * sintheta;
            _j = y * sintheta;
            _k = z * sintheta;

            // this->normalize();
        }
    }

    constexpr Quaternion<T> quat_depuis_xyz(const T& x, const T& y, const T& z) const {
        T dx = conversion::deg_vers_rad(x * 0.5);
        T dy = conversion::deg_vers_rad(y * 0.5);
        T dz = conversion::deg_vers_rad(z * 0.5);

        T cx = std::cos(dx);
        T sx = std::sin(dx);
        T cy = std::cos(dy);
        T sy = std::sin(dy);
        T cz = std::cos(dz);
        T sz = std::sin(dz);

        return Quaternion<T>(
            cx * cy * cz - sx * sy * sz,
            sx * cy * cz + cx * sy * sz,
            cx * sy * cz - sx * cy * sz,
            cx * cy * sz + sx * sy * cz
        ).normalize();
    }

    constexpr vec3 quat_vers_xyz() const {
        T rx, ry, rz;

        // X
        T roll0 = 2.0 * (_a * _i - _j * _k);
        T roll1 = 1.0 - 2.0 * (_i * _i + _j * _j);
        if (std::fpclassify(roll0) == FP_ZERO && std::fpclassify(roll1) == FP_ZERO)
            rx = 0.0;
        else
            rx = std::atan2(roll0, roll1);
        
        // Y
        T yaw = std::clamp(2.0 * (_a * _j + _i * _k), T(-1.0), T(1.0));
        ry = std::asin(yaw);

        // Z
        T pitch0 = 2.0 * (_a * _k - _i * _j);
        T pitch1 = 1.0 - 2.0 * (_j * _j + _k * _k);
        if (std::fpclassify(pitch0) == FP_ZERO && std::fpclassify(pitch1) == FP_ZERO)
            rz = 0.0;
        else
            rz = std::atan2(pitch0, pitch1);

        return vec3{ rx, ry, rz };
    }

    constexpr auto quat_vers_matrice() const {
        T ii = get_i() * get_i();
        T jj = get_j() * get_j();
        T kk = get_k() * get_k();
        T ai = get_i() * get_a();
        T aj = get_j() * get_a();
        T ak = get_k() * get_a();
        T ij = get_i() * get_j();
        T ik = get_i() * get_k();
        T jk = get_j() * get_k();

        return mat3{{
            {1.0 - 2.0 * jj - 2.0 * kk, 2.0 * ij - 2.0 * ak, 2.0 * ik + 2.0 * aj},
            {2.0 * ij + 2.0 * ak, 1.0 - 2.0 * ii - 2.0 * kk, 2.0 * jk - 2.0 * ai},
            {2.0 * ik - 2.0 * aj, 2.0 * jk + 2.0 * ai, 1.0 - 2.0 * ii - 2.0 * jj}
        }};
    }

    constexpr Quaternion<T> matrice_vers_quat(const mat3& mat) const {
        T trace, a, i, j, k;

        static_assert(std::is_array<mat3>::value != true);
        static_assert(std::rank<mat3>::value != 2);

        trace = mat.at(0).at(0) + mat.at(1).at(1) + mat.at(2).at(2);
        a = std::sqrt((1 + trace) / 4.0);
        i = std::sqrt((1 + mat.at(0).at(0) - mat.at(1).at(1) - mat.at(2).at(2)) / 4);
        j = std::sqrt((1 - mat.at(0).at(0) + mat.at(1).at(1) - mat.at(2).at(2)) / 4);
        k = std::sqrt((1 - mat.at(0).at(0) - mat.at(1).at(1) + mat.at(2).at(2)) / 4);

        return Quaternion<T>(a, i, j, k).normalize();
    }

    friend std::ostream& operator << (std::ostream& os, const Quaternion<T>& q) {
        std::cout.setf(std::ios_base::dec, std::ios_base::basefield);;
        std::setiosflags(std::ios::fixed);
        auto precision{ std::numeric_limits<T>::digits };
        std::cout.precision(precision);

        os << std::setprecision(4) << std::showpoint << std::showpos << std::unitbuf << std::format("Quaternion({}, {}, {}, {})", q._a, q._i, q._j, q._k);

        return os;
    }

    // void coutMat(std::array<std::array<T, n>, n> &mat) {
    //     for (const auto& x: mat){
    //         for (const auto& val: x)
    //             std::cout << val << " ";
    //         std::cout << "\n";
    //     }
    // }

    friend std::ostream& operator << (std::ostream& os, const mat3& a) {
        std::setiosflags(std::ios::fixed);
        auto precision{ std::numeric_limits<T>::digits10 };
        std::cout.precision(precision);
        os << std::setprecision(8) << "|" << a.at(0).at(0) << ", " << a.at(0).at(1) << a.at(0).at(2) << "|" \
            << std::setprecision(8) << "|" << a.at(1).at(0) << ", " << a.at(1).at(1) << a.at(1).at(2) << "|" \
            << std::setprecision(8) << "|" << a.at(2).at(0) << ", " << a.at(2).at(1) << a.at(2).at(2);

        return os;
    }

    friend std::istream& operator >> (std::istream& is, const Quaternion<T>& q) {
        T a, i, j, k;

        is >> a >> "," >> i >> "," >> j >> "," >> k;

        if (!is.fail()) {
            q._a = a;
            q._i = i;
            q._j = j;
            q._k = k;
        }

        return is;
    }
};
