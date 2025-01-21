#include "quaternion.h"

int main() {
    auto debut = std::chrono::high_resolution_clock::now();
    auto precision{std::numeric_limits<double>::digits};
    std::cout.precision(precision);

    Quaternion<> a(9.0, 3.0, 4.0, 2.0);
    Quaternion<> b(1.0, -8.0, -5.0, -6.0);
    Quaternion<> c(7, 2, -9, 1);
    Quaternion<> d;

    std::cout << "a = " << a << std::endl;
    std::cout << "-a = " << (-a) << std::endl;
    std::cout << "b = " << b << std::endl;
    std::cout << "a + b = " << a + b << std::endl;
    std::cout << "a - b = " << a - b << std::endl;
    std::cout << "a * b = " << a * b << std::endl;
    std::cout << "a * 10 = " << a * 10 << std::endl;
    std::cout << "a / b = " << a / b << std::endl;
    std::cout << "b / a = " << b / a << std::endl;
    // std::cout << "ERRREUR a/0 = " << a/0 << std::endl;
    std::cout << "b / 100 = " << b / 100 << std::endl;
    std::cout << "-a + (-b) + b * (a * b) = " << -a + (-b) + b * (a * b) << std::endl;
    std::cout << "a * b - b * a = " << a * b - b * a << std::endl;
    std::cout << "a * b * -c = " << a * b * (-c) << std::endl;
    std::cout << "a == b = " << (a == b) << std::endl;
    std::cout << "a != b = " << (a != b) << std::endl;
    std::cout << "conj(a) = " << a.conjugate() << std::endl;
    std::cout << "conj(conj(a)) = " << (a.conjugate()).conjugate() << std::endl;
    std::cout << "inv(a) = " << a.inverse() << std::endl;
    std::cout << "inv(inv(a)) = " << (a.inverse()).inverse() << std::endl;
    std::cout << "a * inv(a) = " << a * a.inverse() << std::endl;
    std::cout << "a * inv(b) = " << a * b.inverse() << std::endl;
    std::cout << "b * inv(a) = " << b * a.inverse() << std::endl;
    std::cout << "||a|| = " << a.norm() << std::endl;
    std::cout << "|a| = " << a.normalize() << std::endl;
    Quaternion<> q1(7, 2, -9, 1);
    std::cout << "q1 = " << q1 << std::endl;
    Quaternion<> q2(-3, 8, -4, -5);
    std::cout << "q2 = " << q2 << std::endl;
    std::cout << "q1 | q2 = " << q1.dot(q2) << std::endl;
    std::cout << "a | b = " << a.dot(b) << std::endl;
    std::cout << "a x b = " << a.cross(b) << std::endl;
    std::cout << "a^b = " << a.angle(b) << std::endl;
    auto cplx_a = a.get_complex();
    std::cout << "a(_a,_i) = " << cplx_a.first << ", a(_j,_k) = " << cplx_a.second << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    Quaternion<> u(1.0, 0.0, 1.0, 0.0);
    Quaternion<> v(-1.0, 0.0, 1.0, 0.0);
    std::cout << "u = " << u << std::endl;
    std::cout << "v = " << v << std::endl;
    u.normalize();
    v.normalize();
    std::cout << "un = " << u << std::endl;
    std::cout << "vn = " << v << std::endl;
    std::cout << "lint(u,v) = " << u.interpolation_lineaire(v, 0.5) << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    c.set_a(0.37);
    c.set_i(-0.41);
    c.set_j(0.26);
    c.set_k(-0.85);
    std::cout << "c = " << c << std::endl;

    std::cout << "-------------------------------------" << std::endl;
    double fv3[3]{5.3, -12.8, 6.2};
    Quaternion<> e(-10, fv3);
    std::cout << "e = " << e << std::endl;

    std::cout << "-------------------------------------" << std::endl;
    double fv4[4]{-0.2, -0.8, -0.1, 0.7};
    Quaternion<> f(fv4);
    std::cout << "f = " << f << std::endl;

    std::cout << "-------------------------------------" << std::endl;
    Quaternion<> g, h;
    g.depuis_axe_angle(1, 0, 0, std::numbers::pi/4);
    std::cout << "g.depuis_axe_angle(1, 0, 0, pi/4) = " << g << std::endl;
    h.depuis_axe_angle(0, 1, 0, -std::numbers::pi/3);
    std::cout << "h.depuis_axe_angle(0, 1, 0, -pi/3) = " << h << std::endl;
    
    std::cout << "-------------------------------------" << std::endl;
    Quaternion<> x;
    std::cout << "x.quat_depuis_xyz(30.0, 60.0, 45.0) = ";
    Quaternion<> y = x.quat_depuis_xyz(30.0, 60.0, 45.0);
    std::cout << y << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    Quaternion<> z(x);
    std::cout << "z.quat_vers_xyz(" << y << ") = ";
    std::array<double, 3> tmp = y.quat_vers_xyz();
    for (const auto& data : tmp) {
        std::cout << conversion::rad_vers_deg(data) << " ";
    }
    std::cout << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "y = " << y << std::endl;
    std::array<std::array<double, 3>, 3> matrice1;
    matrice1 = y.quat_vers_matrice();
    std::cout << "y.quat_vers_matrice() = " << std::endl;
    for (const auto& j: matrice1) {
        for (const auto& i: j) {
            std::cout << i << ", ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------------------------" << std::endl;
    // [ 0.7803301, -0.0561862, -0.2803301, 0.5561862 ]
    // (15, -30, 75)
    std::array<std::array<double, 3>, 3> matrice2{{
        {0.2241439, -0.8365163, -0.5000000},
        {0.8995190,  0.3750000, -0.2241439},
        {0.3750000, -0.3995191,  0.8365163}
    }};
    Quaternion<> w;
    w.matrice_vers_quat(matrice2);
    std::cout << "w.matrice_vers_quat() = " << w << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::unique_ptr<Quaternion<> > k1 = std::make_unique<Quaternion<> >(4.0, 2.0, -7.0, 8.0);
    std::cout << *k1 << std::endl;

    auto k2 = std::make_unique<Quaternion<> >(-1.0, -3.0, -8.0, -2.0);
    std::cout << *k2 << std::endl;
    k2->inverse();
    std::cout << *k2 << std::endl;

    
    std::cout << "-------------------------------------" << std::endl;

    auto fin = std::chrono::high_resolution_clock::now();
    auto temps = std::chrono::duration_cast<std::chrono::milliseconds>((fin - debut));
    std::cout << "Temps de calcul en millisecondes : " << temps.count() << " ms" << std::endl;

    std::exit(EXIT_SUCCESS);
}
