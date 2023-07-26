#pragma once
#include <fstream>
#include <ostream>
#include <string>
#include <cmath>
#include <vector>

#define M_PI 3.14159265358979323846

// mostly based on this implementation:
// https://github.com/google/spherical-harmonics/blob/master/sh/spherical_harmonics.cc

class SphericalHarmonics
{

public:
    int image_size_x_;
    int image_size_y_;
    int order_;
    int transform_size_;
    double normalization_factor_;
    std::vector<std::vector<double>> transform_matrix_;

    SphericalHarmonics(int image_x, int image_y, int order)
        : image_size_x_(image_x), image_size_y_(image_y), order_(order), transform_size_((order_ + 1) * (order_ + 1) * image_x * image_y), normalization_factor_(std::sqrt(4 * M_PI / (image_x * image_y)))
    {
        transform_matrix_.resize(transform_size_);
        transform_matrix_ = get_feature_matrx(order_, image_size_x_, image_size_y_);
    }

    std::vector<std::vector<double>> transform_matrix()
    {

        return transform_matrix_;
    }

    // Get the total number of coefficients for a function represented by
    // all spherical harmonic basis of degree <= @order (it is a point of
    // confusion that the order of an SH refers to its degree and not the order).
    static constexpr int GetCoefficientCount(int order)
    {
        return (order + 1) * (order + 1);
    }

    template <typename T>
    T Zero();
    template <>
    double Zero()
    {
        return 0.0;
    }
    template <>
    float Zero()
    {
        return 0.0;
    }

    //! caching the factorials instead of calculating
    double factorial_lookup_table[21] = { 1.0,
                                          1.0,
                                          2.0,
                                          6.0,
                                          24.0,
                                          120.0,
                                          720.0,
                                          5040.0,
                                          40320.0,
                                          362880.0,
                                          3628800.0,
                                          39916800.0,
                                          479001600.0,
                                          6227020800.0,
                                          87178291200.0,
                                          1307674368000.0,
                                          20922789888000.0,
                                          355687428096000.0,
                                          6402373705728000.0,
                                          121645100408832000.0,
                                          2432902008176640000.0 };

    /**
         *
         * @tparam L order
         * @tparam M band
         * @param d vector[x,y,z]
         * @return SH basis function
         *
         * [l=0,m= 0]  0.5 * sqrt(1/pi)
         * [l=1,m=-1] -sqrt(3/(4pi)) * y
         * [l=1,m= 0] sqrt(3/(4pi)) * z
         * [l=1,m= 1] -sqrt(3/(4pi)) * x
         * [l=1,m=-2] 0.5 * sqrt(15/pi) * x * y
         * [l=2,m=-1] -0.5 * sqrt(15/pi) * y * z
         * [l=2,m= 0] 0.25 * sqrt(5/pi) * (-x^2-y^2+2z^2)
         * [l=2,m= 1] -0.5 * sqrt(15/pi) * x * z
         * [l=2,m= 2] 0.25 * sqrt(15/pi) * (x^2 - y^2)
         * [l=3,m=-3] -0.25 * sqrt(35/(2pi)) * y * (3x^2 - y^2)
         * [l=3,m=-2] 0.5 * sqrt(105/pi) * x * y * z
         * [l=3,m=-1] -0.25 * sqrt(21/(2pi)) * y * (4z^2-x^2-y^2)
         * [l=3,m= 0] 0.25 * sqrt(7/pi) * z * (2z^2 - 3x^2 - 3y^2)
         * [l=3,m= 1] -0.25 * sqrt(21/(2pi)) * x * (4z^2-x^2-y^2)
         * [l=3,m= 2] 0.25 * sqrt(105/pi) * z * (x^2 - y^2)
         * [l=3,m= 3] -0.25 * sqrt(35/(2pi)) * x * (x^2-3y^2)
         * [l=4,m= 4] 0.75 * sqrt(35/pi) * x * y * (x^2-y^2)
         * [l=4,m=-3] -0.75 * sqrt(35/(2pi)) * y * z * (3x^2-y^2)
         * [l=4,m=-2] 0.75 * sqrt(5/pi) * x * y * (7z^2-1)
         * [l=4,m=-1] -0.75 * sqrt(5/(2pi)) * y * z * (7z^2-3)
         * [l=4,m= 0] 3/16 * sqrt(1/pi) * (35z^4-30z^2+3)
         * [l=4,m= 1] -0.75 * sqrt(5/(2pi)) * x * z * (7z^2-3)
         * [l=4,m= 2] 3/8 * sqrt(5/pi) * (x^2 - y^2) * (7z^2 - 1)
         * [l=4,m= 3] -0.75 * sqrt(35/(2pi)) * x * z * (x^2 - 3y^2)
         * [l=4,m= 4] 3/16*sqrt(35/pi) * (x^2 * (x^2 - 3y^2) - y^2 * (3x^2 - y^2))
         */
    template <int L, int M>
    double SH_FUNC(const double* d);
    template <>
    double SH_FUNC<0, 0>(const double* d)
    {
        return 0.282095;
    }
    template <>
    double SH_FUNC<1, -1>(const double* d)
    {
        return -0.488603 * d[1];
    }
    template <>
    double SH_FUNC<1, 0>(const double* d)
    {
        return 0.488603 * d[2];
    }
    template <>
    double SH_FUNC<1, 1>(const double* d)
    {
        return -0.488603 * d[0];
    }
    template <>
    double SH_FUNC<2, -2>(const double* d)
    {
        return 1.092548 * d[0] * d[1];
    }
    template <>
    double SH_FUNC<2, -1>(const double* d)
    {
        return -1.092548 * d[1] * d[2];
    }
    template <>
    double SH_FUNC<2, 0>(const double* d)
    {
        return 0.315392 * (-d[0] * d[0] - d[1] * d[1] + 2.0 * d[2] * d[2]);
    }
    template <>
    double SH_FUNC<2, 1>(const double* d)
    {
        return -1.092548 * d[0] * d[2];
    }
    template <>
    double SH_FUNC<2, 2>(const double* d)
    {
        return 0.546274 * (d[0] * d[0] - d[1] * d[1]);
    }
    template <>
    double SH_FUNC<3, -3>(const double* d)
    {
        return -0.590044 * d[1] * (3.0 * d[0] * d[0] - d[1] * d[1]);
    }
    template <>
    double SH_FUNC<3, -2>(const double* d)
    {
        return 2.890611 * d[0] * d[1] * d[2];
    }
    template <>
    double SH_FUNC<3, -1>(const double* d)
    {
        return -0.457046 * d[1] * (4.0 * d[2] * d[2] - d[0] * d[0] - d[1] * d[1]);
    }
    template <>
    double SH_FUNC<3, 0>(const double* d)
    {
        return 0.373176 * d[2] *
               (2.0 * d[2] * d[2] - 3.0 * d[0] * d[0] - 3.0 * d[1] * d[1]);
    }
    template <>
    double SH_FUNC<3, 1>(const double* d)
    {
        return -0.457046 * d[0] * (4.0 * d[2] * d[2] - d[0] * d[0] - d[1] * d[1]);
    }
    template <>
    double SH_FUNC<3, 2>(const double* d)
    {
        return 1.445306 * d[2] * (d[0] * d[0] - d[1] * d[1]);
    }
    template <>
    double SH_FUNC<3, 3>(const double* d)
    {
        return -0.590044 * d[0] * (d[0] * d[0] - 3.0 * d[1] * d[1]);
    }
    template <>
    double SH_FUNC<4, -4>(const double* d)
    {
        return 2.503343 * d[0] * d[1] * (d[0] * d[0] - d[1] * d[1]);
    }
    template <>
    double SH_FUNC<4, -3>(const double* d)
    {
        return -1.770131 * d[1] * d[2] * (3.0 * d[0] * d[0] - d[1] * d[1]);
    }
    template <>
    double SH_FUNC<4, -2>(const double* d)
    {
        return 0.946175 * d[0] * d[1] * (7.0 * d[2] * d[2] - 1.0);
    }
    template <>
    double SH_FUNC<4, -1>(const double* d)
    {
        return -0.669047 * d[1] * d[2] * (7.0 * d[2] * d[2] - 3.0);
    }
    template <>
    double SH_FUNC<4, 0>(const double* d)
    {
        double z2 = d[2] * d[2];
        return 0.105786 * (35.0 * z2 * z2 - 30.0 * z2 + 3.0);
    }
    template <>
    double SH_FUNC<4, 1>(const double* d)
    {
        return -0.669047 * d[0] * d[2] * (7.0 * d[2] * d[2] - 3.0);
    }
    template <>
    double SH_FUNC<4, 2>(const double* d)
    {
        return 0.473087 * (d[0] * d[0] - d[1] * d[1]) * (7.0 * d[2] * d[2] - 1.0);
    }
    template <>
    double SH_FUNC<4, 3>(const double* d)
    {
        return -1.770131 * d[0] * d[2] * (d[0] * d[0] - 3.0 * d[1] * d[1]);
    }
    template <>
    double SH_FUNC<4, 4>(const double* d)
    {
        double x2 = d[0] * d[0];
        double y2 = d[1] * d[1];
        return 0.625836 * (x2 * (x2 - 3.0 * y2) - y2 * (3.0 * x2 - y2));
    }

    //! compile time factorial lookup table
    template <int I>
    constexpr long double fact_c();
    template <>
    constexpr long double fact_c<0>()
    {
        return factorial_lookup_table[0];
    }
    template <>
    constexpr long double fact_c<1>()
    {
        return factorial_lookup_table[1];
    }
    template <>
    constexpr long double fact_c<2>()
    {
        return factorial_lookup_table[2];
    }
    template <>
    constexpr long double fact_c<3>()
    {
        return factorial_lookup_table[3];
    }
    template <>
    constexpr long double fact_c<4>()
    {
        return factorial_lookup_table[4];
    }
    template <>
    constexpr long double fact_c<5>()
    {
        return factorial_lookup_table[5];
    }
    template <>
    constexpr long double fact_c<6>()
    {
        return factorial_lookup_table[6];
    }
    template <>
    constexpr long double fact_c<7>()
    {
        return factorial_lookup_table[7];
    }
    template <>
    constexpr long double fact_c<8>()
    {
        return factorial_lookup_table[8];
    }
    template <>
    constexpr long double fact_c<9>()
    {
        return factorial_lookup_table[9];
    }
    template <>
    constexpr long double fact_c<10>()
    {
        return factorial_lookup_table[10];
    }
    template <>
    constexpr long double fact_c<11>()
    {
        return factorial_lookup_table[11];
    }
    template <>
    constexpr long double fact_c<12>()
    {
        return factorial_lookup_table[12];
    }
    template <>
    constexpr long double fact_c<13>()
    {
        return factorial_lookup_table[13];
    }
    template <>
    constexpr long double fact_c<14>()
    {
        return factorial_lookup_table[14];
    }
    template <>
    constexpr long double fact_c<15>()
    {
        return factorial_lookup_table[15];
    }
    template <>
    constexpr long double fact_c<16>()
    {
        return factorial_lookup_table[16];
    }
    template <>
    constexpr long double fact_c<17>()
    {
        return factorial_lookup_table[17];
    }
    template <>
    constexpr long double fact_c<18>()
    {
        return factorial_lookup_table[18];
    }
    template <>
    constexpr long double fact_c<19>()
    {
        return factorial_lookup_table[19];
    }
    template <>
    constexpr long double fact_c<20>()
    {
        return factorial_lookup_table[20];
    }

    double factorial(int n)
    {
        switch (n) {
        case 0:
            //
            return 1;
        case 1:
            return fact_c<1>();
        case 2:
            return fact_c<2>();
        case 3:
            return fact_c<3>();
        case 4:
            return fact_c<4>();
        case 5:
            return fact_c<5>();
        case 6:
            return fact_c<6>();
        case 7:
            return fact_c<7>();
        case 8:
            return fact_c<8>();
        case 9:
            return fact_c<9>();
        case 10:
            return fact_c<10>();
        case 11:
            return fact_c<11>();
        case 12:
            return fact_c<12>();
        case 13:
            return fact_c<13>();
        case 14:
            return fact_c<14>();
        case 15:
            return fact_c<15>();
        case 16:
            return fact_c<16>();
        case 17:
            return fact_c<17>();
        case 18:
            return fact_c<18>();
        case 19:
            return fact_c<19>();
        case 20:
            return fact_c<20>();
        default:
            return -1;
        }
    }

    //! calculate the double factorial
    constexpr double fact2_c(int x)
    {
        constexpr const double dbl_factorial_cache[16] = {
            1.0, 1.0, 2.0, 3.0, 8.0, 15.0, 48.0, 105.0, 384.0, 945.0, 3840.0, 10395.0, 46080.0, 135135.0, 645120.0, 2027025.0
        };
        if (x < 16) {
            return dbl_factorial_cache[x];
        }
        else {
            double s = dbl_factorial_cache[16 - (x % 2 == 0 ? 2 : 1)];
            double n = x;
            while (n >= 16) {
                s *= n;
                n -= 2.0;
            }
            return s;
        }
    }

    //! Clamp the first argument to be greater than or equal to the second
    //! and less than or equal to the third.
    constexpr double Clamp(double val, double min, double max)
    {
        if (val < min) {
            val = min;
        }
        if (val > max) {
            val = max;
        }
        return val;
    }

    /** Evaluate the associated Legendre polynomial of degree @l and order @m at
         * coordinate @x. The inputs must satisfy:
         * 1. l >= 0
         * 2. 0 <= m <= l
         * 3. -1 <= x <= 1
         */
    constexpr double EvalLegendrePolynomial(int l, int m, double x)
    {
        // Compute Pmm(x) = (-1)^m(2m - 1)!!(1 - x^2)^(m/2)
        double pmm = 1.0;
        // P00 is defined as 1.0, do don't evaluate Pmm unless we know m > 0
        if (m > 0) {
            double sign = (m % 2 == 0 ? 1 : -1);
            pmm = sign * fact2_c(2 * m - 1) * pow(1 - x * x, m / 2.0);
        }
        if (l == m) {
            return pmm;
        }

        // Compute Pmm+1(x) = x(2m + 1)Pmm(x)
        double pmm1 = x * (2 * m + 1) * pmm;
        if (l == m + 1) {
            return pmm1;
        } // Pml == Pmm+1

        // Pml(x) = (x(2l - 1)Pml-1 - (l + m - 1)Pml-2) / (l - m)
        for (int n = m + 2; n <= l; n++) {
            double pmn = (x * (2 * n - 1) * pmm1 - (n + m - 1) * pmm) / (n - m);
            pmm = pmm1;
            pmm1 = pmn;
        }
        return pmm1; // Pmm1 = Pml
    }

    //! converting from lat lon to cartesian coordinates
    void ToVector(double phi, double theta, double* vec3d)
    {
        double r = sin(theta);
        vec3d[0] = r * cos(phi);
        vec3d[1] = r * sin(phi);
        vec3d[2] = cos(theta);
    }

    //! Return true if the first value is within epsilon of the second value.
    bool NearByMargin(const double actual, const double expected)
    {
        double diff = actual - expected;
        if (diff < 0.0) {
            diff = -diff;
        }
        // 5 bits of error in mantissa (source of '32 *')
        return diff < 32 * std::numeric_limits<double>::epsilon();
    }

    //! given xyz rotation angle, return the corresponding phi and theta
    void ToSphericalCoords(const double* rotation_dir, double* phi, double* theta)
    {
        const double x = rotation_dir[0];
        const double y = rotation_dir[1];
        const double z = rotation_dir[2];
        double normL2 = sqrt(x * x + y * y + z * z);
        bool success = NearByMargin(normL2, 1.0); // Explicitly clamp the z
        assert(success);
        *theta = acos(Clamp(z, -1.0, 1.0));
        *phi = atan2(y, x);
    }

    double FastFMod(double x, double m)
    {
        return x - (m * floor(x / m));
    }

    //! note: directions are measured from the center of the pixel, so add 0.5 to
    //! conv from int ind to float pixel coord
    double ImageXToPhi(const int x, const int width)
    {
        return 2.0 * M_PI * (x + 0.5) / width;
    }
    //! convert vertical cartesian image coordinate to spherical
    //! @height height of the image
    //! @y image coordinate
    double ImageYToTheta(const int y, const int height)
    {
        return M_PI * (y + 0.5) / height;
    }

    //! given an equi-rectangular image size, and polar coordinates get cartesian coordinates
    //! @phi polar coordinate1 (horizontal)
    //! @theta polar coordinate2 (vertical)
    //! @height height of the image
    //! @width width of the image
    //! @vec2d pointer to the coordinates
    void ToImageCoords(double phi, double theta, const int width, const int height,
                       double* vec2d)
    {
        // Allow theta to repeat and map to 0 to pi. However, to account for cases
        // where y goes beyond the normal 0 to pi range, phi may need to be adjusted.
        theta = Clamp(FastFMod(theta, 2.0 * M_PI), 0.0, 2.0 * M_PI);
        if (theta > M_PI) {
            theta = 2.0 * M_PI - theta; // now theta is between 0 and pi
            phi += M_PI;
        }
        phi = Clamp(FastFMod(phi, 2.0 * M_PI), 0.0, 2.0 * M_PI);
        // Now phi is in [0, 2pi] and theta is in [0, pi]
        vec2d[0] = width * phi / (2.0 * M_PI);
        vec2d[1] = height * theta / M_PI;
    }

    //! template< int L, int M>
    double SH_fast(const int l, const int m, int phi, int theta)
    {
        double dir[3] = { 0.0, 0.0, 0.0 };
        ToVector(phi, theta, dir);
        if (l <= 4) {
            if (l == 0) return SH_FUNC<0, 0>(dir);
            if (l == 1 && m == -1) return SH_FUNC<1, -1>(dir);
            if (l == 1 && m == 0) return SH_FUNC<1, 0>(dir);
            if (l == 1 && m == 1) return SH_FUNC<1, 1>(dir);
            if (l == 2 && m == -2) return SH_FUNC<2, -2>(dir);
            if (l == 2 && m == -1) return SH_FUNC<2, -1>(dir);
            if (l == 2 && m == 0) return SH_FUNC<2, 0>(dir);
            if (l == 2 && m == 1) return SH_FUNC<2, 1>(dir);
            if (l == 2 && m == 2) return SH_FUNC<2, 2>(dir);
            if (l == 3 && m == -3) return SH_FUNC<3, -3>(dir);
            if (l == 3 && m == -2) return SH_FUNC<3, -2>(dir);
            if (l == 3 && m == -1) return SH_FUNC<3, -1>(dir);
            if (l == 3 && m == 0) return SH_FUNC<3, 0>(dir);
            if (l == 3 && m == 1) return SH_FUNC<3, 1>(dir);
            if (l == 3 && m == 2) return SH_FUNC<3, 2>(dir);
            if (l == 3 && m == 3) return SH_FUNC<3, 3>(dir);
            if (l == 4 && m == -4) return SH_FUNC<4, -4>(dir);
            if (l == 4 && m == -3) return SH_FUNC<4, -3>(dir);
            if (l == 4 && m == -2) return SH_FUNC<4, -2>(dir);
            if (l == 4 && m == -1) return SH_FUNC<4, -1>(dir);
            if (l == 4 && m == 0) return SH_FUNC<4, 0>(dir);
            if (l == 4 && m == 1) return SH_FUNC<4, 1>(dir);
            if (l == 4 && m == 2) return SH_FUNC<4, 2>(dir);
            if (l == 4 && m == 3) return SH_FUNC<4, 3>(dir);
            if (l == 4 && m == 4) return SH_FUNC<4, 4>(dir);
        }
        else {
            double phi, theta;
            ToSphericalCoords(dir, &phi, &theta);
            return SH_ext(l, m, phi, theta);
        }
        return 0.0;
    }

    //! if band is greater than 4, we don't use the pre-computed version
    double SH_ext(int L, int M, double phi, double theta)
    {
        assert(L >= 0);
        assert(-L <= M && M <= L);
        double fact1 = factorial_lookup_table[L - abs(M)];
        double fact2 = factorial_lookup_table[L + abs(M)];
        double kml = sqrt((2.0 * L + 1) * fact1 / (4.0 * M_PI * fact2));
        if (M > 0) {
            return sqrt(2.0) * kml * cos(M * phi) * EvalLegendrePolynomial(L, M, cos(theta));
        }
        else if (M < 0) {
            return sqrt(2.0) * kml * sin(-M * phi) * EvalLegendrePolynomial(L, -M, cos(theta));
        }
        else {
            return kml * EvalLegendrePolynomial(L, 0, cos(theta));
        }
    }

    // Get the one dimensional index associated with a particular degree @l
    // and order @m. This is the index that can be used to access the Coeffs
    // returned by SHSolver.
    constexpr int GetIndex(int l, int m)
    {
        return l * (l + 1) + m;
    }

    void getLM(int index, int& L, int& M)
    {
        L = (int)floor(sqrt(index));
        M = index - (L * L + L);
    }

    double rotateZ(std::vector<double> coefficents, int index, double rotation_degree)
    {
        int L = 0;
        int M = 0;
        double coeff_n;
        double coeff_p;
        double rot_coeff = 0;
        double alpha = rotation_degree * M_PI / 180.0;
        getLM(index, L, M);

        if (M < 0) {
            coeff_n = coefficents[index];
            coeff_p = coefficents[GetIndex(L, -M)];
        }
        else {
            coeff_n = coefficents[GetIndex(L, -M)];
            coeff_p = coefficents[index];
        }
        if (M < 0) {
            rot_coeff = coeff_n * cos(abs(M) * alpha) - coeff_p * sin(abs(M) * alpha);
        }
        else {
            rot_coeff = coeff_n * sin(abs(M) * alpha) + coeff_p * cos(abs(M) * alpha);
        }
        return rot_coeff;
    }
    std::vector<double> getRotatedCoefficents(const std::vector<double> coefficents, double z_rot_degree)
    {
        std::vector<double> rotated_coeffs;
        for (int i = 0; i < coefficents.size(); i++) {
            double rot_coeff;
            rot_coeff = rotateZ(coefficents, i, z_rot_degree);
            rotated_coeffs.push_back(rot_coeff);
        }
        return rotated_coeffs;
    }

   

    //! precomputes the matrix of feature pre-fixes, just multiply with transpose of image-vector
    std::vector<double> preComputeCoeffs(int l, int m, int width, int height)
    {
        std::vector<double> prefix_values;
        double pixel_area = (2.0 * M_PI / width) * (M_PI / height);
        for (int t = 0; t < height; t++) {
            const double theta = ImageYToTheta(t, height);
            double weight = pixel_area * sin(theta);
            for (int p = 0; p < width; p++) {
                const double phi = ImageXToPhi(p, width);
                // normalisation factor
                double norm = sqrt((2.0 * l + 1.0) / (4.0 * M_PI) * factorial(l - m) / factorial(l + m));
                double sh = SH_ext(l, m, phi, theta) * weight * norm;
                prefix_values.push_back(sh);
            }
        }
        return prefix_values;
    }

    //! projects the image to spherical harmonics
    std::unique_ptr<std::vector<double>> ProjectEnvironment(int order, const cv::Mat& env)
    {
        std::unique_ptr<std::vector<double>> sh_coeffs(new std::vector<double>());
        sh_coeffs->assign(GetCoefficientCount(order), 0.0);
        double pixel_area = (2.0 * M_PI / env.size().width) * (M_PI / env.size().height);
        for (int t = 0; t < env.size().height; t++) {
            const double theta = ImageYToTheta(t, env.size().height);
            double weight = pixel_area * sin(theta);
            for (int p = 0; p < env.size().width; p++) {
                const double phi = ImageXToPhi(p, env.size().width);
                double point = (double)env.at<float>(t, p);
                for (int l = 0; l <= order; l++) {
                    for (int m = -l; m <= l; m++) {
                        int i = GetIndex(l, m);
                        double sh = SH_ext(l, m, phi, theta);
                        double norm = sqrt((2.0 * l + 1.0) / (4.0 * M_PI) * factorial(l - m) / factorial(l + m));
                        sh_coeffs->at(i) += sh * weight * point * norm;
                    }
                }
            }
        }
        return sh_coeffs;
    }

    void rollLeft(const cv::Mat& in, cv::Mat& out, size_t pixelsLeft)
    {
        assert(in.type() == CV_8UC1);

        // Make sure out is big enough
        out.create(in.size(), CV_8UC1);

        // Loop through rows
        for (int y = 0; y < in.rows; y++) {
            // Get pointer to start of row
            const uint8_t* rowPtrIn = in.ptr(y);
            uint8_t* rowPtrOut = out.ptr(y);

            // Rotate row to left by pixels
            std::rotate_copy(rowPtrIn, rowPtrIn + pixelsLeft, rowPtrIn + in.cols, rowPtrOut);
        }
    }

    std::vector<std::vector<double>> get_feature_matrx(int ord, int width, int height)
    {
        auto feature_n = SphericalHarmonics::GetCoefficientCount(ord);
        std::vector<std::vector<double>> feature_matrix;
        for (int i = 0; i < feature_n; i++) {
            int L;
            int M;
            SphericalHarmonics::getLM(i, L, M);
            auto prefix = SphericalHarmonics::preComputeCoeffs(L, M, width, height);
            feature_matrix.push_back(prefix);
        }

        return feature_matrix;
    }

    void generate_matrix_code(int ord, int width, int height)
    {
        // open a file in write mode.
        auto features_sh = get_feature_matrx(ord, width, height);
        std::ofstream outfile;
        std::stringstream file_name;
        file_name << "sh_transform_generated" << ".cuh";
        outfile.open(file_name.str());
        auto coeff_n = SphericalHarmonics::GetCoefficientCount(ord);
        outfile << "/// auto generated file to store feature matrix to avoid precomputing, ";
        outfile << "the file can be regenerated with the given order (L) and image size (W) (H) parameters\n";
        outfile << "#ifndef SH_TRANSFORM_GENERATED_H\n#define SH_TRANSFORM_GENERATED_H\n";
        outfile << "const double sh_transform_mat";
        outfile << "[" << coeff_n << "][" << width * height << "] =\n{\n"; // matrix start
        std::stringstream matrix_string;
        for (int i = 0; i < coeff_n; i++) {
            // matrix row start
            matrix_string << "\t{";
            for (int j = 0; j < width * height; j++) {
                matrix_string << features_sh[i][j];
                // separate with "," until the last element
                if (j < width * height - 1) {
                    matrix_string << ", ";
                }
            }
            // row end
            matrix_string << "}";
            if (i < coeff_n - 1) {
                matrix_string << ",\n";
            }
        }
        // matrix end
        matrix_string << "\n};\n";
        outfile << matrix_string.str();
        outfile.close();

        // write input data into the file.
    }
};