/// <summary>
/// This class provides functions to perform spherical harmonic transforms on images using CUDA.
/// </summary>
#pragma once
#ifndef SH_TRANSFORM_H
#include <opencv2/opencv.hpp>
#include "cublas_v2.h"
#include "spherical_harmonics.hpp"




namespace GPU_SH
{
class SHTransform
{
public:
    SHTransform(int image_width, int image_height, int order, cublasHandle_t handle);
    ~SHTransform();

    void init_from_precalc(int order, int image_width, int image_height, SphericalHarmonics* sph);
    void get_all_sh_rotations(double* input_coeffs, double* output_coeffs, int num_lm_coeffs);
    std::vector<double> transform(cv::Mat &img);
    cv::Mat inverse_transform(std::vector<double> coefficents);
    int sh_count();
    int get_order();
    double factorial(int num);
    int get_index(int L, int m);

private:
    double* d_current_image_;
    double* d_SH_Matrix_;
    double* d_coeffs;
    int image_height_;
    int image_width_;
    int order_;
    std::unique_ptr<SphericalHarmonics> sph_;
    cublasHandle_t handle_;

    void tf(cv::Mat &img);
    
    void transfromSH( int m, int n, double* d_A, cv::Mat& current_image, double* d_Y);
    

    void inverse_transform(double* d_feature_matrix, int m, int n, double* d_coefficents, int n_coeffs, double* d_image);

};

} // GPU_SH

#endif // SH_TRANSFORM_H
