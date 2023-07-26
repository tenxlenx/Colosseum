#include "cuda.h"
#include "cublas_v2.h"
#include "cuda_runtime.h"
#include "sh_transform.cuh"

#include <bitset>
#include <memory>
#include "spherical_harmonics.hpp"
#include <cufft.h>
#include <device_launch_parameters.h>
#include <filesystem>

#include <opencv2/opencv.hpp>



#ifndef HAVE_HEADER_H
#include "sh_transform_generated.cuh"
void GPU_SH::SHTransform::init_from_precalc(
                  int order = 8
                , int image_width = 128
                , int image_height = 64,
                  SphericalHarmonics* sph = nullptr)
{

    int num_coeffs = SphericalHarmonics::GetCoefficientCount(order);
    int image_size = image_height * image_width;
    int matrix_size = image_size * num_coeffs;

    const size_t rows = std::size(sh_transform_mat);
    const size_t cols = std::size(sh_transform_mat[0]);
    cudaMalloc(&d_SH_Matrix_, sizeof(double) * matrix_size);
    cudaMemcpy(d_SH_Matrix_, sh_transform_mat, sizeof(double) * matrix_size, cudaMemcpyHostToDevice);
}
#else

void GPU_SH::SHTransform::init_from_precalc(
    int order = 8, int image_width = 128, int image_height = 64,
    SphericalHarmonics* sph = nullptr)
{
    /// Check if sh_transform_mat is defined in the header file
    // If not defined, generate it here
    if (sph == nullptr) sph = new SphericalHarmonics(image_width, image_height, order);
    sph->generate_matrix_code(order, image_width, image_height);

}

#endif



__global__ void inverse_transform_kernel(double* d_feature_matrix, int m, int n, double* d_coefficents, int n_coeffs, double* d_image)
{
    // get the thread index
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;

    // check if the thread index is within the bounds of the image
    if (idx < n && idy < m) {
        // initialize the sum to 0
        double sum = 0.0;

        // calculate the sum
        for (int i = 0; i < n_coeffs; i++) {
            sum += d_feature_matrix[i * n * m + idx * m + idy] * d_coefficents[i];
        }

        // store the result in the image
        d_image[idx * m + idy] = sum;
    }
}

__device__ int GetIndex(int l, int m)
{
    return l * (l + 1) + m;
}

__device__ void getLM(int index, int& L, int& M)
{
    L = (int)floor(sqrtf(index));
    M = index - (L * L + L);
}

__device__ double rotateZ(double* coefficients, int index, double rotation_degree)
{
    int L = 0;
    int M = 0;
    double coeff_n;
    double coeff_p;
    double rot_coeff = 0;
    double alpha = rotation_degree * M_PI / 180.0;
    getLM(index, L, M);

    if (M < 0) {
        coeff_n = coefficients[index];
        coeff_p = coefficients[GetIndex(L, -M)];
    }
    else {
        coeff_n = coefficients[GetIndex(L, -M)];
        coeff_p = coefficients[index];
    }
    if (M < 0) {
        rot_coeff = coeff_n * cos(abs(M) * alpha) - coeff_p * sin(abs(M) * alpha);
    }
    else {
        rot_coeff = coeff_n * sin(abs(M) * alpha) + coeff_p * cos(abs(M) * alpha);
    }
    return rot_coeff;
}

__global__ void rotateCoeffs(double* input_coeffs, double* output_coeffs, double rotation_degree, int num_lm_coeffs)
{
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index < num_lm_coeffs) {
        output_coeffs[index] = rotateZ(input_coeffs, index, rotation_degree);
    }
}

void GPU_SH::SHTransform::get_all_sh_rotations(double* input_coeffs, double* output_coeffs, int num_rotations)
{
    int num_lm_coeffs = sph_->GetCoefficientCount(order_);
    // Launch kernel with num_rotations threads per block, and enough blocks to cover all coefficients
    int block_size = num_rotations;
    int num_blocks = (num_lm_coeffs + block_size - 1) / block_size;
    for (int i = 0; i < num_rotations; i++) {
        double rotation_degree = 360.0 * i / num_rotations;

        rotateCoeffs<<<num_blocks, block_size>>>(input_coeffs, &output_coeffs[num_lm_coeffs * i], rotation_degree, num_lm_coeffs);
        cudaDeviceSynchronize();
        // process the rotated coefficients here, e.g. save them to a file
    }
}

template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T>>& orig)
{
    std::vector<T> ret;
    for (const auto& v : orig)
        ret.insert(ret.end(), v.begin(), v.end());
    return ret;
}

std::vector<double> GPU_SH::SHTransform::transform(cv::Mat& img)
{
    //calculates coeffs
    tf(img);
    int numElements = sh_count();
    double* tempHostArray = new double[numElements];
    cudaMemcpy(tempHostArray, d_coeffs, numElements * sizeof(double), cudaMemcpyDeviceToHost);
    std::vector<double> hostVector(numElements);
    hostVector.assign(tempHostArray, tempHostArray + numElements);
    delete[] tempHostArray;
    return hostVector;
}

void GPU_SH::SHTransform::tf(cv::Mat& img)
{
    int m = sph_->GetCoefficientCount(order_);
    int n = img.rows * img.cols;

    transfromSH(m, n, d_SH_Matrix_, img, d_coeffs);
}

cv::Mat GPU_SH::SHTransform::inverse_transform(std::vector<double> coefficients)
{
    int num_coeffs = sph_->GetCoefficientCount(order_);
    int m = image_height_;
    int n = image_width_;

    cudaMemcpy(d_coeffs, coefficients.data(), num_coeffs * sizeof(double), cudaMemcpyHostToDevice);

    double* d_image;
    cudaMalloc(&d_image, n * m * sizeof(double));

    inverse_transform(d_SH_Matrix_, m, n, d_coeffs, num_coeffs, d_image);

    cv::Mat result_image(m, n, CV_64F);
    cudaMemcpy(result_image.ptr<double>(), d_image, n * m * sizeof(double), cudaMemcpyDeviceToHost);

    cudaFree(d_image);

    return result_image;
}

GPU_SH::SHTransform::~SHTransform()
{
    cudaFree(d_coeffs);
    cudaFree(d_SH_Matrix_);
    cudaFree(d_current_image_);
    cublasDestroy(handle_);
}

int GPU_SH::SHTransform::sh_count()
{
    return sph_->GetCoefficientCount(order_);
}

void GPU_SH::SHTransform::transfromSH(int m, int n, double* d_A, cv::Mat& current_image, double* d_Y)
{
    // If input image is not CV_64F, convert to CV_64F
    if (current_image.type() != CV_64F) {
        current_image.convertTo(current_image, CV_64F);
    }

    // Multiply the feature matrix (d_A) with the image (1d flattened) to get the coefficients
    cudaMemcpy(d_current_image_, current_image.ptr<double>(), current_image.rows * current_image.cols * sizeof(double), cudaMemcpyHostToDevice);

    double alpha = 1.0;
    double beta = 0.0;

    cublasDgemv(handle_, CUBLAS_OP_T, n, m, &alpha, d_A, n, d_current_image_, 1, &beta, d_Y, 1);

    cudaDeviceSynchronize();
}

int GPU_SH::SHTransform::get_index(int L, int m)
{
    return sph_->GetIndex(L, m);
}

int GPU_SH::SHTransform::get_order()
{
    return order_;
}

double GPU_SH::SHTransform::factorial(int num)
{
    return sph_->factorial(num);
}




GPU_SH::SHTransform::SHTransform(int image_width, int image_height, int order, cublasHandle_t handle = nullptr)
    : d_SH_Matrix_(nullptr)
    , image_height_(image_height)
    , image_width_(image_width)
    , order_(order)
{
    cudaMalloc((void**)&d_current_image_, image_width_ * image_height_ * sizeof(double));
    sph_ = std::make_unique<SphericalHarmonics>(image_width_, image_height_, order_);
    auto trans_mat = sph_->transform_matrix();

    // if generated header exists, we use the generated file otherwise, we generate one
    init_from_precalc(order, image_width, image_height, sph_.get());
    

    if (handle == nullptr) {
        cublasCreate(&handle_);
    }
    else {
        handle_ = handle;
    }

    // uploading matrix to gpu
    int num_coeffs = sph_->GetCoefficientCount(order_);
    auto mat_size = image_height_ * image_width_ * num_coeffs;
    cudaMalloc(&d_coeffs, num_coeffs * sizeof(double));
    cudaMalloc(&d_SH_Matrix_, mat_size * sizeof(double));
    cudaMemcpy(d_SH_Matrix_, flatten(trans_mat).data(), mat_size * sizeof(double), cudaMemcpyHostToDevice);
}

// function call
void GPU_SH::SHTransform::inverse_transform(double* d_feature_matrix, int m, int n, double* d_coefficents, int n_coeffs, double* d_image)
{
    // calculate the number of blocks and threads
    int block_size = 32;
    int num_blocks_x = (n + block_size - 1) / block_size;
    int num_blocks_y = (m + block_size - 1) / block_size;

    // launch the kernel
    inverse_transform_kernel<<<num_blocks_x, num_blocks_y>>>(d_feature_matrix, m, n, d_coefficents, n_coeffs, d_image);
}