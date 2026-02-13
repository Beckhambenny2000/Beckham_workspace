#include "gv_d100/depth2color.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;
namespace gv
{
	bool bCompute_xn = false;

	double* xn_left;

	void comp_distortion_oulu_640x400(double* xn, double* x_distort, double kc[], int x_i, int y_i, int h, int w)
	{
		double x[2];
		double k[3], p[2];
		k[0] = kc[0];
		k[1] = kc[1];
		k[2] = kc[4];
		p[0] = kc[2];
		p[1] = kc[3];
		x[0] = x_distort[0];
		x[1] = x_distort[1];
		double r_2, k_radial, delta_x[2];

		for (int i = 0; i < 20; i++) {
			double x0_2 = x[0] * x[0];
			double x1_2 = x[1] * x[1];
			r_2 = x0_2 + x1_2;

			k_radial = 1 + k[0] * r_2 + k[1] * r_2 * r_2 + k[2] * pow(r_2, 3);
			delta_x[0] = 2 * p[0] * x[0] * x[1] + p[1] * (r_2 + 2 * x0_2);
			delta_x[1] = p[0] * (r_2 + 2 * x1_2) + 2 * p[1] * x[0] * x[1];

			x[0] = (x_distort[0] - delta_x[0]) / k_radial;
			x[1] = (x_distort[1] - delta_x[1]) / k_radial;
		}
		xn[y_i * w + x_i] = x[0];
		xn[y_i * w + x_i + h * w] = x[1];
	}

	void normalize_pixel_640x400(double* xn, double x_kk[], double fc[], double cc[], double n, double kc[], double alpha_c, int x_i, int y_i, int h, int w)
	{
		double* x_distort = new double[2];
		x_distort[1] = (x_kk[1]+1 - cc[1]) / fc[1];
		x_distort[0] = (x_kk[0]+1 - cc[0]) / fc[0] - alpha_c * x_distort[1];

		if (n != 0) {
			comp_distortion_oulu_640x400(xn, x_distort, kc, x_i, y_i, h, w);
		}
		else
		{
			xn[y_i * w + x_i] = x_distort[0];
			xn[y_i * w + x_i + h * w] = x_distort[1];
		}
		delete[] x_distort;
	}

	void project_points2_640x400(double* xp, double X[], double f[], double c[], double k[])
	{
		double inv_Z = 1 / X[2];
		double x[2];
		x[0] = X[0] * inv_Z;
		x[1] = X[1] * inv_Z;

		double x0_2 = x[0] * x[0];
		double x1_2 = x[1] * x[1];
		double r2 = x0_2 + x1_2;
		double r4 = r2 * r2;
		double r6 = r4 * r2;
		double cdist = 1 + k[0] * r2 + k[1] * r4 + k[4] * r6;

		double xd1[2];
		xd1[0] = x[0] * cdist;
		xd1[1] = x[1] * cdist;

		double a1 = 2 * x[0] * x[1];
		double a2 = r2 + 2 * x0_2;
		double a3 = r2 + 2 * x1_2;

		xd1[0] += (k[2] * a1 + k[3] * a2);
		xd1[1] += (k[2] * a3 + k[3] * a1);

		double xd3[2];
		//xd3[0] = xd1[0] +  xd1[2];
		xd3[0] = xd1[0];
		xd3[1] = xd1[1];

		xp[0] = xd3[0] * f[0] + c[0];
		xp[1] = xd3[1] * f[1] + c[1];
	}

	void Bilinera_640x400(double* dst, int width, int height, double* depth,double sw, double sc,int w)
	{
		int inputHeight = height;
		int inputWidth = width;
		int squaresize = 2 * w + 1;
		int spatialsize = squaresize * squaresize;
		double* spatial = new double[spatialsize];
		for (int i = -w; i <= w; i++)
		{
			for (int j = -w; j <= w; j++)
			{
				spatial[(i + w) * squaresize + j + w] = exp(-(j*j+i*i)/(2*sw*sw));
			}
		}
		
		for (int i = 0; i < inputHeight; i++)
		{
			for (int j = 0; j < inputWidth; j++)
			{
				int iMin=max(i-w,0);
				int iMax = min(i + w, inputHeight-1);
				int jMin = max(j - w, 0);
				int jMax = min(j + w, inputWidth-1);
				double result2 = 0;
				double result3 = 0;
				for (int ii = iMin; ii <= iMax; ii++)
					for (int jj = jMin; jj <= jMax; jj++)
					{
						if (depth[ii * inputWidth + jj] > 0)
						{
							double depth_weight =  spatial[(ii-i+w) * squaresize + jj-j+w];
							double depth_sum = depth[ii * inputWidth + jj] * depth_weight;
							result3 += depth_weight;
							result2 += depth_sum;
						}
					}
				if(result3==0)
					dst[i * inputWidth + j] = 0;
				else
					dst[i * inputWidth + j] = result2 / result3;
			}
		}
	}


	void medianFilter(double* dst, int width, int height, double* depth) 
	{
		int inputHeight = height-1;
		int inputWidth = width-1;

		for(int h = 1; h < inputHeight; h++)
		{
			for(int w = 1; w < inputWidth; w++)
			{
				int x = w;
				int y = h;
				int x1 = int(x);
				int y1 = int(y);
				int x2 = x1 + 1;
				int y2 = y1 + 1;

				double q11 = depth[y1 * inputWidth + x1];
				double q12 = depth[y2 * inputWidth + x1];
				double q21 = depth[y1 * inputWidth + x2];
				double q22 = depth[y2 * inputWidth + x2];
				//printf("inter : %lf, %lf, %lf, %lf\n", q11, q12, q21, q22);
				if (q11 == 0 && q12 == 0 && q21 == 0 && q22 == 0) {
					dst[h * inputWidth + w] = 0.;
				}

				double r1 = ((x2 - x) / (x2 - x1)) * q11 + ((x - x1) / (x2 - x1)) * q21;
				double r2 = ((x2 - x) / (x2 - x1)) * q12 + ((x - x1) / (x2 - x1)) * q22;
				
				//printf("r res : %lf, %lf\n", r1, r2);
				double value = ((y2 - y) / (y2 - y1)) * r1 + ((y - y1) / (y2 - y1)) * r2;
				//printf("value : %lf \n", value);
				dst[h * inputWidth + w] = ((y2 - y) / (y2 - y1)) * r1 + ((y - y1) / (y2 - y1)) * r2;
				//printf("depth : %lf, %lf \n",  depth[h * inputWidth + w], dst[h * inputWidth + w]);
				//printf("\n");
			}	
		}
	}


	void computeXN(gv::DepthAlignToColor::Parameters d2cParams)
	{
		int width = d2cParams.width;
		int height = d2cParams.height;
		double fx = d2cParams.left_fc[0];
		double fy = d2cParams.left_fc[1];
		double cx = d2cParams.left_cc[0];
		double cy = d2cParams.left_cc[1];
		double k1 = d2cParams.left_kc[0];
		double k2 = d2cParams.left_kc[1];
		double p1 = d2cParams.left_kc[2];
		double p2 = d2cParams.left_kc[3];
		double k3 = 0.;
		//double x_pixel[2];
		xn_left = new double[height * width * 2];

		for (int y_i = 0; y_i < height; y_i++) {
			for (int x_i = 0; x_i < width; x_i++) {
				double x = (x_i - cx) / fx;
				double y = (y_i - cy) / fy;
				double r = std::sqrt(x * x + y * y);
				double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
				double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
				double u_distorted = fx * x_distorted + cx;
				double v_distorted = fy * y_distorted + cy;
				double X_left = (u_distorted - cx) / fx;
				double Y_left = (v_distorted - cy) / fy;
				xn_left[y_i * width + x_i] = X_left;
				xn_left[y_i * width + x_i + height * width] = Y_left;
			}
		}
		bCompute_xn = true;
	}

	void DepthAlignToColor::depthAlignToColor(double *Depth, Parameters d2cParams, double *alignDepthFilter, double *alignPointcloud)
	{		
		if(!bCompute_xn)
		{
			computeXN(d2cParams);
		}

		int width = d2cParams.width;
		int height = d2cParams.height;
		const int img_size = width * height;
		double* x_reproj = new double[2];
		double* alignDepth = new double[img_size];
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                alignDepth[i * width + j] = 0.;
            }
        }

		double XYZ_1[3];
		for (int y_i = 0; y_i < height; y_i++) {
			for (int x_i = 0; x_i < width; x_i++) {
				double Depth_xy = Depth[y_i * width + x_i];
				if(Depth_xy > 0.)
				{
					double X_left = xn_left[y_i * width + x_i] * Depth_xy;
					double Y_left = xn_left[y_i * width + x_i + width * height] * Depth_xy;
					double Z_left = Depth_xy;
					XYZ_1[0] = d2cParams.R[0] * X_left + d2cParams.R[1] * Y_left + d2cParams.R[2] * Z_left + d2cParams.T[0];
					XYZ_1[1] = d2cParams.R[3] * X_left + d2cParams.R[4] * Y_left + d2cParams.R[5] * Z_left + d2cParams.T[1];
					XYZ_1[2] = d2cParams.R[6] * X_left + d2cParams.R[7] * Y_left + d2cParams.R[8] * Z_left + d2cParams.T[2];
					project_points2_640x400(x_reproj, XYZ_1, d2cParams.right_fc, d2cParams.right_cc, d2cParams.right_kc);
					int j = round(x_reproj[0]-1);
					int i = round(x_reproj[1]-1);
					if (i >= 1 && j >= 1 && i < 398 && j < 638) {
						for(int m = i-1; m < (i+2); m++)
						{
							for(int n = j-1; n < (j+2); n++)
							{
								if(alignDepth[m * width + n] == 0)
									alignDepth[m * width + n] = XYZ_1[2];
							}
						}
					}
				}
			}
		}

		medianFilter(alignDepthFilter, width, height, alignDepth);
		
		double rfx = d2cParams.right_fc[0];
		double rfy = d2cParams.right_fc[1];
		double rcx = d2cParams.right_cc[0];
		double rcy = d2cParams.right_cc[1];
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				double Depth_xy = alignDepthFilter[i * width + j];
				//double Depth_xy = alignDepth[i * width + j];
				if(Depth_xy > 0)
				{
					//alignDepthFilter[i * width + j] = Depth_xy;
					double X = (j - rcx) / rfx  * Depth_xy;
					double Y = (i - rcy) / rfy  * Depth_xy;
					double Z = Depth_xy;
		
					alignPointcloud[i * width * 3 + j * 3] = X;
					alignPointcloud[i * width * 3 + j * 3 + 1] = Y;
					alignPointcloud[i * width * 3 + j * 3 + 2] = Z;
				}
			}
		}
		delete(x_reproj);
		delete(alignDepth);
	}

	void DepthAlignToColor::depthAlignToHDColor(double *Depth, Parameters d2cParams, double *alignDepthFilter, double *alignPointcloud)
	{		
		if(!bCompute_xn)
		{
			computeXN(d2cParams);
		}

		int width = d2cParams.width;
		int height = d2cParams.height;
		int hdWidth = d2cParams.hdWidth;
		int hdHeight = d2cParams.hdHeight;
		const int img_size = width * height;
		double* x_reproj = new double[2];
		double* alignDepth = new double[hdWidth * hdHeight];
        for (int i = 0; i < hdHeight; i++) {
            for (int j = 0; j < hdWidth; j++) {
                alignDepth[i * hdWidth + j] = 0.;
            }
        }
	
		double XYZ_1[3];
		for (int y_i = 0; y_i < height; y_i++) {
			for (int x_i = 0; x_i < width; x_i++) {
				double Depth_xy = Depth[y_i * width + x_i];
				if(Depth_xy > 0.)
				{
					double X_left = xn_left[y_i * width + x_i] * Depth_xy;
					double Y_left = xn_left[y_i * width + x_i + width * height] * Depth_xy;
					double Z_left = Depth_xy;
					XYZ_1[0] = d2cParams.hdR[0] * X_left + d2cParams.hdR[1] * Y_left + d2cParams.hdR[2] * Z_left + d2cParams.hdT[0];
					XYZ_1[1] = d2cParams.hdR[3] * X_left + d2cParams.hdR[4] * Y_left + d2cParams.hdR[5] * Z_left + d2cParams.hdT[1];
					XYZ_1[2] = d2cParams.hdR[6] * X_left + d2cParams.hdR[7] * Y_left + d2cParams.hdR[8] * Z_left + d2cParams.hdT[2];
					project_points2_640x400(x_reproj, XYZ_1, d2cParams.hdRight_fc, d2cParams.hdRight_cc, d2cParams.hdRight_kc);
					int j = round(x_reproj[0]-1);
					int i = round(x_reproj[1]-1);
					if (i >= 2 && j >= 2 && i < 956 && j < 1276) {
					//if (i >= 2 && j >= 2 && i < 796 && j < 1276) {
						// if(alignDepth[i * hdWidth + j] == 0)
						// 			alignDepth[i * hdWidth + j] = XYZ_1[2];
						for(int m = i-2; m < (i+4); m++)
						{
							for(int n = j-2; n < (j+4); n++)
							{
								if(alignDepth[m * hdWidth + n] == 0)
									alignDepth[m * hdWidth + n] = XYZ_1[2];
							}
						}
					}
				}
			}
		}

		medianFilter(alignDepthFilter, hdWidth, hdHeight, alignDepth);
		
		double rfx = d2cParams.right_fc[0];
		double rfy = d2cParams.right_fc[1];
		double rcx = d2cParams.right_cc[0];
		double rcy = d2cParams.right_cc[1];
		for (int i = 0; i < hdHeight; i++) {
			for (int j = 0; j < hdWidth; j++) {
				//double Depth_xy = alignDepthFilter[i * width + j];
				double Depth_xy = alignDepth[i * hdWidth + j];
				if(Depth_xy > 0)
				{
					//std::cout << " { " << i << "," << j << "," << Depth_xy << "} ";

					alignDepthFilter[i * hdWidth + j] = Depth_xy;
					double X = (j - rcx) / rfx  * Depth_xy;
					double Y = (i - rcy) / rfy  * Depth_xy;
					double Z = Depth_xy;
		
					alignPointcloud[i * hdWidth * 3 + j * 3] = X;
					alignPointcloud[i * hdWidth * 3 + j * 3 + 1] = Y;
					alignPointcloud[i * hdWidth * 3 + j * 3 + 2] = Z;
				}
			}
		}
		delete(x_reproj);
		delete(alignDepth);
	}
}