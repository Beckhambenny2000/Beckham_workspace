#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>

using namespace std;
namespace gv
{
	class DepthAlignToColor
	{
	public:
		struct Parameters {
			double left_fc[2];
			double left_cc[2];
			double left_kc[5];
			double R[9];
			double T[3];
			double right_fc[2];
			double right_cc[2];
			double right_kc[5];
			double hdRight_fc[2];
			double hdRight_cc[2];
			double hdRight_kc[5];
			double hdR[9];
			double hdT[3];
			double algo_cx;
			double algo_cy;
			double baseline;
			double focus;
			double alpha_c = 0.;
			double Ralpha_c = 0.;
			int width = 640;
			int height = 400; 
			int hdWidth = 1280;
			int hdHeight = 960;
		};

	public:

		void depthAlignToColor(double *Depth, Parameters d2cParams, double *alignDepth, double *alignPointcloud);
		void depthAlignToHDColor(double *Depth, Parameters d2cParams, double *alignDepth, double *alignPointcloud);
	};
}

