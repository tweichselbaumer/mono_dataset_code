/* Copyright (c) 2016, Jakob Engel
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "EquiUndistorter.h"

#include <sstream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <locale.h>


UndistorterEqui::UndistorterEqui()
{
	remapX = nullptr;
	remapY = nullptr;
	valid = false;
}

UndistorterEqui::UndistorterEqui(const char* configFileName)
{

	setlocale(LC_NUMERIC, "C");
	remapX = nullptr;
	remapY = nullptr;
	valid = false;

	// read parameters
	std::ifstream infile(configFileName);
	if (!infile.good())
	{
		printf("Failed to read camera calibration (invalid format?)\nCalibration file: %s\n", configFileName);
		return;
	}


	std::string l1, l2, l3, l4;
	std::getline(infile, l1);
	std::getline(infile, l2);
	std::getline(infile, l3);
	std::getline(infile, l4);


	// l1 & l2
	if (std::sscanf(l1.c_str(), "%f %f %f %f %f %f %f %f", &inputCalibration[0], &inputCalibration[1], &inputCalibration[2], &inputCalibration[3], &inputCalibration[4], &inputCalibration[5], &inputCalibration[6], &inputCalibration[7]) == 8 &&
		std::sscanf(l2.c_str(), "%d %d", &in_width, &in_height) == 2)
	{
		printf("Input resolution: %d %d\n", in_width, in_height);
		printf("Input Calibration (fx fy cx cy | k1 k2 k3 k4): %f %f %f %f | %f %f %f %f\n",
			in_width*inputCalibration[0], in_height*inputCalibration[1], in_width*inputCalibration[2], in_height*inputCalibration[3], inputCalibration[4], inputCalibration[5], inputCalibration[6], inputCalibration[7]);
	}
	else
	{
		printf("Failed to read camera calibration (invalid format?)\nCalibration file: %s\n", configFileName);
		return;
	}


	// l3
	if (std::sscanf(l3.c_str(), "%f %f %f %f", &outputCalibration[0], &outputCalibration[1], &outputCalibration[2], &outputCalibration[3]) == 4)
	{
		printf("Out: %f %f %f\n",
			outputCalibration[0], outputCalibration[1], outputCalibration[2], outputCalibration[3]);
	}
	else
	{
		printf("Out: Failed to Read Output pars... not rectifying.\n");
		return;
	}

	// l4
	if (std::sscanf(l4.c_str(), "%d %d", &out_width, &out_height) == 2)
	{
		printf("Output resolution: %d %d\n", out_width, out_height);
	}
	else
	{
		printf("Out: Failed to Read Output resolution... not rectifying.\n");
		return;
	}


	valid = true;

	// =============================== build rectification map ===============================
	remapX = new float[out_width * out_height];
	remapY = new float[out_width * out_height];
	for (int y = 0; y < out_height; y++)
		for (int x = 0; x < out_width; x++)
		{
			remapX[x + y * out_width] = x;
			remapY[x + y * out_width] = y;
		}
	distortCoordinates(remapX, remapY, out_height*out_width);

	bool hasBlackPoints = false;
	for (int i = 0; i < out_width * out_height; i++)
	{
		if (remapX[i] == 0) remapX[i] = 0.01;
		if (remapY[i] == 0) remapY[i] = 0.01;
		if (remapX[i] == in_width - 1) remapX[i] = in_width - 1.01;
		if (remapY[i] == in_height - 1) remapY[i] = in_height - 1.01;


		if (!(remapX[i] > 0 && remapY[i] > 0 && remapX[i] < in_width - 1 && remapY[i] < in_height - 1))
		{
			//printf("black pixel at %d %d %f %f!\n", i, i, remapX[i], remapY[i]);
			hasBlackPoints = true;
			remapX[i] = -1;
			remapY[i] = -1;

		}
	}

	if (hasBlackPoints)
		printf("\n\nEqui Undistorter: Warning! Image has black pixels.\n\n\n");

	// =============================== set Krect ===============================
	Krect.setIdentity();
	Krect(0, 0) = outputCalibration[0] * out_width;
	Krect(1, 1) = outputCalibration[1] * out_height;
	Krect(0, 2) = outputCalibration[2] * out_width - 0.5;
	Krect(1, 2) = outputCalibration[3] * out_height - 0.5;


	Korg.setIdentity();
	Korg(0, 0) = inputCalibration[0] * in_width;
	Korg(1, 1) = inputCalibration[1] * in_height;
	Korg(0, 2) = inputCalibration[2] * in_width - 0.5;
	Korg(1, 2) = inputCalibration[3] * in_height - 0.5;
}



UndistorterEqui::~UndistorterEqui()
{
	if (remapX != 0) delete[] remapX;
	if (remapY != 0) delete[] remapY;
}


void UndistorterEqui::distortCoordinates(float* in_x, float* in_y, int n)
{
	if (!valid)
	{
		printf("ERROR: invalid UndistorterEqui!\n");
		return;
	}


	float k1 = inputCalibration[4];
	float k2 = inputCalibration[5];
	float k3 = inputCalibration[6];
	float k4 = inputCalibration[7];

	// current camera parameters
	float fx = inputCalibration[0] * in_width;
	float fy = inputCalibration[1] * in_height;
	float cx = inputCalibration[2] * in_width - 0.5;
	float cy = inputCalibration[3] * in_height - 0.5;

	float ofx = outputCalibration[0] * out_width;
	float ofy = outputCalibration[1] * out_height;
	float ocx = outputCalibration[2] * out_width - 0.5f;
	float ocy = outputCalibration[3] * out_height - 0.5f;

	for (int i = 0; i < n; i++)
	{
		float x = in_x[i];
		float y = in_y[i];
		float ix = (x - ocx) / ofx;
		float iy = (y - ocy) / ofy;

		float r = sqrtf(ix * ix + iy * iy);

		float theta = atan(r);
		float theta2 = theta * theta;
		float theta4 = theta2 * theta2;
		float theta6 = theta4 * theta2;
		float theta8 = theta4 * theta4;
		float thetad = theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
		float scaling = (r > 1e-15) ? thetad / r : 1.0;

		float ox = fx * ix*scaling + cx;
		float oy = fy * iy*scaling + cy;

		in_x[i] = ox;
		in_y[i] = oy;
	}
}


template<typename T>
void UndistorterEqui::undistort(const T* input, float* output, int nPixIn, int nPixOut) const
{
	if (!valid) return;

	if (nPixIn != in_width * in_height)
	{
		printf("ERROR: undistort called with wrong input image dismesions (expected %d pixel, got %d pixel)\n",
			in_width*in_height, nPixIn);
		return;
	}
	if (nPixOut != out_width * out_height)
	{
		printf("ERROR: undistort called with wrong output image dismesions (expected %d pixel, got %d pixel)\n",
			out_width*out_height, nPixOut);
		return;
	}


	for (int idx = 0; idx < out_width*out_height; idx++)
	{
		// get interp. values
		float xx = remapX[idx];
		float yy = remapY[idx];

		if (xx < 0)
			output[idx] = 0;
		else
		{
			// get integer and rational parts
			int xxi = xx;
			int yyi = yy;
			xx -= xxi;
			yy -= yyi;
			float xxyy = xx * yy;

			// get array base pointer
			const T* src = input + xxi + yyi * in_width;

			// interpolate (bilinear)
			output[idx] = xxyy * src[1 + in_width]
				+ (yy - xxyy) * src[in_width]
				+ (xx - xxyy) * src[1]
				+ (1 - xx - yy + xxyy) * src[0];
		}
	}
}
template void UndistorterEqui::undistort<float>(const float* input, float* output, int nPixIn, int nPixOut) const;
template void UndistorterEqui::undistort<unsigned char>(const unsigned char* input, float* output, int nPixIn, int nPixOut) const;
