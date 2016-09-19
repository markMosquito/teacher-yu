#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <mutex>
#include <thread>
#include <string>

#include <climits>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

void abRead(cv::Mat &im, const std::string &fname) {
	std::ifstream inF;
	inF.open(fname.c_str(), std::ios::binary);
	if (!inF)
	{
		std::cerr << "failed to open the file : " << fname << std::endl;
		abort();
	}
	inF.read((char*)(im.ptr(0)), im.rows*im.cols*im.elemSize());
	inF.close();
}

int main()
{
	std::string fname = "H:/";
	cv::Mat im = cv::imread(fname + "test.jpg");
	cv::namedWindow("disp1");
	cv::imshow("disp1", im);

	cv::Mat x0 = cv::imread(fname + "imL.png");
	cv::Mat x1 = cv::imread(fname + "imR.png");
	cv::cvtColor(x0, x0, CV_BGR2GRAY);
	cv::cvtColor(x1, x1, CV_BGR2GRAY);
	cv::Mat1f x0_ = x0;
	cv::Mat1f x1_ = x1;
	cv::Scalar mean_, std_;
	cv::meanStdDev(x0_, mean_, std_);
	x0_ = (x0_ - mean_) / std_(0);
	cv::meanStdDev(x1_, mean_, std_);
	x1_ = (x1_ - mean_) / std_(0);
	//cv::imshow("1", x0_);
	//cv::imshow("2", x1_);
	//cv::waitKey(0);


	int disparityTotal_ = 256;
	double disparityFactor_ = 256;

	int height_ = 370;
	int width_ = 1226;
	float pi1 = 2.3;
	float pi2 = 18.38;
	float sgm_q1 = 3;
	float sgm_q2 = 2;
	float tau_so = 0.08;
	float alpha = 1.25;
	float pim1 = pi1 / (sgm_q1 * sgm_q2);
	float pin1 = pi1 / sgm_q1;
	float pim2 = pi2 / (sgm_q1 * sgm_q2);
	float pin2 = pi2 / sgm_q1;

	cv::Mat costVol(height_, width_, CV_32FC(disparityTotal_));
	abRead(costVol, fname + "test_.bin");
	//std::vector<cv::Mat> cV_channels;
	//cv::split(costVol, cV_channels);
	
	float* costImage = (float*)_mm_malloc(height_ * width_ * disparityTotal_ * sizeof(float), 16);			
	//动态内存分配的对齐参考_aligned_malloc、_mm_malloc，如果用malloc分配的内存大小大于8，则内存是8字节对齐的，否则为最小2的多次方对齐，比如如果分配7字节，则4字节对齐.void *mm_malloc(MM *mm, size_t size)
	memset(costImage, 0.0, height_ * width_ * disparityTotal_ * sizeof(float));
	int total = height_ * width_ * disparityTotal_;
	for (int i = 0; i < total; ++i)//total的值 116126720
	{
		int x = (i / disparityTotal_) % width_;//列数
		int y = (i / disparityTotal_) / width_;//行数
		int d = i % disparityTotal_;//对256求余
		costImage[i] = costVol.ptr<float>(y)[x * disparityTotal_ + d];//第i个匹配代价的值，y行，x列，d视差。最大到 116126720。数组costimage[]存放所有原始的匹配代价。
	}
	
	int* disparityImage = reinterpret_cast<int*>(malloc(width_*height_ * sizeof(int)));//reinterpret_cast强制转换。//数组disparityImage[]存放最终视差值。
	//cv::Mat S = cv::Mat::zeros(height, width, CV_32FC(disp_max));
	//std::vector<cv::Mat> S_channels;
	//cv::split(S, S_channels);
	//cv::Mat cur_tmp, pre_tmp;

	int pathRowBufferTotal_ = 2;
	int disparitySize_ = disparityTotal_ + 16;//256+16=272
	int pathTotal_ = 4;		// path: dx = -1, dy = 0; dx = 1, dy = 0; dx = 0, dy = -1; dx = 0, dy = 1
	int pathDisparitySize_ = pathTotal_*disparitySize_;//4*256=1088

	int costSumBufferRowSize_ = width_*disparityTotal_;//1226*256=313856
	int costSumBufferSize_ = costSumBufferRowSize_*height_;		// S        //313856*370= 116126720
	int pathMinCostBufferSize_ = (width_ + 2) * pathTotal_;		// min_k Lr(p-r, k)         //(1226+2)*4=4912
	int pathCostBufferSize_ = pathMinCostBufferSize_*disparitySize_;	// Lr(p-r, k)        //4912*272=1336064
	int totalBufferSize_ = (pathMinCostBufferSize_ + pathCostBufferSize_)*pathRowBufferTotal_ + costSumBufferSize_ + 16;  //  118808688

	_MM_ALIGN16 float* sgmBuffer_ = (_MM_ALIGN16 float*)(_mm_malloc(totalBufferSize_ * sizeof(float), 16));

	/*************************start************************/

	const float costMax = FLT_MAX;
	int widthStepCostImage = width_*disparityTotal_;	// step     1226*256=313856

	float* costSums = sgmBuffer_;
	memset(costSums, 0, costSumBufferSize_ * sizeof(float));//这儿中断了。。。。。

	float** pathCosts = new float*[pathRowBufferTotal_];//new float*[2]:   //int **m=new int*[10];这个m也是个指针，它也是指向一个数组长度为10的首地址。	只不过这个数组存放的元素类型是int*类型。所以要用int **m指向首地址
	                                                                                             
	                                                                     // 一维: int *a = new int[100];开辟一个大小为100的整型数组空间, 返回一个指向该存储空间的地址
	                                                                     //delete [] a; //释放int数组空间
	float** pathMinCosts = new float*[pathRowBufferTotal_];

	//float smoothnessPenaltySmall_ = 800;
	//float smoothnessPenaltyLarge_ = 1600;

	const int processPassTotal = 2;
	for (int processPassCount = 0; processPassCount < processPassTotal; ++processPassCount)
	{
		int startX, endX, stepX;
		int startY, endY, stepY;
		if (processPassCount == 0)
		{
			startX = 0; endX = width_; stepX = 1;
			startY = 0; endY = height_; stepY = 1;//Stepx=stepy=1;
		}
		else 
		{
			startX = width_ - 1; endX = -1; stepX = -1;
			startY = height_ - 1; endY = -1; stepY = -1;//////Stepx=stepy=-1;
		}

		// init buffer
		// for ideal alignment we have negative indices(d = -1), shift 4 because of 4 * sizeof(float) == 16 
		for (int i = 0; i < pathRowBufferTotal_; ++i) 
		{
			pathCosts[i] = costSums + costSumBufferSize_ + pathCostBufferSize_*i + pathDisparitySize_ + 4;
			memset(pathCosts[i] - pathDisparitySize_ - 4, 0, pathCostBufferSize_ * sizeof(float));
			pathMinCosts[i] = costSums + costSumBufferSize_ + pathCostBufferSize_*pathRowBufferTotal_
				+ pathMinCostBufferSize_*i + pathTotal_;
			memset(pathMinCosts[i] - pathTotal_, 0, pathMinCostBufferSize_ * sizeof(float));
		}

		// swap(pathCosts[0], pathCosts[1]) ===> Update preCost and curCost
		for (int y = startY; y != endY; y += stepY) 
		{
			float* pixelCostRow = costImage + widthStepCostImage*y;//数组costImage[]存的是所有原始匹配代价
			float* costSumRow = costSums + costSumBufferRowSize_*y;

			memset(pathCosts[0] - pathDisparitySize_ - 4, 0, pathDisparitySize_ * sizeof(float));
			memset(pathCosts[0] + width_ * pathDisparitySize_ - 4, 0, pathDisparitySize_ * sizeof(float));
			memset(pathMinCosts[0] - pathTotal_, 0, pathTotal_ * sizeof(float));
			memset(pathMinCosts[0] + width_*pathTotal_, 0, pathTotal_ * sizeof(float));

			for (int x = startX; x != endX; x += stepX)
			{
				int pathMinX = x * pathTotal_;
				int pathX = pathMinX * disparitySize_;

				float prePathMin0 = pathMinCosts[0][pathMinX - stepX * pathTotal_];//pathMinCosts[0]是指向数组的指针。**。
				float prePathMin1 = pathMinCosts[1][pathMinX + 2];

				float* prePathCost0 = pathCosts[0] + pathX - stepX * pathDisparitySize_;//X
				float* prePathCost1 = pathCosts[1] + pathX + 2 * disparitySize_;//Y

				prePathCost0[-1] = prePathCost0[disparityTotal_] = costMax;//X方向
				prePathCost1[-1] = prePathCost1[disparityTotal_] = costMax;//Y方向

				float* curPathCost = pathCosts[0] + pathX;
				float* costSumCurrent = costSumRow + disparityTotal_ * x;
				float* pixelCostCurrent = pixelCostRow + disparityTotal_ * x;
				
				__m128 regPathMin0, regPathMin1;//_m128用于SSE，表示封装了;4个32bit的单精度浮点数；
				regPathMin0 = _mm_set1_ps(prePathMin0);//设置4个值为同一值。
				regPathMin1 = _mm_set1_ps(prePathMin1);

				__m128 regPenaltySmallX_, regPenaltyLargeX_, regPenaltySmallY_, regPenaltyLargeY_;
				//regPenaltySmallX_ = _mm_set1_ps(smoothnessPenaltySmall_);
				//regPenaltyLargeX_ = _mm_set1_ps(smoothnessPenaltyLarge_);
				//regPenaltySmallY_ = _mm_set1_ps(smoothnessPenaltySmall_);
				//regPenaltyLargeY_ = _mm_set1_ps(smoothnessPenaltyLarge_);

				__m128 regTotalMin = _mm_set1_ps(costMax);

				float DX1 = (x - stepX > 0) && (x - stepX < width_) ? 
					x0_.ptr<float>(y)[x] - x0_.ptr<float>(y)[x - stepX] : 10;
				float DY1 = (y - stepY > 0) && (y - stepY < height_) ? 
					x0_.ptr<float>(y)[x] - x0_.ptr<float>(y - stepY)[x] : 10;

				_MM_ALIGN16 float tmpPenaltyXP1_[4];
				_MM_ALIGN16 float tmpPenaltyXP2_[4];
				_MM_ALIGN16 float tmpPenaltyYP1_[4];
				_MM_ALIGN16 float tmpPenaltyYP2_[4];

				for (int d = 0; d < disparityTotal_; d += 4)
				{
					__m128 regPixelCost = _mm_load_ps(pixelCostCurrent + d);//regPixelCost里面装的是y行x列像素 在4个d视差处的匹配代价值。

					__m128 regPathCost0, regPathCost1;
					regPathCost0 = _mm_load_ps(prePathCost0 + d);//X方向
					regPathCost1 = _mm_load_ps(prePathCost1 + d);//Y方向
					
					for (int j = 0; j < 4; ++j) 
					{
						float DX2 = std::min(x - (d + j), x - (d + j) - stepX) > 0 && 
							std::max(x - (d + j), x - (d + j) - stepX) < width_ ?
							x1_.ptr<float>(y)[x - (d + j)] - x1_.ptr<float>(y)[x - (d + j) - stepX] : 10;
						float DY2 = std::min(x - (d + j), y - stepY) > 0 && (y - stepY < height_) ?
							x1_.ptr<float>(y)[x - (d + j)] - x1_.ptr<float>(y - stepY)[x - (d + j)] : 10;
						
						tmpPenaltyXP1_[j] = DX1 < tau_so && DX2 < tau_so ? pi1 :
							DX1 > tau_so && DX2 > tau_so ? pim1 : pin1;
						tmpPenaltyXP2_[j] = DX1 < tau_so && DX2 < tau_so ? pi2 :
							DX1 > tau_so && DX2 > tau_so ? pim2 : pin2;

						tmpPenaltyYP1_[j] = DY1 < tau_so && DY2 < tau_so ? pi1 :
							DY1 > tau_so && DY2 > tau_so ? pim1 : pin1;
						tmpPenaltyYP2_[j] = DY1 < tau_so && DY2 < tau_so ? pi2 :
							DY1 > tau_so && DY2 > tau_so ? pim2 : pin2;
					}
					regPenaltySmallX_ = _mm_load_ps(tmpPenaltyXP1_);
					regPenaltyLargeX_ = _mm_load_ps(tmpPenaltyXP2_);
					regPenaltySmallY_ = _mm_load_ps(tmpPenaltyYP1_);
					regPenaltyLargeY_ = _mm_load_ps(tmpPenaltyYP2_);

					regPathCost0 = _mm_min_ps(regPathCost0, _mm_add_ps(_mm_loadu_ps(prePathCost0 + d - 1),
						regPenaltySmallX_));//前一个像素点在视差为 d处和d-1处的最小代价
					regPathCost0 = _mm_min_ps(regPathCost0, _mm_add_ps(_mm_loadu_ps(prePathCost0 + d + 1),
						regPenaltySmallX_));//d,d-1,d+1
					regPathCost0 = _mm_min_ps(regPathCost0, _mm_add_ps(regPathMin0, regPenaltyLargeX_));//	四者最小值

					regPathCost1 = _mm_min_ps(regPathCost1, _mm_add_ps(_mm_loadu_ps(prePathCost1 + d - 1),
						regPenaltySmallY_));//Y方向。
					regPathCost1 = _mm_min_ps(regPathCost1, _mm_add_ps(_mm_loadu_ps(prePathCost1 + d + 1),
						regPenaltySmallY_));
					regPathCost1 = _mm_min_ps(regPathCost1, _mm_add_ps(regPathMin1, regPenaltyLargeY_));

					regPathCost0 = _mm_sub_ps(_mm_add_ps(regPathCost0, regPixelCost), regPathMin0);//sub:减法  X  regPixelCost里面装的是y行x列像素 在4个d视差处的匹配代价值。regPathCost0;迭代下去。。。。
					regPathCost1 = _mm_sub_ps(_mm_add_ps(regPathCost1, regPixelCost), regPathMin1);//Y 能量

					_mm_store_ps(curPathCost + d, regPathCost0);
					_mm_store_ps(curPathCost + d + 2 * disparitySize_, regPathCost1);

					__m128 regMin = _mm_min_ps(_mm_unpacklo_ps(regPathCost0, regPathCost1), 
						_mm_unpackhi_ps(regPathCost0, regPathCost1));
					regMin = _mm_min_ps(_mm_unpacklo_ps(regMin, regMin), 
						_mm_unpackhi_ps(regMin, regMin));
					regTotalMin = _mm_min_ps(regMin, regTotalMin);

					__m128 regCostSum = _mm_load_ps(costSumCurrent + d);
					regCostSum = _mm_add_ps(regCostSum, regPathCost0);
					regCostSum = _mm_add_ps(regCostSum, regPathCost1);
					_mm_store_ps(costSumCurrent + d, regCostSum);
				}
				/*
				_MM_ALIGN16 float tmpMin[4];
				_mm_store_ps(tmpMin, regTotalMin);
				regTotalMin = _mm_load_ss(&tmpMin[0]);
				_mm_store_ss(&pathMinCosts[0][pathMinX], regTotalMin);
				regTotalMin = _mm_load_ss(&tmpMin[2]);
				_mm_store_ss(&pathMinCosts[0][pathMinX + 2], regTotalMin);
				*/
				_mm_store_ps(&pathMinCosts[0][pathMinX], regTotalMin);
			}

			if (processPassCount == processPassTotal - 1) 
			{
				int* disparityRow = disparityImage + width_*y;//--------------------disparityImage-------------------------
				for (int x = 0; x < width_; ++x) 
				{
					float* costSumCurrent = costSumRow + disparityTotal_*x;
					float bestSumCost = costSumCurrent[0];
					int bestDisparity = 0;
					for (int d = 1; d < disparityTotal_; ++d)
					{
						if (costSumCurrent[d] < bestSumCost) {
							bestSumCost = costSumCurrent[d];
							bestDisparity = d;
						}
					}
					if (bestDisparity > 0 && bestDisparity < disparityTotal_ - 1)
					{
						float centerCostValue = costSumCurrent[bestDisparity];
						float leftCostValue = costSumCurrent[bestDisparity - 1];
						float rightCostValue = costSumCurrent[bestDisparity + 1];
						if (rightCostValue < leftCostValue) 
						{
							bestDisparity = static_cast<int>(bestDisparity * disparityFactor_
								+ static_cast<double>(rightCostValue - leftCostValue) / (centerCostValue - leftCostValue) / 2.0 * disparityFactor_ + 0.5);
						}
						else 
						{
							bestDisparity = static_cast<int>(bestDisparity * disparityFactor_
								+ static_cast<double>(rightCostValue - leftCostValue) / (centerCostValue - rightCostValue) / 2.0 * disparityFactor_ + 0.5);
						}
					}
					else
					{
						bestDisparity = static_cast<int>(bestDisparity * disparityFactor_);
					}

					disparityRow[x] = static_cast<int>(bestDisparity);
				}
			}

			std::swap(pathCosts[0], pathCosts[1]);
			std::swap(pathMinCosts[0], pathMinCosts[1]);
		}
	}

	delete[] pathCosts;
	delete[] pathMinCosts;

	cv::Mat _disp_(height_, width_, CV_8UC1);

	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) 
		{
			_disp_.at<uchar>(y, x) = static_cast<uchar>(disparityImage[width_ * y + x] / disparityFactor_ * 4);
		}
	}

	cv::imshow("111", _disp_);
	cv::imwrite("disp.png",_disp_);
	std::cout << "done" << std::endl;
	cv::waitKey(0);
	return 0;
}

