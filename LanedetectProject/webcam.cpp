//ī�޶� �׽�Ʈ�� cpp ���Ϸ�, ������ ����, ��ķ ��뿡 ���� ����
//webcamtest.cpp �� �����ϰ� �������� ������ x �� üũ�ؼ� �����ϱ�
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int ac, char**av)
{
	//cap(��ȣ)�� ��ȣ ����
	/*
		1. ��Ʈ��(���� ��ķ)�� ���, VideoCapture cap(0) �� ��Ʈ�� ��ķ�̴�.
		2. ��Ʈ�� ������ķ�� �ְ�, ���� usb ķ�� ������ ��� ��, VideoCapture cap(1) �� �ȴ�.
		3. ķ�� ���� ����ũž�� ��� usb ���� �� VideoCapture cap(0) �� �ȴ�.
	*/

	VideoCapture cap(1);			//��ķ ��ü ����
	
	if (!cap.isOpened())
	{
		printf("ī�޶� ����");
		return -1; 
	}

	//���� �ݺ�
	Mat img;

	while (1)
	{
		cap >> img;

		if (img.empty())
		{
			printf("empty image");
			return 0;
		}

		imshow("camera img", img);

		if (waitKey(25) == 27)
			break;
	}


	return 0;
}