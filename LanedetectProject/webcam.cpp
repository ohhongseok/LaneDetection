//ī�޶� �׽�Ʈ�� cpp ���Ϸ�, ������ ����, ��ķ ��뿡 ���� ����
//webcamtest.cpp �� �����ϰ� �������� ������ x �� üũ�ؼ� �����ϱ�
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const int Width = 320;		//width �츮�� ����� ������ ��
const int Height = 240;		//height ����

int main(int ac, char**av)
{
	//cap(��ȣ)�� ��ȣ ����
	/*
		1. ��Ʈ��(���� ��ķ)�� ���, VideoCapture cap(0) �� ��Ʈ�� ��ķ�̴�.
		2. ��Ʈ�� ������ķ�� �ְ�, ���� usb ķ�� ������ ��� ��, VideoCapture cap(1) �� �ȴ�.
		3. ķ�� ���� ����ũž�� ��� usb ���� �� VideoCapture cap(0) �� �ȴ�.
	*/

	VideoCapture cap(1,CAP_DSHOW);			//��ķ ��ü ����, CAP_DSHOW ���� ĸ�� �� �� �ִ�.
	
	

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
		int fps = cap.get(CAP_PROP_FPS);

		if (img.empty())
		{
			printf("empty image");
			return 0;
		}
		// ���Ⱑ ���� ����� img
		resize(img, img, Size(Width, Height));

		imshow("camera img", img);
		printf("%d\n", fps);
		if (waitKey(25) == 27)
			break;
	}


	return 0;
}