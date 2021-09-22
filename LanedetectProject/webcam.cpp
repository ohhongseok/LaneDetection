//카메라 테스트용 cpp 파일로, 프레임 조절, 웹캠 사용에 목적 있음
//webcamtest.cpp 만 제외하고 나머지는 컴파일 x 로 체크해서 시행하기
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const int Width = 320;		//width 우리가 사용할 영상의 값
const int Height = 240;		//height 영상

int main(int ac, char**av)
{
	//cap(번호)의 번호 순서
	/*
		1. 노트북(내장 웹캠)의 경우, VideoCapture cap(0) 은 노트북 웹캠이다.
		2. 노트북 내장웹캠이 있고, 따로 usb 캠을 연결해 사용 시, VideoCapture cap(1) 이 된다.
		3. 캠이 없는 데스크탑의 경우 usb 연결 시 VideoCapture cap(0) 가 된다.
	*/

	VideoCapture cap(1,CAP_DSHOW);			//웹캠 객체 생성, CAP_DSHOW 쓰면 캡쳐 할 수 있다.
	
	

	if (!cap.isOpened())
	{
		printf("카메라 없음");
		return -1; 
	}

	//영상 반복
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
		// 여기가 받은 출력한 img
		resize(img, img, Size(Width, Height));

		imshow("camera img", img);
		printf("%d\n", fps);
		if (waitKey(25) == 27)
			break;
	}


	return 0;
}