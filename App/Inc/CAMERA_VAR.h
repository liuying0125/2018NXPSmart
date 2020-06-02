#ifndef CAMERA_VAR
#define CAMERA_VAR

#define CAMERA_W_8	20

#define IMG_SIZE 	CAMERA_W_8 * CAMERA_H

/*#define CAMERA_W 	80
// #define CAMERA_W_8	10
// #define CAMERA_H 	60
// #define IMG_SIZE 	600   */

#define IMG_W      160
#define IMG_W_8    20
#define IMG_H     50
#define IMG_W_HALF 80

extern uint16 chang;
extern int16 tbRow;
extern int errtempL;
extern int errtempR;
extern int bian;
extern int bianr;

extern int countl;
extern int countr;



extern uint8 Image[IMG_SIZE + 4];     
extern uint8 ImageC[IMG_H][IMG_W];
//extern vuint8 Image2[50][CAMERA_W_8];

extern uint8 Image_Upload[IMG_H * IMG_W_8 + 4];   // 50*20+4
//extern uint8 *Image_UploadPtr;
extern uint8 Image_Row[IMG_H];
extern uint8 fill_Width[50];
extern uint8 upLoadSwitch;
extern int ef;

//������λ��
extern uint8 imgbuff[CAMERA_SIZE];   
extern uint8 img[CAMERA_W*CAMERA_H];
//extern void sendimg(uint8 *imgaddr, uint32 imgsize);
extern void img_extract(uint8 *dst, uint8 *src, uint32 srclen);

//test
extern int Lend_row[50], Rend_row[50], mid[50], slope_point[50];
extern char width[50];

extern int row,col;	
extern int centerLine;
extern int centerLineSub;
extern int edgetemp;
extern int lastLine;

typedef struct {
	int start;			//���߼�����ʼλ��
	int end;			//���߼��Ľ���λ��
	int offset;			//���ڴ��Ԥ����ߵ�ƫ��ֵ
	int lastLine;	//���ݱ任��
	int8 find;
}Search_Edge;
extern Search_Edge searchLeftEdge, searchRightEdge;

typedef struct {
	int leftEdge;
	int rightEdge;
	int total;
	int bothMissBool;
}MissCnt;
extern MissCnt missCnt;

typedef struct {		//ʮ�ֲ���
	int bool;
	int temp;
	int start_row;
	int end_row;
//	int cnt;
}FillEdge;
extern FillEdge filledge;

typedef struct {		//�ϰ���
	int start_row;
	int side;			//0xa ��� 0xb �ұ�
	int cnt;
	int bool;
}Obstacle;		
extern Obstacle obstacle;
//typedef enum {
//	bothLine,
//	leftLine,
//	rightLine,
//	bothLose,
//	noAvail,
//	Other,
//}img_Info;



typedef struct {
	int leftEdge;	//0.��߽�
	int rightEdge;	//1.�ұ߽�
	int middle;		//2.����
	int Info;		//3.��ȡ���ı߽���Ϣ 
						//Info = 1 ����������Ϊ �����Ҷ����� BOTHLINEABLE
						//Info = 2 ����������Ϊ �����		 LEFTABLE
						//Info = 3 ����������Ϊ �ҿ���		 
						//Info = 4 ����������Ϊ ���߶�ʧ
						//Info = 5 ���һ�в�����û�����߶���Ϊ5 	��֡ͼ��ʹ�õ�����Զһ��			
	int slope;		//4.б��\����
}Img_EdgeInfo;

typedef struct {
  int starthang;        //�����ſ���ʼ��
  int count;             //��¼ ��count����Ԥ��ֵʱ�ж�Ϊ��������
  int countR;
  
}CircleInfo;
//extern CircleInfo circleInfom;

extern Img_EdgeInfo img_EdgeInfo[IMG_H];
//extern int img_EdgeInfo[IMG_H][5];//ͼ��߽���Ϣ
//[0]����߽� [1]���ұ߽� [2]������ [3]����ȡ���ı߽���Ϣ [4]��б��\����


extern void Send_Img(void);
extern void Img_Pro(void);
#endif	 