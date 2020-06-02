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

//发送上位机
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
	int start;			//边线检测的起始位置
	int end;			//边线检测的结束位置
	int offset;			//用于存放预测边线的偏差值
	int lastLine;	//横纵变换用
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

typedef struct {		//十字补线
	int bool;
	int temp;
	int start_row;
	int end_row;
//	int cnt;
}FillEdge;
extern FillEdge filledge;

typedef struct {		//障碍物
	int start_row;
	int side;			//0xa 左边 0xb 右边
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
	int leftEdge;	//0.左边界
	int rightEdge;	//1.右边界
	int middle;		//2.中线
	int Info;		//3.获取到的边界信息 
						//Info = 1 中线类型设为 左中右都可用 BOTHLINEABLE
						//Info = 2 边线类型设为 左可用		 LEFTABLE
						//Info = 3 中线类型设为 右可用		 
						//Info = 4 中线类型设为 三线丢失
						//Info = 5 最后一行不管有没有中线都设为5 	本帧图像使用到的最远一行			
	int slope;		//4.斜率\曲率
}Img_EdgeInfo;

typedef struct {
  int starthang;        //赛道张开开始行
  int count;             //记录 当count大于预设值时判定为环形赛道
  int countR;
  
}CircleInfo;
//extern CircleInfo circleInfom;

extern Img_EdgeInfo img_EdgeInfo[IMG_H];
//extern int img_EdgeInfo[IMG_H][5];//图像边界信息
//[0]：左边界 [1]：右边界 [2]：中线 [3]：获取到的边界信息 [4]：斜率\曲率


extern void Send_Img(void);
extern void Img_Pro(void);
#endif	 