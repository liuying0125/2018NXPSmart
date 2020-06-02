#include "include.h"
#include "common.h"

uint8 Image[IMG_SIZE + 4] = {0};
uint8 ImageC[IMG_H][IMG_W];
//vuint8 Image2[50][CAMERA_W_8];
int  g_nUseLine[5]={35,35,35,35,35};

int16 tbRow=0;          //跳变行总和

uint8 Image_Upload[IMG_H * IMG_W_8 + 4];
//uint8 *Image_UploadPtr;
//uint8 Image_Row[50] = { 14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,35,37,39,41,43,45,47,49,51,53,55,57,59,61,64,67,70,73,76,79,82,85,89,93,97,101,105,109,114,119 };
//uint8 Image_Row[50] = {8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,94,96,98,100,102,104,106};


//uint8 Image_Row[50] = { 12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,31,33,35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,65,67,69,71,74,77,80,83,86,89,92,95,98,101,105 };
//uint8 Image_Row[50] = {5,6,7,9,10,11,13,14,16,17,19,20,21,22,24,25,27,29,30,32,34,35,37,38,39,41,42,44,45,47,48,50,51,53,54,56,57,59,60,62,63,65,66,68,69,71,72,74,76,78};

 uint8 Image_Row[50] = {
50,51,52,53,54,
55,56,57,58,59,
60,61,62,63,64,
65,66,67,68,69,
70,71,72,73,74,
75,76,77,78,79,
80,81,82,83,84,
85,86,87,88,89,
90,91,92,93,94,
95,96,97,98,99
};
 


/*uint8 Image_Row[120] = {
0,1,2,3,4,5,6,7,8,9,10,
11,12,13,14,15,16,17,18,19,20,
21,22,23,24,25,26,27,28,29,30,
31,32,33,34,35,36,37,38,39,40,
41,42,43,44,45,46,47,48,49,
50,51,52,53,54,
55,56,57,58,59,
60,61,62,63,64,
65,66,67,68,69,
70,71,72,73,74,
75,76,77,78,79,
80,81,82,83,84,
85,86,87,88,89,
90,91,92,93,94,
95,96,97,98,99,
100,101,102,103,104,105,
106,107,108,109,110,111,
112,113,114,115,116,117,
118,119
};*/




///备份




//uint8 fill_Width[50] = { 21,22,24,25,26,28,29,30,31,32,32,35,35,37,37,39,39,40,41,42,43,44,45,46,47,48,48,49,50,51,52,54,53,55,56,55,56,56,57,58,58,59,59,60,60,61,61,61,62,62 };
//uint8 fill_WidthHalf[50] = { 21,22,24,25,26,28,29,30,31,32,32,35,36,37,37,39,39,40,41,42,43,44,45,46,46,47,48,49,50,51,51,53,53,55,55,55,56,56,57,57,58,58,59,59,60,60,61,62,62,62 };
uint8 fill_WidthHalf[50] = { 11,12,12,13,15,15,15,15,17,17,17,17,18,18,19,19,20,20,23,23,25,25,26,27,28,29,30,31,32,32,34,35,35,37,38,39,40,41,42,43,45,47,48,49,51,52,53,55,57,59 };
//uint8 fill_Width[50] =   { 23,23,26,29,30,31,32,33,33,34,34,34,34,35,35,37,37,41,42,43,44,45,46,47,48,49,50,51,51,53,54,55,56,56,57,58,58,59,59,60,62,62,62,62,62,63,63,63,63 };
uint8 fill_Width[50] ={31,31,34,35,36,37,38,39,40,41,42,42,42,43,43,44,45,46,47,51,52,52,53,54,55,56,57,58,59,59,60,61,62,63,64,65,65,66,66,67,67,67,68,68,69,69,69,69,69,70};
//uint8 fill_Width[50] =     { 9,10,11,11,12,12,13,13,14,14,15,16,16,17,17,18,19,21,22,23,24,25,26,27,28,30,31,32,33,34,35,36,37,38,39,40,42,44,45,47,49,50,52,53,55,57,59};
//uint8 fill_Width[50] =     { 28,30,31,33,35,36,38,39,40,42,43,44,44,47,48,49,49,50,51,52,53,53,54,55,56,57,57,58,59,59,60,62,62,62,62,62,62,62,62,63,63,63,63,63,63,63,63,63,63,63 };
uint8 upLoadSwitch = 1;
int Lend_row[50], Rend_row[50], mid[50], slope_point[50];
char width[50];

int row,col;
int centerLine = CAMERA_W / 2;   //80
int centerLineSub;
int edgetemp;
int lastLine;
int ef = 0;
int gh=0;

int errtempR;
int errtempL;
int errL;
int errR;
int bian;
int bianr;

int countl;
int countr;
uint16 chang=0;

double slope[3];
double XSum;
double YSum;
double XYSum;
double X2Sum;

Search_Edge searchLeftEdge, searchRightEdge;

MissCnt missCnt;

FillEdge filledge;

Obstacle obstacle;

Img_EdgeInfo img_EdgeInfo[IMG_H];
//int img_EdgeInfo[IMG_H][5];//图像边界信息
//[0]：左边界 [1]：右边界 [2]：中线 [3]：获取到的边界信息 [4]：斜率\曲率

//发送上位机
uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
uint8 img[CAMERA_W*CAMERA_H];

/*

150fps情况下，经过计算80*60的图像每个PCLK为1s/150fps/(80*60) = 1388.88ns，由于鹰眼经过了硬件二值化，
所以一个PCLK脉冲输出8个像素的二值化值，所以实际PCLK应该如下：

  图像大小       帧数      真实PCLK周期    二值化PCLK周期    HREF周期    VSNY周期
  80 * 60       150fps      1388.88ns     11111.11ns      111.11us    6.666ms
  160*120       150fps      347.222ns      2777.77ns      55.55us     6.666ms
  240*180       150fps      154.321ns      1234.56ns      37.03us     6.666ms
  320*240       150fps      86.8055ns      694.444ns      27.77us     6.666ms
  640*480       60fps       54.2534ns      694.444ns      34.72us     16.66ms

*/

void Send_Img(void){
	int i;
	// Uartn_Putchar(0, (char)0x01);
	// Uartn_Putchar(0, (char)0xFE);
	// Uart0_DMAPuts(&Image[0],IMG_SIZE);
	// //Uartn_Puts(0, (char*)Image[0]);   
	// Uartn_Putchar(0, (char)0xFE);
	// Uartn_Putchar(0, (char)0x01);

	/*山外多功能调试助手图像传输协议 图像在开始标识0x01 0xFE 
	和 结束标识 0xFE 0x 01之间才可被识别
	*/
        if (upLoadSwitch == 0) {
	//设置本幅图像的结束标识
	Image[IMG_SIZE] = 0xFE;
	Image[IMG_SIZE + 1] = 0x01;
	//设置下一幅图像的开始标识
	Image[IMG_SIZE + 2] = 0x01;
	Image[IMG_SIZE + 3] = 0xFE;
	//采集的原始图像
        uart_putbuff(UART2, &Image[0], IMG_SIZE + 4);       
	//Uart0_DMAPuts(&Image[0],IMG_SIZE + 4);
        }
	//取过50行后的图像+中线
	else if (upLoadSwitch == 1) {
		for (row = IMG_H - 1; row >= 0; row--) {
			if (img_EdgeInfo[row].Info == 5)break;
			Image_Upload[row * 20 + (uint8)(img_EdgeInfo[row].middle / 8)] |= 1 << (7 - img_EdgeInfo[row].middle & 0x07); //0000 0111
		}
		for (i = 0; i < 20; i++) {
			Image_Upload[g_nUseLine[0] * 20 + i] = (uint8)~Image_Upload[g_nUseLine[0] * 20 + i];
			Image_Upload[g_nUseLine[1] * 20 + i] = (uint8)~Image_Upload[g_nUseLine[1] * 20 + i];
			Image_Upload[g_nUseLine[2] * 20 + i] = (uint8)~Image_Upload[g_nUseLine[2] * 20 + i];								
		}
		//设置本幅图像的结束标识
		Image_Upload[IMG_H * IMG_W_8] = 0xFE;
		Image_Upload[IMG_H * IMG_W_8 + 1] = 0x01;
		//设置下一幅图像的开始标识
		Image_Upload[IMG_H * IMG_W_8 + 2] = 0x01;
		Image_Upload[IMG_H * IMG_W_8 + 3] = 0xFE;
		//采集的原始图像

		//Uartn_Putchar(0, (char)0x01);
		//Uartn_Putchar(0, (char)0xFE);
                uart_putbuff(UART2, &Image_Upload[0], IMG_H * IMG_W_8 + 4); 
		//Uart0_DMAPuts(&Image_Upload[0], IMG_H * IMG_W_8 + 4);
		//Uartn_Putchar(0, (char)0xFE);
		//Uartn_Putchar(0, (char)0x01);
	}
	//边界+中线
	else if (upLoadSwitch == 2) {
		for (row = IMG_H - 1; row >= 0; row--) {
			if (img_EdgeInfo[row].Info == 5)break;
			Image_Upload[row * 20 + (uint8)(img_EdgeInfo[row].middle / 8)] |= 1 << (7 - img_EdgeInfo[row].middle & 0x07);
			Image_Upload[row * 20 + (uint8)(img_EdgeInfo[row].leftEdge / 8)] |= 1 << (7 - img_EdgeInfo[row].leftEdge & 0x07);
			Image_Upload[row * 20 + (uint8)(img_EdgeInfo[row].rightEdge / 8)] |= 1 << (7 - img_EdgeInfo[row].rightEdge & 0x07);
		}
		/*for (i = 0; i < 20; i++) {
			Image_Upload[g_nUseLine[0] * 20 + i] = (uint8)~Image_Upload[g_nUseLine[0] * 20 + i];
			Image_Upload[g_nUseLine[1] * 20 + i] = (uint8)~Image_Upload[g_nUseLine[1] * 20 + i];
			Image_Upload[g_nUseLine[2] * 20 + i] = (uint8)~Image_Upload[g_nUseLine[2] * 20 + i];
			
		}*/
		//设置本幅图像的结束标识
			Image_Upload[IMG_H * IMG_W_8] = 0xFE;
		Image_Upload[IMG_H * IMG_W_8 + 1] = 0x01;
		//设置下一幅图像的开始标识
		Image_Upload[IMG_H * IMG_W_8 + 2] = 0x01;
		Image_Upload[IMG_H * IMG_W_8 + 3] = 0xFE;

		//采集的原始图像

		//Uartn_Putchar(0, (char)0x01);
		//Uartn_Putchar(0, (char)0xFE);
                uart_putbuff(UART2, &Image_Upload[0], IMG_H * IMG_W_8 + 4); 
		//Uart0_DMAPuts(&Image_Upload[0], IMG_H * IMG_W_8 + 4);
		//Uartn_Putchar(0, (char)0xFE);
		//Uartn_Putchar(0, (char)0x01);
                
                printf("upload = 2\n\r");
	}
	else if (upLoadSwitch == 3) {
		for (row = IMG_H - 1; row >= 0; row--) {
			if (img_EdgeInfo[row].Info == 5)break;
			Image_Upload[row * 20 + (uint8)(img_EdgeInfo[row].middle / 8)] |= 1 << (7 - img_EdgeInfo[row].middle & 0x07);
			Image_Upload[row * 20 + (uint8)(img_EdgeInfo[row].leftEdge / 8)] |= 1 << (7 - img_EdgeInfo[row].leftEdge & 0x07);
			Image_Upload[row * 20 + (uint8)(img_EdgeInfo[row].rightEdge / 8)] |= 1 << (7 - img_EdgeInfo[row].rightEdge & 0x07);
		}

		//设置本幅图像的结束标识
		Image_Upload[IMG_H * IMG_W_8] = 0xFE;
		Image_Upload[IMG_H * IMG_W_8 + 1] = 0x01;
		//设置下一幅图像的开始标识
		Image_Upload[IMG_H * IMG_W_8 + 2] = 0x01;
		Image_Upload[IMG_H * IMG_W_8 + 3] = 0xFE;
		//采集的原始图像

		//Uartn_Putchar(0, (char)0x01);
		//Uartn_Putchar(0, (char)0xFE);
                uart_putbuff(UART2, &Image_Upload[0], IMG_H * IMG_W_8 + 4); 
		//Uart0_DMAPuts(&Image_Upload[0], IMG_H * IMG_W_8 + 4);
		//Uartn_Putchar(0, (char)0xFE);
		//Uartn_Putchar(0, (char)0x01);
	}
	//预测边界+预测中线
	else if (upLoadSwitch == 4) {
		for (row = IMG_H - 1; row >= 0; row--) {
			if (img_EdgeInfo[row].Info == 5)break;
			Image_Upload[row * 20 + (uint8)(Lend_row[row] / 8)] |= 1 << (7 - Lend_row[row] & 0x07);
			Image_Upload[row * 20 + (uint8)(Rend_row[row] / 8)] |= 1 << (7 - Rend_row[row] & 0x07);
			Image_Upload[row * 20 + (uint8)(mid[row] / 8)] |= 1 << (7 - mid[row] & 0x07);
		}
		//设置本幅图像的结束标识
		Image_Upload[IMG_H * IMG_W_8] = 0xFE;
		Image_Upload[IMG_H * IMG_W_8 + 1] = 0x01;
		//设置下一幅图像的开始标识
		Image_Upload[IMG_H * IMG_W_8 + 2] = 0x01;
		Image_Upload[IMG_H * IMG_W_8 + 3] = 0xFE;
		//采集的原始图像

		//Uartn_Putchar(0, (char)0x01);
		//Uartn_Putchar(0, (char)0xFE);
                uart_putbuff(UART2, &Image_Upload[0], IMG_H * IMG_W_8 + 4); 
		//Uart0_DMAPuts(&Image_Upload[0], IMG_H * IMG_W_8 + 4);
		//Uartn_Putchar(0, (char)0xFE);
		//Uartn_Putchar(0, (char)0x01);
	}	

}

/*void sendimg(uint8 *imgaddr, uint32 imgsize)
{
	uint8 cmd[4] = { 0xFE, 0x01 , 0x01,0xFE};    //帧头帧尾   

	uart_putbuff(UART1, cmd, sizeof(cmd));    //先发送命令

	uart_putbuff(UART1, imgaddr, imgsize); //再发送图像
}*/

//压缩二值化图像解压（空间 换 时间 解压）
//srclen 是二值化图像的占用空间大小
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
	uint8 colour[2] = { 255, 0 }; //0 和 1 分别对应的颜色
	//注：山外的摄像头 0 表示 白色，1表示 黑色
	uint8 tmpsrc;
	while (srclen--)
	{
		tmpsrc = *src++;
		*dst++ = colour[(tmpsrc >> 7) & 0x01];    
		*dst++ = colour[(tmpsrc >> 6) & 0x01];
		*dst++ = colour[(tmpsrc >> 5) & 0x01];
		*dst++ = colour[(tmpsrc >> 4) & 0x01];
		*dst++ = colour[(tmpsrc >> 3) & 0x01];
		*dst++ = colour[(tmpsrc >> 2) & 0x01];
		*dst++ = colour[(tmpsrc >> 1) & 0x01];
                *dst++ = colour[(tmpsrc >> 0) & 0x01];
	}
};
		

void Slope_Pro(void) {
	int i;
	int m;
	for (i = 5; i < 25; i++)      
        {
		XSum += i;
		YSum += img_EdgeInfo[i].middle;
		XYSum += i * img_EdgeInfo[i].middle;
		X2Sum += i * i;
	}
	slope[0] = ((25 - 5) * XYSum - XSum * YSum) / ((25 - 5) * X2Sum - XSum * XSum);

	slope_point[(25 - 5) / 2] = YSum / (25 - 5);
	m = YSum / (25 - 5);
	for (i = 5; i < 25; i++) {
		slope_point[i] = m + (i - (25 - 5) / 2) * slope[0];
	}
}

void Img_Pro(void) {                            //定义这个函数
	vuint8 *ptr;          
	uint32 data;
        int16 tbTotal=0;                //跳变列和       
        int countR=0;
        int countR2=0;
         int ab=0;
        int cd=0;
         char buffer[30];
	int i, j;
	//char buffer[5];
	//PIT0_ENABLE();
	//MCF_PIT0_PMR = 6000;   //设置计数值

	for (row = 0; row < IMG_H; row++)   //从0到49   ，从第一行到最后一行
        {
                
		ptr = &ImageC[row][0];
		for (col = 0; col < IMG_W_8; col++) {    // IMG_W_8=20
			data = Image[Image_Row[row]* 20 + col];  //Image_Row[row]  
                //  data = Image[row* 20 + col];
			if (chang >= 15)
                        {
				if (upLoadSwitch == 1) 
                                {
					Image_Upload[row * 20 + col] = (uint8)data;
				}
				else if (upLoadSwitch == 2 | upLoadSwitch == 3)
				{
					Image_Upload[row * 20 + col] = 0;
				}

			}
			*ptr++ = (char)(data & 0x80);      //  1000 0000 
			data = data << 1;
			*ptr++ = (char)(data & 0x80);
			data = data << 1;
			*ptr++ = (char)(data & 0x80);
			data = data << 1;
			*ptr++ = (char)(data & 0x80);
			data = data << 1;
			*ptr++ = (char)(data & 0x80);
			data = data << 1;
			*ptr++ = (char)(data & 0x80);
			data = data << 1;
			*ptr++ = (char)(data & 0x80);
			data = data << 1;
			*ptr++ = (char)(data & 0x80);
			data = data << 1;
		}
	}

      // 图像处理开始

	//Uartn_Putchar(0, (char)0x01);
	//Uartn_Putchar(0, (char)0xFE);
	//Uart0_DMAPuts(&ImageC[0][0], 19200);
	//Uartn_Putchar(0, (char)0xFE);
	//Uartn_Putchar(0, (char)0x01);

	centerLine = IMG_W_HALF;     // center line 中线  80
	centerLineSub = 0;
	lastLine = IMG_H - 1;
        
	searchLeftEdge.start = IMG_W_HALF - 1;
	searchLeftEdge.end = 1;
	searchLeftEdge.offset = 0;   
	searchLeftEdge.find = 0;
	searchLeftEdge.lastLine = IMG_H - 1;

	searchRightEdge.start = IMG_W_HALF + 1;
	searchRightEdge.end = IMG_W - 1;
	searchRightEdge.offset = 0;
	searchRightEdge.find = 0;
	searchRightEdge.lastLine = IMG_H - 1;

	missCnt.total = 0;
	missCnt.leftEdge = 0;
	missCnt.rightEdge = 0;
	missCnt.bothMissBool = 0;

	filledge.temp = 0;
	filledge.bool = 0;
	filledge.start_row = 0;
	filledge.end_row = 0;

	obstacle.bool = 0;
	obstacle.cnt = 0;
	obstacle.start_row = 0;
	obstacle.side = 0;
        int count1;
        int count2 = 0;
        int count3;
        int count4;
        
        
        
 	for (row = IMG_H - 1; row >= 0; row--) {          //大循环  从最里面一行开始扫描    从下到上
               /*   tbTotal=0;
                  count1=0; 
                  count2=0;
                  count3=0;
                  count4=0;*/

		searchLeftEdge.start = centerLine;                       
		searchRightEdge.start = centerLine;
		searchLeftEdge.find = 0;
		searchRightEdge.find = 0;

		img_EdgeInfo[row].leftEdge = 0;
		img_EdgeInfo[row].rightEdge = 0;
		img_EdgeInfo[row].middle = 0;
		img_EdgeInfo[row].Info = 0;
		img_EdgeInfo[row].slope = 0;

		//若中间线为黑线，则表示检测到结尾
		if (ImageC[row][centerLine] == 0x80) {              //    0x80   1000 0000 
			//排除图像中间可能有的噪点
			if ((row == IMG_H - 1 || (row > 5 && img_EdgeInfo[row + 1].Info == 1)) && ImageC[(uint8)(row * 0.8)][centerLine] != 0x80) {
				i = (int)(row * 0.8);
				while (i <= row) {
					img_EdgeInfo[i].Info = 6;
					if (row == IMG_H - 1)
						img_EdgeInfo[i].middle = 80;
					else
						img_EdgeInfo[i].middle = img_EdgeInfo[row + 1].middle;
					i++;
				}
				row *= 0.8;
				continue;
			}
			else {		//本幅图像遍历完毕.
				//lastLine = row + 1;
				img_EdgeInfo[row].Info = 5;
				break;
			}
		}
                //环形赛道中间黑圆判断************************************************
                /*
                for(col=5;col<80;col++){
                  if(!ImageC[row][col]){
                          count1++;
                          
                  }               
                }
                 for(col=80;col<155;col++){
                  if(!ImageC[row][col]){
                          count3++;
                          
                  }               
                }               //当前行白色点
                
                 for(col=50;col<80;col++){
                  if(ImageC[row][col]){
                          count2++;                        
                  }                
                }
                 for(col=80;col<110;col++){             
                  if(ImageC[row][col]){
                          count4++;                        
                  }                
                }                               //当前行黑色点
                
                if(row<42){
                  if((count1+count3)>130) countR2++;
                
                }
                if(count1>=21&&count2>=22&&count4>=22&&count3>=21&&((count2-count4)<4||(count2-count4)>-4))
                  countR++;
               */

		//寻找左边界
		for (col = 140; col >= searchLeftEdge.end; col -= 2) {
                  /*if(ImageC[row][col + 1] && ImageC[row][col])
                  {       if(col>72&&col<90){jtn
					if (ImageC[row][col + 1]==0 && ImageC[row][col]==0){
						img_EdgeInfo[row].leftEdge = col;
						searchLeftEdge.find = 1;
					}
				
                  }}
		else*/
                  
                  if (ImageC[row][col + 1] && ImageC[row][col]) {
				img_EdgeInfo[row].leftEdge = col + 1;
                                
                                
                                
               
                                
                                
                                
                               
                                //if(img_EdgeInfo[row+1].leftEdge-img_EdgeInfo[row].leftEdge>=4)//虚凸：row<
                                  ab++;
                               // if(img_EdgeInfo[row].leftEdge-img_EdgeInfo[row+1].leftEdge>=5)//实凸：row<
                                 // cd++;
                                
				searchLeftEdge.find = 1;
                                if(img_EdgeInfo[row].leftEdge>=80)
                               bianr++;
                                bian++;
                                
				if (row < IMG_H - 2 && img_EdgeInfo[row + 2].leftEdge) {
					//此处可能需要加入障碍物检测
					if (!missCnt.bothMissBool && obstacle.start_row == 0 && (img_EdgeInfo[row].leftEdge - img_EdgeInfo[row+2].leftEdge) > 20) {
						obstacle.start_row = row;
						obstacle.side = 0xa; //左边的障碍  // 0000 1010
						obstacle.cnt++;
					}
					else if (!missCnt.bothMissBool && (img_EdgeInfo[row].leftEdge - img_EdgeInfo[row + 1].leftEdge > 15 || img_EdgeInfo[row].leftEdge - img_EdgeInfo[row + 1].leftEdge < -15))
						img_EdgeInfo[row].leftEdge = img_EdgeInfo[row + 1].leftEdge;
				}
				/*左边偏差值计算*/
				if (row == IMG_H - 1) {
					searchLeftEdge.offset = 0;
					break;
				}
				edgetemp = img_EdgeInfo[row].leftEdge - img_EdgeInfo[row + 1].leftEdge;
				if (edgetemp && img_EdgeInfo[row + 1].leftEdge) {
					searchLeftEdge.offset = (0.1 * searchLeftEdge.offset + 0.9 * edgetemp) * 3.5;
				}
				else {
					searchLeftEdge.offset = 0;
				}
				searchLeftEdge.lastLine = row;
				break;
			}
		}

		//寻找找右边界
		for (col = searchRightEdge.start; col <= searchRightEdge.end; col += 2) {
                  
			if (ImageC[row][col - 1] && ImageC[row][col]) {
				img_EdgeInfo[row].rightEdge = col - 1;
                                 if(img_EdgeInfo[row].rightEdge>img_EdgeInfo[row+1].rightEdge)
                               cd++;
				searchRightEdge.find = 1;
				if (row < IMG_H - 2 && img_EdgeInfo[row + 2].rightEdge) {
					//此处可能需要加入障碍物检测
					if (!missCnt.bothMissBool && obstacle.start_row == 0 && (img_EdgeInfo[row].rightEdge - img_EdgeInfo[row + 2].rightEdge) < -20) {
						obstacle.start_row = row;
						obstacle.side = 0xb; //右边的障碍
						obstacle.cnt++;
					}
					else if (!missCnt.bothMissBool && (img_EdgeInfo[row].rightEdge - img_EdgeInfo[row + 1].rightEdge < -15 || img_EdgeInfo[row].rightEdge - img_EdgeInfo[row + 1].rightEdge > 15))
						img_EdgeInfo[row].rightEdge = img_EdgeInfo[row + 1].rightEdge;
				}
				/*右边偏差值计算*/
				if (row == IMG_H - 1) {
					searchLeftEdge.offset = 0;
					break;
				}
				edgetemp = img_EdgeInfo[row].rightEdge - img_EdgeInfo[row + 1].rightEdge;
				if (edgetemp && img_EdgeInfo[row + 1].rightEdge) {
					searchRightEdge.offset = (0.1 * searchRightEdge.offset + 0.9 * edgetemp) * 3.5;
				}
				else {
					searchRightEdge.offset = 0;
				}
				searchRightEdge.lastLine = row;
				break;
			}
		}


		if (searchLeftEdge.find && searchRightEdge.find)
                {
       
                
       
              
			/*1 设置中线类型*/
			img_EdgeInfo[row].Info = 1;

			/*2 预测并设置下一行边线的范围,减少数据处理量*/
			if (searchLeftEdge.offset > -4) {
				searchLeftEdge.end = img_EdgeInfo[row].leftEdge - 2 - 3;
			}
			else {
				searchLeftEdge.end = img_EdgeInfo[row].leftEdge - 2 + searchLeftEdge.offset;
			}
			if (searchRightEdge.offset < 4) {
				searchRightEdge.end = img_EdgeInfo[row].rightEdge + 2 + 3;
			}
			else {
				searchRightEdge.end = img_EdgeInfo[row].rightEdge + 2 + searchRightEdge.offset;
			}

			/*3 求出中线的位置*/
			img_EdgeInfo[row].middle = (img_EdgeInfo[row].leftEdge + img_EdgeInfo[row].rightEdge) * 0.5;

			/*4 保存最后一次中线的行列*/

			/*5 计算中线的偏差值*/
                 
			if (row < IMG_H - 1 && img_EdgeInfo[row + 1].middle) {
				centerLineSub = centerLineSub * 0.1 + (img_EdgeInfo[row].middle - img_EdgeInfo[row + 1].middle) * 0.9;
			}
			else {
				centerLineSub = 0;
			}
			/*6 设置预测的中线值*/
			centerLine = img_EdgeInfo[row].middle + centerLineSub;


			//if (filledge.bool) {
			//	//filledge.temp = -1;
			//	filledge.start_row = 0;
			//}
			filledge.temp = 0;
			if (filledge.bool != 0 && filledge.start_row != 0 && filledge.end_row == 0) {   //需要补线,开始行不为0,结束行为0
				filledge.end_row = row;
			}

			//第一列宽度小于80 判定为正在过障碍
			if (row ==  IMG_H - 2 && (img_EdgeInfo[row].rightEdge - img_EdgeInfo[row].leftEdge < 85)) {     
				obstacle.start_row = IMG_H - 2;
				obstacle.bool = 1;
			}
			if (obstacle.side == 0xa && (searchLeftEdge.offset < 10 && searchLeftEdge.offset > -4)) {
				obstacle.cnt++;
				if (obstacle.cnt > 5) {
					obstacle.bool = 1;//认为有障碍物
				}
			}
			if (obstacle.side == 0xb && (searchRightEdge.offset < 4 && searchRightEdge.offset > -10)) {
				obstacle.cnt++;
				if (obstacle.cnt > 5) {
					obstacle.bool = 1;
				}
			}
		}
		else if (searchLeftEdge.find) {
			/*1 设置中线类型*/
			img_EdgeInfo[row].Info = 2;
			/*2 预测并设置下一行边线的范围*/
			if (missCnt.bothMissBool) {
				if (searchLeftEdge.offset) {
					searchLeftEdge.end += searchLeftEdge.offset * 0.1;
				}
				else {
					searchLeftEdge.end += 1;
				}
				if (searchRightEdge.offset) {
					searchRightEdge.end += searchRightEdge.offset * 0.1;
				}
				else {
					searchRightEdge.end -= 1;
				}
			}
			else if (searchLeftEdge.offset > -4) {
				searchLeftEdge.end = img_EdgeInfo[row].leftEdge - 2 - 3;
				searchRightEdge.end += searchRightEdge.offset * 0.1;
			}
			else {
				searchLeftEdge.end = img_EdgeInfo[row].leftEdge - 2 + searchLeftEdge.offset;
				searchRightEdge.end += searchRightEdge.offset * 0.1;
			}
			/*3 求出中线的位置*/
			//if (!missCnt.bothMissBool) {
			//	img_EdgeInfo[row].middle = (img_EdgeInfo[row].leftEdge + searchRightEdge.end) * 0.5;	//暂时处理方法 之后 可以采用偏差进行计算
			//}
                       // errL=img_EdgeInfo[row].leftEdge-img_EdgeInfo[row+2].leftEdge;
                    //    errR=img_EdgeInfo[row].leftEdge-img_EdgeInfo[row+2].leftEdge
                        if(img_EdgeInfo[row].leftEdge-img_EdgeInfo[row+2].leftEdge>8){
                          
                                  errtempL+=(img_EdgeInfo[row].leftEdge-img_EdgeInfo[row+2].leftEdge);
                                  countl++;
                        
                        }
                      
               
                        
                        
                        
                        if(img_EdgeInfo[row].leftEdge-img_EdgeInfo[row+2].leftEdge<-3){
                                  errtempR+=(img_EdgeInfo[row+2].leftEdge-img_EdgeInfo[row].leftEdge);
                                  countr++;
                        }

                        
			if (obstacle.bool) {
				img_EdgeInfo[row].rightEdge = searchRightEdge.end - 30;//刘建业修改
				img_EdgeInfo[row].middle = (img_EdgeInfo[row].leftEdge + img_EdgeInfo[row].rightEdge) * 0.5;
			}
			else {
				img_EdgeInfo[row].middle = img_EdgeInfo[row].leftEdge + fill_WidthHalf[row];/////为什么没有障碍物就要补线，
			}
			/*4 保存最后一次中线的行列*/

			/*5 计算中线的偏差值*/


			/*6 设置预测的中线值*/
			//centerLine = img_EdgeInfo[row].middle + centerLineSub;
			centerLine += centerLineSub;

			missCnt.rightEdge++;
			if (filledge.bool == 0 && filledge.temp == 0) {
				filledge.temp = row;
			}

		}
		else if (searchRightEdge.find) {
			/*1 设置中线类型*/
			img_EdgeInfo[row].Info = 3;
			/*2 预测并设置下一行边界线的范围*/
			if (missCnt.bothMissBool) {
				if (searchLeftEdge.offset) {
					searchLeftEdge.end += searchLeftEdge.offset * 0.1;
				}
				else {
					searchLeftEdge.end += 1;
				}
				if (searchRightEdge.offset) {
					searchRightEdge.end += searchRightEdge.offset * 0.1;
				}
				else {
					searchRightEdge.end -= 1;
				}
			}
			else if (searchRightEdge.offset < 4) {
				searchLeftEdge.end += searchLeftEdge.offset * 0.1;
				searchRightEdge.end = img_EdgeInfo[row].rightEdge + 2 + 3;
			}
			else {
				searchLeftEdge.end += searchLeftEdge.offset * 0.1;
				searchRightEdge.end = img_EdgeInfo[row].rightEdge + 2 + searchRightEdge.offset;
			}
			/*3 求出中线的位置*/
			//if (!missCnt.bothMissBool) {
			//	img_EdgeInfo[row].middle = (searchLeftEdge.end + img_EdgeInfo[row].rightEdge) * 0.5;	//暂时处理方法 之后 可以采用偏差进行计算
			//}
			if (obstacle.bool) {
				img_EdgeInfo[row].leftEdge = searchLeftEdge.end + 30;/////刘建业修改
                    
                                
				img_EdgeInfo[row].middle = (img_EdgeInfo[row].leftEdge + img_EdgeInfo[row].rightEdge) * 0.5;
			}
			else {
				img_EdgeInfo[row].middle = img_EdgeInfo[row].rightEdge - fill_WidthHalf[row];
			}
			/*4 保存最后一次中线的行列*/

			/*5 计算中线的偏差值*/


			/*6 设置预测的中线值*/
			//centerLine = img_EdgeInfo[row].middle + centerLineSub;
			centerLine += centerLineSub;

			missCnt.leftEdge++;
			if (filledge.bool == 0 && filledge.temp == 0) {
				filledge.temp = row;
			}
		}
		else {
			/*1 设置中线类型*/
			img_EdgeInfo[row].Info = 4;

			/*2 预测并设置下一行边线的范围*/
			//if (missCnt.bothMissBool) {
			//	searchLeftEdge.end += 2;
			//	searchRightEdge.end -= 2;
			//}
			//searchLeftEdge.start = centerLine - 1;
			//searchRightEdge.start = centerLine + 1;
			/*3 求出中线的位置*/

			/*4 保存最后一次中线的行列*/

			/*5 计算中线的偏差值*/


			/*6 设置预测的中线值*/
			img_EdgeInfo[row].middle = centerLine;	//预设值 如果十字补线成立则被覆盖
			centerLine = centerLine + centerLineSub;

			/*7 设置预测的中线值*/


			missCnt.total++;
			if (filledge.bool == 0) {
				if (filledge.temp == 0 && missCnt.total > 0) {
					filledge.temp = row;
				}
				if (missCnt.total > 3) {                        //两边丢线超过三行,至missCnt.bothMissBool = 1
					missCnt.bothMissBool = 1;
				}
				if (missCnt.bothMissBool) {                       //bothMiss时,确定需要十字补线      
					filledge.bool = 1;
					filledge.start_row = filledge.temp;       //补线开始行 赋值 当前行
				}
			}
		}

		if (searchLeftEdge.end < 1) {
			searchLeftEdge.end = 1;
		}
		if (searchRightEdge.end > IMG_W - 1) {
			searchRightEdge.end = IMG_W - 1;
		}
		if (centerLine > 157) {
			centerLine = 157;
		}
		else if (centerLine < 3) {
			centerLine = 3;
		}

		centerLine = centerLine * 0.6 + ((searchLeftEdge.end + searchRightEdge.end) * 0.5) * 0.4;

		//width[row] = img_EdgeInfo[row].rightEdge - img_EdgeInfo[row].leftEdge;
		Lend_row[row] = searchLeftEdge.end;
		Rend_row[row] = searchRightEdge.end;
		mid[row] = centerLine;
		//Image_Upload[row * 20 + (uint8)(img_EdgeInfo[row].middle / 8)] |= 1 << (7 - img_EdgeInfo[row].middle & 0x07);

	}       
        
	img_EdgeInfo[0].Info = 5;

	if (searchLeftEdge.lastLine < searchRightEdge.lastLine)
        {
		lastLine = searchLeftEdge.lastLine;
	}
	else {
		lastLine = searchRightEdge.lastLine;
	}
	//sprintf(TXBuffer, "leftEdge:%d rightEdge:%d bothMiss:%d\r\n", missCnt.leftEdge, missCnt.rightEdge, missCnt.total);
	//sprintf(TXBuffer, "leftEdge:%d rightEdge:%d", searchLeftEdge.end, searchRightEdge.end);
    // sprintf(buffer, "leftEdge:%d rightEdge:%d bian:%d bianr:%d countl:%d countr:%d\r\n",errtempL ,errtempR,bian,bianr,countl,countr);
     // sprintf(buffer, "leftEdge:%d rightEdge:%d \r\n\r\n",errtempL ,errtempR,bian,bianr,countl,countr);
     
       uart_putstr(UART0,buffer);
         if(errtempL >=100&&errtempR>=10&&bian>=40&&countl>=4&&countr>=3&&bianr>=20&&bianr<=40)
         {
           ef++;
           
         }
         
	errtempL=0;
        errtempR=0;
        bian=0;
        countl=0;
        countr=0;
         bianr=0;
       

}

