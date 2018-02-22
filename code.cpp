#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <map>
#include <sstream>

using namespace cv;
using namespace std;
#define PI 3.14159265

/*stores triplets of points*/
struct node{
	Point a1;
	Point a2;
	Point a3;
};

int minimum(int x1,int x2,int x3,int x4){
	int min = x1;
	if(x2<min)min=x2;
	if(x3<min)min=x3;
	if(x4<min)min=x4;
	return min;
}

int maximum(int x1,int x2,int x3,int x4){
	int max = x1;
	if(x2>max)max=x2;
	if(x3>max)max=x3;
	if(x4>max)max=x4;
	return max;
}

/*This code snippet to check whether a point lies within a triangle or not has been taken from an online site*/
/*Source: https://www.geeksforgeeks.org/check-whether-a-given-point-lies-inside-a-triangle-or-not/ */
float area(int x1, int y1, int x2, int y2, int x3, int y3){
   return abs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0);
}

bool isInside(int x1, int y1, int x2, int y2, int x3, int y3, int x, int y) {   
   float A = area (x1, y1, x2, y2, x3, y3);
   float A1 = area (x, y, x2, y2, x3, y3);
   float A2 = area (x1, y1, x, y, x3, y3);
   float A3 = area (x1, y1, x2, y2, x, y);
   return (A == A1 + A2 + A3);
}

// Draw delaunay triangles
static vector<node> draw_delaunay( Mat& img, Subdiv2D& subdiv ){
	vector<Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);
	vector<Point> pt(3);
	Size size = img.size();
	Rect rect(0,0, size.width, size.height);
	vector<node> V;
	
	for( size_t i = 0; i < triangleList.size(); i++ ){
		Vec6f t = triangleList[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		if ( rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2])){
			struct node A;
			A.a1 = pt[0];
			A.a2 = pt[1];
			A.a3 = pt[2];
			V.push_back(A);
		}
	}
	return V;
}

vector<node> genTriangles(Mat img , vector<Point2f> points){
	points.resize(points.size());
    Mat img_orig = img.clone();
    Size size = img.size();
    Rect rect(0, 0, size.width, size.height);
    Subdiv2D subdiv(rect);

    for( vector<Point2f>::iterator it = points.begin(); it != points.end(); it++){
        subdiv.insert(*it);
        Mat img_copy = img_orig.clone();
        draw_delaunay( img_copy, subdiv );
    }
    vector<node> V = draw_delaunay(img,subdiv);
    return V;
}

/*tri1 and tri2 are 4:1 matrices with points of triangle except reference point i.e origin*/
/*This function find the transformation matrix for given triangles*/
Mat warpAffin(float tri1[] , float tri2[]){
	float xyij[4][4] = {0};
	xyij[1][2] = xyij[0][0] = tri2[0];
	xyij[1][3] = xyij[0][1] = tri2[1];
	xyij[3][2] = xyij[2][0] = tri2[2];
	xyij[3][3] = xyij[2][1] = tri2[3];
	/*Matrix operations to generate the transformation matrix C (4:1) */	
	Mat M = Mat(4,4,CV_32F, xyij);
	Mat M1 = M.inv();
	Mat X = Mat(4,1,CV_32F , tri1);
	Mat C = M1 * X;
	return C;
}
/*Code for morphing between 2 images*/
void morphImage( Mat src , Mat dst , float morphs , string f1 , string f2 ){
	int size_m = 0;
	int temp_m = (int)morphs;
	while(temp_m!=0){
		size_m++;
		temp_m /= 10;
	}
	vector<Point2f> P1;
	vector<Point2f> P2;
    ifstream ifs ;
    ifs.open(f1.c_str());
    int x, y;
    while(ifs >> x >> y){
        P1.push_back(Point2f(x,y));
    }
	ifstream ifs2;
	ifs2.open(f2.c_str());
	while(ifs2 >> x >> y){
		P2.push_back(Point2f(x,y));
	}
	if(P1.size()!=P2.size()){
		cout << "Errors in tie points: they must be equal in number" << endl;
		return ;
	}
	float t = 1/morphs;
	for(float i=1 ; i<=morphs ; i++){
		vector<Point2f> temp_P; /*tie points in ith morph*/
		/*Find output image size*/
		int temp_rows = cvRound((1-i*t)*src.rows + t*i*dst.rows) ;
		int temp_cols = cvRound((1-i*t)*src.cols + t*i*dst.cols) ;
		Mat output = Mat::zeros( temp_rows , temp_cols , dst.type());
		vector<node> relations;
		for(int j=0 ; j<P1.size() ; j++){
			/*interpolation for tie points in the intermediate images*/
			int temp_x = cvRound((1-i*t)*P1[j].x + t*i*P2[j].x) ;
			int temp_y = cvRound((1-i*t)*P1[j].y + t*i*P2[j].y) ;
			Point2f temp(temp_x , temp_y);
			temp_P.push_back(temp);
			struct node a;
			a.a1 = P1[j];
			a.a2 = temp;
			a.a3 = P2[j];
			relations.push_back(a);
		}
		/*Delaunay Triangulation on all intermediate images*/
		vector<node> V_morph = genTriangles(output,temp_P);
		/*for all triangles keep calculating warpAffine and traverse through all vertices within the triangle of a output 			image and use bilinear to warp the image*/
		for(int k=0 ; k<V_morph.size() ; k++){
			float f1[4];
			float f2[4];
			float f3[4];
			
			Point2f p1_i,p2_i,p3_i;
			Point2f p1_o,p2_o,p3_o;
			/*find relational triplets*/
			for(int r=0 ; r<relations.size() ; r++){
				if( relations[r].a2 == V_morph[k].a1 ){
					p1_i = relations[r].a1;
					p1_o = relations[r].a3;
				}else if( relations[r].a2 == V_morph[k].a2 ){
					p2_i = relations[r].a1;
					p2_o = relations[r].a3;
				}else if( relations[r].a2 == V_morph[k].a3 ){
					p3_i = relations[r].a1;
					p3_o = relations[r].a3;
				}
			}
			/*Shift to origin for affine transformation*/
			f2[0] = V_morph[k].a2.x - V_morph[k].a1.x ;
			f2[1] = V_morph[k].a2.y - V_morph[k].a1.y ;
			f2[2] = V_morph[k].a3.x - V_morph[k].a1.x ;
			f2[3] = V_morph[k].a3.y - V_morph[k].a1.y ;
			
			f1[0] = p2_i.x - p1_i.x ;
			f1[1] = p2_i.y - p1_i.y ;
			f1[2] = p3_i.x - p1_i.x ;
			f1[3] = p3_i.y - p1_i.y ;
			
			f3[0] = p2_o.x - p1_o.x ;
			f3[1] = p2_o.y - p1_o.y ;
			f3[2] = p3_o.x - p1_o.x ;
			f3[3] = p3_o.y - p1_o.y ;
			
			/* Mappings from f2->f1 and f2->f3 i.e intermediate morph to input image 1 and intermediate morph to input image */
			Mat to_in = warpAffin(f1,f2);
			Mat to_out = warpAffin(f3,f2);
			
			/*Map all points within the triangle using matrix obtained and cross dissolve*/
			/*Find the wrapping rectangle i.e., circumrectangle to traverse */
			int start_x = min( V_morph[k].a1.x , V_morph[k].a2.x) ;
			start_x = min( start_x , V_morph[k].a3.x ) ;
			int end_x = max( V_morph[k].a1.x , V_morph[k].a2.x );
			end_x = max( end_x , V_morph[k].a3.x ) ;
			int start_y = min( V_morph[k].a1.y , V_morph[k].a2.y ) ;
			start_y = min( start_y , V_morph[k].a3.y ) ;
			int end_y = max( V_morph[k].a1.y , V_morph[k].a2.y );
			end_y = max( end_y , V_morph[k].a3.y ) ;

			for(int xx = start_y ; xx<end_y ; xx++){
				
				for(int yy = start_x ; yy<end_x ; yy++){
					if(isInside(V_morph[k].a1.x,V_morph[k].a1.y,V_morph[k].a2.x,V_morph[k].a2.y,V_morph[k].a3.x,V_morph[k].a3.y,yy,xx))
					{
						/*Calculating target points in image1*/
						float x_prime = p1_i.x + to_in.at<float>(0,0)*(yy-V_morph[k].a1.x) 
										+ to_in.at<float>(1,0)*(xx-V_morph[k].a1.y); 
						float y_prime = p1_i.y + to_in.at<float>(2,0)*(yy-V_morph[k].a1.x)
										+ to_in.at<float>(3,0)*(xx-V_morph[k].a1.y);
										
						float l_x = x_prime - (int)x_prime;
						float l_y = y_prime - (int)y_prime;
						int new_x = (int)x_prime ;
						int new_y = (int)y_prime ;
						
						float int_in[3]={0},int_out[3]={0};
						/*Bilinear Interpolation for intensity calculation*/
						if(new_x+1<src.cols && new_x>=0 && new_y+1<src.rows && new_y>=0){
							for(int rgb=0 ; rgb<3 ; rgb++){
								int_in[rgb] = (float)(1-l_x)*(1-l_y)*src.at<Vec3b>(new_y,new_x)[rgb]
										+ (float)(1-l_x)*l_y*src.at<Vec3b>(new_y,new_x+1)[rgb]
										+ (float)(1-l_y)*l_x*src.at<Vec3b>(new_y+1,new_x)[rgb]
										+ (float)l_x*l_y*src.at<Vec3b>(new_y+1,new_x+1)[rgb] ;
							}
						}
						/*Calculating target points in image2*/
						x_prime = p1_o.x + to_out.at<float>(0,0)*(yy-V_morph[k].a1.x) 
								  + to_out.at<float>(1,0)*(xx-V_morph[k].a1.y);
						y_prime = p1_o.y + to_out.at<float>(2,0)*(yy-V_morph[k].a1.x)
								  + to_out.at<float>(3,0)*(xx-V_morph[k].a1.y);
								  
						l_x = x_prime - (int)x_prime;
						l_y = y_prime - (int)y_prime;
						new_x = (int)x_prime;
						new_y = (int)y_prime;
						/*Bilinear Interpolation for intensity calculation*/
						if(new_x+1<dst.cols && new_x>=0 && new_y+1<dst.rows && new_y>=0){
							for(int rgb=0 ; rgb<3 ; rgb++){
								int_out[rgb] = (float)(1-l_x)*(1-l_y)*dst.at<Vec3b>(new_y,new_x)[rgb]
										  + (float)(1-l_x)*l_y*dst.at<Vec3b>(new_y,new_x+1)[rgb]
										  + (float)(1-l_y)*l_x*dst.at<Vec3b>(new_y+1,new_x)[rgb] 
										  + (float)l_x*l_y*dst.at<Vec3b>(new_y+1,new_x+1)[rgb] ;
							}
						
						}
						for(int rgb=0 ; rgb<3 ; rgb++){
							output.at<Vec3b>(xx, yy)[rgb] = (int)((1-i*t)*int_in[rgb] + i*t*int_out[rgb]) ;
						}
					}
				}
			}
		}
		//int i = 5;
		string s;
		stringstream out;
		out << i;
		s = out.str();
		while(s.length() < size_m){
			s = "0" + s;
		}
		s.append(".jpg");
		imwrite(s,output);
	}
}

/*Calculating intermediate images for affine transformation using tie points*/
Mat tiepoint( Mat output , float xyp[8] , float x1[4] , float y1[4] , Mat image , int frame ){
	float xyij[8][8] = {0};
	for(int i=0 ; i<8 ; i++){
		if(i%2==0){
			xyij[i][0] = x1[i/2] ;
			xyij[i][1] = y1[i/2] ;
			xyij[i][2] = x1[i/2]*y1[i/2] ;
			xyij[i][3] = 1 ;
	
		}else{
			xyij[i][4] = x1[i/2] ;
			xyij[i][5] = y1[i/2] ;
			xyij[i][6] = x1[i/2]*y1[i/2] ;
			xyij[i][7] = 1 ;
		}
	}
	/*Matrix operations to generate the transformation matrix C (8:1) */
	Mat M = Mat(8,8,CV_32F, xyij);
	Mat M1 = M.inv();
	Mat X = Mat(8,1,CV_32F , xyp);
	Mat C = M1 * X;
	int count=0;
	for(int f=0 ; f<8 ; f++){
		if( C.at<float>(f,0) == 0 ){
			count++;
		}
	}
	/*If the entire matrix C is zero then affine transformation is not possible hence an error message will be printed and a blank image will come as output*/
	if(count==8){
		cout << "Frame: " << frame << " Determinant Zero: Cannot create transformation\nDisturbance caused\nContinuing for further morphs...\n" << endl;
	}
	/* Image reconstruction using C*/
	for(int x=0 ; x<output.rows ; x++){
    	for(int y=0 ; y<output.cols; y++){
			double x_prime = (C.at<float>(0,0)*x + C.at<float>(1,0)*y + C.at<float>(2,0)*x*y + C.at<float>(3,0));
			double y_prime = (C.at<float>(4,0)*x + C.at<float>(5,0)*y + C.at<float>(6,0)*x*y + C.at<float>(7,0));
			double l_x = x_prime - (int)x_prime;
			double l_y = y_prime - (int)y_prime;
			int new_x = (int)x_prime;
			int new_y = (int)y_prime;
			/*Bilinear Interpolation*/
			for(int rgb=0 ; rgb<3 ; rgb++){
				if(new_x<image.rows && new_x>=0 && new_y<image.cols && new_y>=0){
					output.at<Vec3b>(x,y)[rgb] = (int)((1-l_x)*(1-l_y)*image.at<Vec3b>(new_x,new_y)[rgb] + (1-l_x)*l_y*image.at<Vec3b>(new_x,new_y+1)[rgb] + (1-l_y)*l_x*image.at<Vec3b>(new_x+1,new_y)[rgb] + l_x*l_y*image.at<Vec3b>(new_x+1,new_y+1)[rgb] );
				}
			}
    	}
    }
	return output;
}

/*Calculate size and tie points for intermediate images*/
void morphAffine(Mat src , float morphs , float ar[3][3] ){

	/*Calculate corners of output image*/
	float temp1_x = ar[0][0]*0 + ar[0][1]*0 + ar[0][2] ; 
	float temp1_y = ar[1][0]*0 + ar[1][1]*0 + ar[1][2] ;
	float temp2_x = ar[0][0]*0 + ar[0][1]*src.cols + ar[0][2] ;
	float temp2_y = ar[1][0]*0 + ar[1][1]*src.cols + ar[1][2] ;
	float temp3_x = ar[0][0]*src.rows + ar[0][1]*0 + ar[0][2] ;
	float temp3_y = ar[1][0]*src.rows + ar[1][1]*0 + ar[1][2] ;
	float temp4_x = ar[0][0]*src.rows + ar[0][1]*src.cols + ar[0][2] ;
	float temp4_y = ar[1][0]*src.rows + ar[1][1]*src.cols + ar[1][2] ;
	
	float t = 1/morphs;
	
	for(int i=1 ; i<=morphs ; i++){
		/*Linear interpolation for intermediate images*/
		int temp1_rows = cvRound((1-i*t)*0 + t*i*temp1_x) ;
		int temp1_cols = cvRound((1-i*t)*0 + t*i*temp1_y) ;
		int temp2_rows = cvRound((1-i*t)*0 + t*i*temp2_x) ;
		int temp2_cols = cvRound((1-i*t)*src.cols + t*i*temp2_y) ;
		int temp3_rows = cvRound((1-i*t)*src.rows + t*i*temp3_x) ;
		int temp3_cols = cvRound((1-i*t)*0 + t*i*temp3_y) ;
		int temp4_rows = cvRound((1-i*t)*src.cols + t*i*temp4_x) ;
		int temp4_cols = cvRound((1-i*t)*src.rows + t*i*temp4_y) ;
		/*Calculating frame size*/
		int rows_s = minimum(temp1_rows , temp2_rows , temp3_rows  , temp4_rows );
		int cols_s = minimum(temp1_cols , temp2_cols , temp3_cols  , temp4_cols );
		int rows_e = maximum(temp1_rows , temp2_rows , temp3_rows  , temp4_rows );
		int cols_e = maximum(temp1_cols , temp2_cols , temp3_cols  , temp4_cols );
		
		Mat output = Mat::zeros(rows_e-rows_s , cols_e-cols_s , src.type());
		float t1[8];
		float x1[4],y1[4];
		/*Offset to shift to origin and setting tie points*/
		x1[0] = temp1_rows-rows_s;
		y1[0] = temp1_cols-cols_s;
		x1[1] = temp2_rows-rows_s;
		y1[1] = temp2_cols-cols_s;
		x1[2] = temp3_rows-rows_s;
		y1[2] = temp3_cols-cols_s;
		x1[3] = temp4_rows-rows_s;
		y1[3] = temp4_cols-cols_s;
		t1[0] = 0;
		t1[1] = 0;
		t1[2] = 0;
		t1[3] = src.cols;
		t1[4] = src.rows;
		t1[5] = 0;
		t1[6] = src.cols;
		t1[7] = src.rows;
		output = tiepoint(output , t1 , x1 , y1 , src , (int)i);
		string s;
		stringstream out;
		out << i;
		s = out.str();
		s.append(".jpg");
		imwrite(s,output);
	}		
}

int main(){
	
	Mat image1;
    Mat image2;
    int morphs=0;
    cout << "Enter Number of Morphs (80-100 recommended): " ;
    cin >> morphs;
    if(morphs<1){
    	cout << "PLease enter a valid input" << endl;
    	return 0;
    }
    int mark=0;
    cout << "Enter 1 for morphing between two images using tie points\nEnter 2 for morphing for affine transformations: " << endl;
    cin >> mark;
    if(mark==2){
    	string im;
    	cout << "Enter filename for Image: <image1>: " ;
    	cin >> im;
    	image1 = imread( im );
		if ( !image1.data ){
		    printf("No image data \n");
		    return -1;
		}
    	float aff[3][3];
    	cout << "Enter 3:3 matrix" << endl;
    	cin >> aff[0][0] >> aff[0][1] >> aff[0][2] ;
    	cin >> aff[1][0] >> aff[1][1] >> aff[1][2] ;
    	cin >> aff[2][0] >> aff[2][1] >> aff[2][2] ;
    	morphAffine(image1,morphs,aff);
    }else if(mark==1){
    	string file1,file2;
    	string im1,im2;
    	cout << "Enter filename for tie points: <file1> <file2> : " ;
    	cin >> file1 >> file2;
    	cout << "Enter filename for Images: <image1> <image2> : " ;
    	cin >> im1 >> im2 ;
    	image1 = imread( im1 );
		if ( !image1.data ){
		    printf("No image data \n");
		    return -1;
		}
		image2 = imread( im2 );
		if ( !image2.data ){
		    printf("No image data \n");
		    return -1;
		}
    	morphImage(image1,image2,morphs,file1,file2);
    }else return 0;
	
	return 0;
}

