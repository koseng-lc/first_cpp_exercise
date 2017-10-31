#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <iostream>


//#define MIN_DIST

using namespace cv;
using namespace std;

int maks_H=255;
int maks_S=255;
int maks_V=255;
int min_H=0;
int min_S=0;
int min_V=0;

Mat boundaryDetect;

//Subtract buat metode 1
void subtract(Mat input, Mat &display){
    Mat drawing = Mat::zeros(input.size(),CV_8UC1);
    Mat kanvas = Mat::zeros(input.size(),CV_8UC3);
    vector<vector<Point > > contours;
    vector<Vec4i> hierarchy;
    findContours(input,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    vector<vector<Point > > contours_poly(contours.size());
    vector<Rect> bound_rect(contours.size());
    for(size_t i=0;i<contours.size();i++){
        approxPolyDP(contours[i],contours_poly[i],3,true);
        bound_rect[i] = boundingRect(Mat(contours[i]));
    }
    int h=0;
    bool firstContour = true;
    for(size_t i=0;i<contours.size();i++){
        if(contourArea(contours[i])>5){
            if(firstContour){
                drawContours(drawing,contours_poly,i,Scalar(255),CV_FILLED);
                h=i;
                firstContour = false;
            }else{
                drawContours(drawing,contours_poly,i,Scalar(255),CV_FILLED);
                line(drawing,
                     Point(bound_rect[h].tl().x+0.5*bound_rect[h].size().width, bound_rect[h].tl().y+0.5*bound_rect[h].size().height),
                     Point(bound_rect[i].br().x+0.5*bound_rect[i].size().width, bound_rect[h].tl().y+0.5*bound_rect[i].size().height),
                     Scalar(255),2,CV_AA);
                h=i;
            }
        }
    }
    imshow("DEBUG",drawing);
    findContours(drawing,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, Point(0,0));
    //if(!contours.size())return drawing;
    double area=0;
    for(size_t i=0;i<contours.size();i++){
        if(area<contourArea(contours[i])){
            area = contourArea(contours[i]);
            h=i;
        }
    }
    vector<vector<Point > > hull(contours.size());
    convexHull(Mat(contours[h]),hull[0]);
    drawContours(kanvas,hull,0,Scalar::all(255),CV_FILLED);
    //imshow("Kanvas",kanvas);
    divide(255,kanvas,kanvas);
    multiply(kanvas,display,display);
    kanvas.copyTo(boundaryDetect);
}

int yBoundaryValue(Mat &img,int x){
    Mat tImg;
    int bound=0;
    transpose(img,tImg);
    uchar *imgPtr = tImg.ptr<uchar>(x);
    for(int i=0;i<img.cols;i++){
        if(imgPtr[i]>0){
            bound = i;
            break;
        }
    }
    return bound;
}

uchar gradienX(Mat &input,int x, int y){
    uchar *baris_1 = input.ptr<uchar>(y-1);
    uchar *baris_2 = input.ptr<uchar>(y);
    uchar *baris_3 = input.ptr<uchar>(y+1);
    return ((baris_1[x+1])+2*(baris_2[x+1])+(baris_3[x+1])) -( baris_1[x-1] + 2*baris_2[x-1] + baris_3[x-1]);
}

Mat tarik_channel(Mat &img, int nomor_channel){
    Mat hasil(img.size(),CV_8UC1);
    for(int i=0;i<img.rows;i++){
        Vec3b *imgPtr = img.ptr<Vec3b>(i);
        uchar *hasilPtr = hasil.ptr<uchar>(i);
        for(int j=0;j<img.cols;j++){
            hasilPtr[j] = imgPtr[j][nomor_channel];
        }
    }
    return hasil;
}

int main(){
    Mat img = imread("/media/lintang/563C3F913C3F6ADF/Photos/ikut/14191693819104.jpg", CV_LOAD_IMAGE_ANYCOLOR);
    Mat img2 = imread("/media/lintang/563C3F913C3F6ADF/Photos/ikut/14191693819104.jpg", CV_LOAD_IMAGE_ANYCOLOR);
    Mat gray;
    Mat mHSV;
    Mat crop_HSV;
    Mat thresh;
   //==============
    Mat thresh_putih;
    Mat tp_HSV;
    Mat trans_tp;
    Mat target = Mat::zeros(img.size(),CV_8UC1);
    //===========
    Mat crop_thresh;
    Mat gradX_1;
    //Mat gradX_2;
    Mat gradY_luma;
    Mat YCrCb;
    Mat luma;
    //mHSV.convertTo(mHSV,CV_32F);
    //cvtColor(img,gray,CV_BGR2GRAY);
    //adaptiveThreshold(~gray,thresh,255,CV_ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,15,-2);
    if(!img.data||!img2.data){
        std::cout<<"TES"<<std::endl;
        return -1;
    }
    /*Sobel(gray,gradX_1,CV_8U,1,0,3,1,0,BORDER_DEFAULT);
    imshow("gradx",gradX_1);
    imshow("sama",gradX_2);*/
    //Mat abu;
    //cvtColor(img,abu,CV_BGR2GRAY);
    /*Metode 2*/
    cvtColor(img,mHSV,CV_BGR2HSV);
    cvtColor(img,YCrCb,CV_BGR2YCrCb);
    //imshow("CEKAJA",YCrCb);
    inRange(mHSV,Scalar(80,34,144),Scalar(101,117,256),thresh_putih);
    //seleksi tiang(garis vertical)
    erode(thresh_putih,thresh_putih,getStructuringElement(MORPH_RECT,Size(1,img.rows/10)),Point(-1,-1));
    dilate(thresh_putih,thresh_putih,getStructuringElement(MORPH_RECT,Size(1,img.rows/10)),Point(-1,-1));
    imshow("THRP",thresh_putih);
    inRange(mHSV,Scalar(41,190,104),Scalar(87,229,161),thresh);
    imshow("cek",thresh);
    vector<vector<Point > > contours;
    vector<Vec4i > hierarchy;
    findContours(thresh,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    //vector<vector<Point > > contours_poly(contours.size());
    //vector<vector<Point > > hull(contours.size());
    //for(size_t i =0;i < contours.size();i++){
      //  approxPolyDP(contours[i],contours_poly[i],3,true);
        //convexHull(Mat(contours[i]),hull[i],false);
    //}
    vector<Point > titik_kontur;
    vector<Point > titik_hull;
    double area1=0,cx1=0,cy1=0;
    double area2=0,cx2=0,cy2=0;
    for(size_t i=0;i<contours.size();i++){
        Moments m1 = moments(Mat(contours[i]));
        Moments m2 = moments(Mat(contours[(int)i-1<0?0:i-1]));
        area1 = m1.m00;
        cx1 = m1.m10/area1;
        cy1 = m1.m01/area1;
        area2 = m2.m00;
        cx1 = m2.m10/area2;
        cx2 = m2.m01/area2;
        //if((contourArea(contours[i])>200)&&sqrt((cx1-cx2)*(cx1-cx2)+(cy1-cy2)*(cy1-cy2))<img.cols/2){
            for(size_t j=0;j<contours[i].size();j++){
                titik_kontur.push_back(contours[i][j]);
            }
        //}
    }
    convexHull(titik_kontur,titik_hull);
    Mat draw = Mat::zeros(img.size(),CV_8UC1);
    Mat hvs = Mat::zeros(img.size(),CV_8UC3);
    Mat hasil(img.size(),CV_8UC3);
    Mat transHVS(img.rows,img.cols,CV_8UC1);
    polylines(draw,titik_hull,true,Scalar::all(255),2);
    imshow("drw",draw);
    vector<vector<Point > > contours2;
    //vector<Vec4i > hierarchy2;
    int batas_lapangan[img.cols];
    findContours(draw,contours2,hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
    drawContours(hvs,contours2,0,Scalar::all(255),CV_FILLED);
    imshow("HVS",hvs);
    cvtColor(hvs,transHVS,CV_BGR2GRAY);
    transpose(transHVS,transHVS);
    for(int i=0;i<transHVS.rows;i++){
        uchar *thPtr = transHVS.ptr<uchar>(i);
        batas_lapangan[i]=transHVS.cols-1;
        for(int j=0;j<transHVS.cols;j++){
            if(thPtr[j]){
                batas_lapangan[i]=j;
                break;
            }
        }
        cout<<i<<","<<batas_lapangan[i]<<endl;
    }
    divide(255,hvs,hvs);
    multiply(img,hvs,hasil);
    imshow("hsl",hasil);
    //while(waitKey(33!=27)){

    //}return 0;

    int offset=0;
    int txaw1=-1,txak1=-1;
    int txaw2=-1,txak2=-1;
    int txti1=0,txti2=0;
    int ti_ats1=-2,ti_bwh1=-2,ti_ats2=-2,ti_bwh2=-2;
    int tyti1=0,tyti2=0;
    int titik_tengah_gawang_x=0,titik_tengah_gawang_y=0;
    int lebar_tiang1=0,lebar_tiang2=0;
    //transpose(thresh_putih,thresh_putih);
    for(int i=0;i<thresh_putih.cols;i++){
        uchar *tpPtr = thresh_putih.ptr<uchar>((batas_lapangan[i]+offset>=img.rows)?img.rows-1:batas_lapangan[i]+offset);
        if(tpPtr[i]){
            if(txaw1==-1)txaw1=i;
            else if(txaw1!=-1)txak1=i;
        }else if(txak1!=-1){
            break;
        }
    }
    for(int i=thresh_putih.cols-1;i>0;i--){
        uchar *tpPtr = thresh_putih.ptr<uchar>((batas_lapangan[i]+offset>=img.rows)?img.rows-1:batas_lapangan[i]+offset);
        if(tpPtr[i]){
            if(txak2==-1)txak2=i;
            else if(txak2!=-1)txaw2=i;
        }else if(txaw2!=-1){
            break;
        }
    }
    //vector<int > y_garis;
    float c=0;
    //int x0=0,x1=0;
    txti1 = (txaw1 + txak1)/2;
    lebar_tiang1=txak1-txaw1;
    txti2 = (txaw2 + txak2)/2;
    lebar_tiang2=txak2-txaw2;
    cout<<txti1<<","<<txti2<<endl;
    int tengah_1=(batas_lapangan[txaw1]+batas_lapangan[txak1])/2;
    int tengah_2=(batas_lapangan[txaw2]+batas_lapangan[txak2])/2;
    cout<<tengah_2<<endl;
    float gradien = (txak1-txaw1)?((batas_lapangan[txak1] - batas_lapangan[txaw1])/(txak1-txaw1)):1;
    if(txak2==txak1){
        putText(img,"Hanya satu tiang",Point(20,20),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),2);
        //flag ke robot cuman ketemu satu tiang
        goto lewat;
    }
    cout<<gradien<<endl;
    gradien = -1/gradien;
    cout<<gradien<<endl;
    c = (float)tengah_1 - gradien*(float)txti1;
    cout<<-1*c/gradien<<endl;
    line(img,Point((int)((0-c)/gradien),0),Point((int)((img.rows-1-c)/gradien),img.rows-1),Scalar(0,255,0),2);
    //namedWindow("THPTH",CV_WINDOW_NORMAL);
    //imshow("THPTH",thresh_putih);
    transpose(thresh_putih,thresh_putih);
    for(int i=tengah_1;i>=0;i--){
        uchar *t1Ptr = thresh_putih.ptr<uchar>(txti1);
        uchar *t2Ptr = thresh_putih.ptr<uchar>(txti2);
        if(!t1Ptr[i]&&ti_ats1==-2){
            //cout<<"DEBUG"<<endl;
            uchar *cekPtr1 = thresh_putih.ptr<uchar>(txti1+1);
            uchar *cekPtr2 = thresh_putih.ptr<uchar>(txti1-1);
            //cout<<"DEBUG"<<endl;
            if(!cekPtr1[i]||!cekPtr2[i]){
                txti1=(!cekPtr1[i])?txti1+1:(!cekPtr2[i])?txti1-1:txti1;
                ti_ats1=i-1;
                i=tengah_2;
                //cout<<"DEBUG"<<endl;
            }
        }
        else if(!t2Ptr[i]&&ti_ats1!=-2){
            uchar *cekPtr1 = thresh_putih.ptr<uchar>(txti2+1);
            uchar *cekPtr2 = thresh_putih.ptr<uchar>(txti2-1);
            cout<<"DEBUG"<<endl;
            if(!cekPtr1[i]||!cekPtr2[i]){
                txti2=(!cekPtr1[i])?txti2+1:(!cekPtr2[i])?txti2-1:txti2;
                ti_ats2=i-1;
                cout<<"DEBUG"<<endl;
            }
        }
        if(ti_ats1!=-2&&ti_ats2!=-2)break;
    }
    txti1 = (txaw1 + txak1)/2;
    txti2 = (txaw2 + txak2)/2;
    for(int i=tengah_2;i<thresh_putih.cols;i++){
        uchar *t1Ptr = thresh_putih.ptr<uchar>(txti1);
        uchar *t2Ptr = thresh_putih.ptr<uchar>(txti2);
        if(!t1Ptr[i]&&ti_bwh1==-2){
            uchar *cekPtr1 = thresh_putih.ptr<uchar>(txti2+1);
            uchar *cekPtr2 = thresh_putih.ptr<uchar>(txti2-1);
            if(!cekPtr1[i]|!cekPtr2[i]){
                txti1=(!cekPtr1[i])?txti1+1:(!cekPtr2[i])?txti1-1:txti1;
                ti_bwh1=i-1;
                i=tengah_2;
            }
        }
        else if(!t2Ptr[i]&&ti_bwh1!=-2){
            uchar *cekPtr1 = thresh_putih.ptr<uchar>(txti2+1);
            uchar *cekPtr2 = thresh_putih.ptr<uchar>(txti2-1);
            if(!cekPtr1[i]|!cekPtr2[i]){
                txti2=(!cekPtr1[i])?txti2+1:(!cekPtr2[i])?txti2-1:txti2;
                ti_bwh2=i-1;
            }
        }
        if(ti_bwh1!=-2&&ti_bwh2!=-2)break;
    }
    tyti1=(ti_ats1+ti_bwh1)/2;
    tyti2=(ti_ats2+ti_bwh2)/2;
    titik_tengah_gawang_x = (txti1 + txti2)/2;
    titik_tengah_gawang_y = (tyti1 + tyti2)/2;
    circle(img,Point(titik_tengah_gawang_x,titik_tengah_gawang_y),5,Scalar(0,255,0),2);
    rectangle(img,Point(txaw1,ti_ats1),Point(txak1,ti_bwh1),Scalar(0,0,255),2);
    rectangle(img,Point(txaw2,ti_ats2),Point(txak2,ti_bwh2),Scalar(0,0,255),2);
    lewat:
    //imshow("HASIL",hasil);
    imshow("COBA",img);
    /*Akhir algoritma metode 2*/

    /*SKIP*/
    /*Mat mask(img.size(),CV_8UC3);
    Mat res(img.size(),CV_8UC3);
    Mat kertas_gambar1 = Mat::zeros(img.size(),CV_8UC1);
    Mat kertas_gambar2 = Mat::zeros(img.size(),CV_8UC3);
    for(size_t i=0;i<contours.size();i++){
        drawContours(kertas_gambar1,contours,i,Scalar(255,255,255),CV_FILLED);
        if(contourArea(contours[i])>100)
            drawContours(kertas_gambar2,hull,i,Scalar(255,255,255),CV_FILLED);//,8,vector<Vec4i >(),0,Point());
        //rectangle(kertas_gambar1,bound_rect[i].tl(),bound_rect[i].br(),Scalar(255),2,8,0);
        //drawContours(kertas_gambar2,contours_poly,i,Scalar(255),CV_FILLED);
    }
    imshow("BEF",kertas_gambar2);
    divide(255,kertas_gambar2,mask);
    multiply(img,mask,res);
    imshow("RES",res);*/
    //int h=0;
    //bool sekali=true;
    //for(size_t i=0;i<contours.size();i++){
      //  if(contourArea(contours[i])>5){
            //if(sekali){
                //drawContours(kertas_gambar2,contours_poly,i,Scalar(255),CV_FILLED);
                //h=i;
                //sekali=false;
            //}else{
                //drawContours(kertas_gambar2,contours_poly,i,Scalar(255),CV_FILLED);
                //line(kertas_gambar2,Point(bound_rect[h].tl().x+0.5*bound_rect[h].size().width,bound_rect[h].tl().y+0.5*bound_rect[h].size().height),
                     //Point(bound_rect[i].br().x+0.5*bound_rect[i].size().width,bound_rect[i].br().y+0.5*bound_rect[i].size().height),Scalar(255),2,CV_AA);
                //line(kertas_gambar2,Point(bound_rect[h].tl().x,bound_rect[h].tl().y),
                  //   Point(bound_rect[i].tl().x,bound_rect[i].tl().y),Scalar(255),2,CV_AA);
            //}
     //   }
    //}
    //imshow("grey",abu);
    //cornerHarris(abu,target,2,5,0.04,BORDER_DEFAULT);
    //imshow("dst",target);


    while(waitKey(33)!=27){
        //imshow("kg1",kertas_gambar1);
        //imshow("kg2",kertas_gambar2);
    }return 0;

    /*Bagian Threshold*/
    namedWindow("Threshold", CV_WINDOW_NORMAL);
    cvtColor(img,mHSV,CV_BGR2HSV);
    cvtColor(img,YCrCb,CV_BGR2YCrCb);
    imshow("ycrcb",YCrCb);
    tarik_channel(YCrCb,0).copyTo(luma);
    //distanceTransform(luma,gradX_1,CV_DIST_L2,3);
    //luma = tarik_channel(YCrCb,0);
    Sobel(luma,gradY_luma,CV_8UC1,1,0,3,1,0,BORDER_DEFAULT);
    //gradX_1 = Mat::zeros(img.size(),CV_8UC1);
    //for(int i=1;i<img.rows-1;i++){
        //uchar* pix = luma.ptr<uchar>(i);
    //    uchar *gradPtr = gradX_1.ptr<uchar>(i);
    //    for(int j=1;j<img.cols-1;j++){
    //        gradPtr[j] = gradienX(luma,j,i);
    //    }
    //}
    imshow("gradL",gradY_luma);
    //imshow("LUMA",gradX_1);
    imshow("HSV",mHSV);
    //inRange(mHSV,Scalar(65, 50, 65), Scalar(75, 195, 125),thresh);
    //threshold(mHSV,thresh,230,255,CV_THRESH_OTSU);
    //inRange(mHSV,Scalar(54,126,132),Scalar(72,216,205),thresh);
    //imshow("Threshold", thresh);
    namedWindow("IMG",CV_WINDOW_NORMAL);
    imshow("IMG",img);
    namedWindow("Random",CV_WINDOW_NORMAL);
    createTrackbar("maksH","Random",&maks_H,256);
    createTrackbar("maksS","Random",&maks_S,256);
    createTrackbar("maksV","Random",&maks_V,256);
    createTrackbar("minH","Random",&min_H,256);
    createTrackbar("minS","Random",&min_S,256);
    createTrackbar("minV","Random",&min_V,256);
    cvtColor(img,mHSV,CV_BGR2HSV);
    while(1){
        inRange(mHSV,Scalar(min_H, min_S, min_V), Scalar(maks_H, maks_S, maks_V),thresh);
        //adaptiveThreshold(YCrCb,target,maks_H,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,5,10);
        //imshow("AThresh",target);
        imshow("Threshold", thresh);
        if(waitKey(30)==27)return -1;
    }
    /*==============================================*/

    /*Metode Pertama*/
    //subtract(thresh,img2);
    //cvtColor(res,crop_HSV,CV_BGR2HSV);
    cvtColor(img,tp_HSV,CV_BGR2HSV);
    inRange(tp_HSV,Scalar(0,0,202),Scalar(179,20,255),thresh_putih);
    inRange(crop_HSV,Scalar(0, 0, 150), Scalar(179, 100, 255),crop_thresh);
    //inRange(crop_HSV,Scalar(54,126,132),Scalar(72,216,205),crop_thresh);
    imshow("thrp",thresh_putih);
    namedWindow("Disp",CV_WINDOW_NORMAL);
    imshow("Disp", img2);
    //cvtColor(img2,ycrcb,CV_BGR2YCrCb);
    //imshow("Ycrcb",ycrcb);
    erode(crop_thresh,crop_thresh,getStructuringElement(MORPH_RECT,Size(3,3)));
    dilate(crop_thresh,crop_thresh,getStructuringElement(MORPH_RECT,Size(3,3)));

    dilate(crop_thresh,crop_thresh,getStructuringElement(MORPH_RECT,Size(3,3)));
    erode(crop_thresh,crop_thresh,getStructuringElement(MORPH_RECT,Size(3,3)));
    namedWindow("CThr",CV_WINDOW_NORMAL);
    imshow("CThr",crop_thresh);
    Mat htl;
    Mat vtl;
    htl = crop_thresh.clone();
    vtl = crop_thresh.clone();
    int htlSize = htl.cols/60;
    int vtlSize = vtl.rows/10;
    erode(htl,htl,getStructuringElement(MORPH_RECT,Size(htlSize,1)),Point(-1,-1));
    dilate(htl,htl,getStructuringElement(MORPH_RECT,Size(htlSize,1)),Point(-1,-1));
    namedWindow("Horizontal",CV_WINDOW_NORMAL);
    imshow("Horizontal",htl);
    htl.copyTo(vtl);
    erode(vtl,vtl,getStructuringElement(MORPH_RECT,Size(1,vtlSize)),Point(-1,-1));
    dilate(vtl,vtl,getStructuringElement(MORPH_RECT,Size(1,vtlSize)),Point(-1,-1));
    //findContours(...)
    namedWindow("Vertical",CV_WINDOW_NORMAL);
    imshow("Vertical",vtl);
    transpose(vtl,vtl);
    int x1_awal=0,x2_awal=0;
    int y1_awal=0,y2_awal=0;
    int x1_akhir=0,x2_akhir=0;
    int y1_akhir=0,y2_akhir=0;
    int tiang1_x,tiang1_y;
    int tiang2_x,tiang2_y;
    int flag=0;
    int cacah=0;
    int tengah_gawang_x=0,tengah_gawang_y=0;
    for(int i=0;i<vtl.rows;i++){
        uchar *vtlPtr = vtl.ptr<uchar>(i);
        cacah=0;
        for(int j=0;j<vtl.cols;j++){
            //cout<<i<<","<<j<<endl;
            if(vtlPtr[j]&(!flag)){
                x1_awal=i;
                y1_awal=j;
                flag=1;
                cacah++;
            }else if(vtlPtr[j]&flag){
                cacah++;
                x1_akhir=i;
                y1_akhir=j;
            }
        }
        if((!cacah)&flag)break;
    }
    flag=0;
    for(int i=(vtl.rows-1);i>=0;i--){
        uchar *vtlPtr = vtl.ptr<uchar>(i);
        cacah=0;
        for(int j=0;j<vtl.cols;j++){
            if(vtlPtr[j]&(!flag)){
                x2_akhir=i;
                y2_akhir=j;
                flag=1;
                cacah++;
            }else if(vtlPtr[j]&flag){
                cacah++;
                x2_awal=i;
                y2_awal=j;
            }
        }
        if((!cacah)&flag)break;
    }
    if(x2_awal==x1_akhir){
        //umpan balik ke robot cuman ketemu satu tiang
    }
    tiang1_x = (x1_awal + x1_akhir)/2;
    tiang2_x = (x2_awal + x2_akhir)/2;
    tiang1_y = (y1_awal + y1_akhir)/2;
    tiang2_y = (y2_awal + y2_akhir)/2;
    tengah_gawang_x = (tiang1_x + tiang2_x)/2;
    tengah_gawang_y = (tiang1_y + tiang2_y)/2;

    int tinggi_atas1=0;
    int tinggi_bawah1=0;
    int tinggi_atas2=0;
    int tinggi_bawah2=0;
    transpose(thresh_putih,trans_tp);
    imshow("transtp",trans_tp);
    uchar *thpPtr1 = trans_tp.ptr<uchar>(tiang1_x);
    for(int i=tiang1_y;i>=0;i--){
        if(!thpPtr1[i]){
            tinggi_atas1=i;
            break;
        }
    }
    for(int i=tiang1_y;i<img.rows;i++){
        if(!thpPtr1[i]){
            tinggi_bawah1=i;
            break;
        }
    }

    uchar *thpPtr2 = trans_tp.ptr<uchar>(tiang2_x);
    for(int i=tiang2_y;i>=0;i--){
        if(!thpPtr2[i]){
            tinggi_atas2=i;
            break;
        }
    }
    for(int i=tiang2_y;i<img.rows;i++){
        if(!thpPtr2[i]){
            tinggi_bawah2=i;
            break;
        }
    }
    //line(img,Point(20,20),Point(20,50),Scalar(0,255,0),2);
    line(img,Point(tiang1_x,tinggi_atas1),Point(tiang1_x,tinggi_bawah1),Scalar(0,255,0),2);
    line(img,Point(tiang2_x,tinggi_atas2),Point(tiang2_x,tinggi_bawah2),Scalar(0,255,0),2);
    circle(img,Point(tengah_gawang_x,tengah_gawang_y-20),7,Scalar(0,255,0),3);
    //rectangle(img,Rect(x1_awal,y1_awal,x1_akhir,y1_akhir),Scalar(0,255,0),2);
    rectangle(img,Point(x1_awal,y1_awal),Point(x1_akhir,y1_akhir),Scalar(0,0,255),2);
    rectangle(img,Point(x2_awal,y2_awal),Point(x2_akhir,y2_akhir),Scalar(0,0,255),2);
    /*Mat paper;
    int count=0;
    vector<vector< Point> > contours;
    vector<Vec4i > hierarchy;
    findContours(vtl,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    vector<vector< Point> > contoursPoly(contours.size());
    for(size_t i=0;i<contours.size();i++){
        if(contourArea(contours[i])>50){
            approxPolyDP(contours[i],contoursPoly[i],3,true);
            count++;
        }
    }*/
    //for(int i=0;i<count;)

    //=======================================
    /*int p1_awal;
    int p2_akhir;
    for(int i=0;i<vtl.rows;i++){
        uchar *vtlPtr = vtl.ptr<uchar>(i);
        for(int j=0;j<vtl.cols;j++){
            if(vtlPtr[j]){
                p1_awal=i;
            }
        }
    }*/


    //=======================================
    /*vector<Vec4i> lines;
    vector<Vec4i> arrange;
    HoughLinesP(vtl,lines,1,CV_PI/180,50,50,10);
    Vec4i t1,t2;
    int tiang_1=-1,tiang_2=-1;
    float tempLength=0;
    float length[lines.size()];
    for(size_t i=0;i<lines.size();i++){
        Vec4i l = lines[i];
        length[i] = sqrt((l[2]-l[0])*(l[2]-l[0])+(l[3]-l[1])*(l[3]-l[1]));
        //cout<<l[0]<<","<<l[1]<<","<<l[2]<<","<<l[3]<<endl;
        //if((l[2]-l[0]>20&&l[2]-l[0]<80)&&(l[3]-l[1]>120&&l[3]-l[1]<150)){
         line(img,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(0,255,0),2);
        //}
    }
    for(size_t i=0;i<lines.size();i++){
        for(size_t j=0;j<lines.size()-1;j++)
            if(length[i]>length[i+1]){
                tempLength = length[i+1];
                length[i+1] = length[i];
                length[i] = tempLength;
        }
    }
    line(src,Point(t1[0],t1[1]),Point(t1[2],t1[3]),Scalar(0,255,0),2);*/
    namedWindow("After",CV_WINDOW_NORMAL);
    imshow("After", img);
    waitKey(0);
    destroyAllWindows();
    return 0;
}
