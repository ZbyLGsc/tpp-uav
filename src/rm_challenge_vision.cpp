#include "rm_challenge_vision.h"

RMChallengeVision::RMChallengeVision( bool visable )
{
    m_visable = visable;
}

void RMChallengeVision::setVisability( bool visable )
{
    m_visable = visable;
}

/*************************************************
Function:       extractColor
Description:    提取指定颜色的区域
Input:              src:     源图像
                color：选定的颜色
Output:             colorRegion: 提取的颜色区域
Others:
*************************************************/
void RMChallengeVision::extractColor( Mat src, COLOR_TYPE color, Mat& colorRegion )
{
    if ( src.channels() != 3 )
        return;
    /*preprocessing*/
    GaussianBlur( src, src, Size( 5, 5 ), 0, 0 );
    /*to hsv color space*/
    Mat hsv;
    vector< Mat > hsvSplit;
    cvtColor( src, hsv, CV_BGR2HSV );
    split( hsv, hsvSplit );
    equalizeHist( hsvSplit[2], hsvSplit[2] );
    merge( hsvSplit, hsv );

    static int iLowH; /*threshold of  hue*/
    static int iHighH;
    static int iLowS; /*threshold of saturation*/
    static int iHighS;
    static int iLowV; /*threshold of value*/
    static int iHighV;
    static int bgrThresh1; /*threshold of r-g g-r b-g ... depending on the
                              color*/
    static int bgrThresh2; /*threshold of r-b g-r b-g...depending on the color*/

    if ( color == RED )
    {
        iLowH = 0;
        iHighH = 210;
        iLowS = 0;
        iHighS = 255;
        iLowV = 0;
        iHighV = 255;
        bgrThresh1 = 40;
        bgrThresh2 = 30;
    }
    else if ( color == BLUE )
    {
        iLowH = 49;
        iHighH = 155;
        iLowS = 0;
        iHighS = 255;
        iLowV = 0;
        iHighV = 255;
        bgrThresh1 = 20;
        bgrThresh2 = 20;
    }
    else if ( color == GREEN )
    {
        iLowH = 49;
        iHighH = 155;
        iLowS = 0;
        iHighS = 255;
        iLowV = 0;
        iHighV = 255;
        bgrThresh1 = 20;
        bgrThresh2 = 20;
    }

    /*Create a gui to control color threshold*/
    if ( m_visable )
    {
        namedWindow( "Control", CV_WINDOW_AUTOSIZE );
        imshow( "Control", src );
        cvCreateTrackbar( "LowH", "Control", &iLowH,
                          255 ); /*control trackbar of  hue*/
        cvCreateTrackbar( "HighH", "Control", &iHighH, 255 );
        cvCreateTrackbar( "LowS", "Control", &iLowS,
                          255 ); /*control trackbar of saturation*/
        cvCreateTrackbar( "HighS", "Control", &iHighS, 255 );
        cvCreateTrackbar( "LowV", "Control", &iLowV, 255 );
        cvCreateTrackbar( "HighV", "Control", &iHighV, 255 );
        cvCreateTrackbar( "rg", "Control", &bgrThresh1, 255 );
        cvCreateTrackbar( "rb", "Control", &bgrThresh2, 255 );
    }
    /*extract region with needed hsv*/
    inRange( hsv, Scalar( iLowH, iLowS, iLowV ), Scalar( iHighH, iHighS, iHighV ),
             hsv );
    vector< Mat > bgrSplit;
    split( src, bgrSplit );
    Mat bgr1, bgr2, bgr3;
    if ( color == RED )
    {
        bgr1 = bgrSplit.at( 2 );
        bgr2 = bgrSplit.at( 1 );
        bgr3 = bgrSplit.at( 0 );
    }
    else if ( color == BLUE )
    {
        bgr1 = bgrSplit.at( 0 );
        bgr2 = bgrSplit.at( 1 );
        bgr3 = bgrSplit.at( 2 );
    }
    else if ( color == GREEN )
    {
        bgr1 = bgrSplit.at( 1 );
        bgr2 = bgrSplit.at( 0 );
        bgr3 = bgrSplit.at( 2 );
    }
    /*extract region that r>b and |r-b|>threshold */
    Mat large, /*r larger than b or g*/
        abs,   /*|r-b or g|>threshold */
        region1, region2;
    cv::compare( bgr1, bgr2, large, CMP_GT );
    cv::absdiff( bgr1, bgr2, abs );
    cv::threshold( abs, abs, bgrThresh1, 255, THRESH_BINARY );
    cv::bitwise_and( large, abs, region1 );
    /*extract region that r>g and |r-g|>threshold */
    cv::compare( bgr1, bgr3, large, CMP_GT );
    cv::absdiff( bgr1, bgr3, abs );
    cv::threshold( abs, abs, bgrThresh2, 255, THRESH_BINARY );
    cv::bitwise_and( large, abs, region2 );
    /*all region merge together*/
    cv::bitwise_and( region1, region2, colorRegion );
    cv::bitwise_and( colorRegion, hsv, colorRegion );
    if ( m_visable )
    {
        if ( color == RED )
        {
            imshow( "Red", colorRegion );
        }
        else if ( color == BLUE )
        {
            imshow( "Blue", colorRegion );
        }
        else if ( color == GREEN )
        {
            imshow( "Green", colorRegion );
        }
    }
}

/*************************************************
Function:  		imageToReadDistance
Description:	计算已知尺寸物体离相机的水平距离
Input:       		imageLength: 物体在图像中的长度，单位：像素
        imageDistance: 物体在图像中距离图像中心的距离，单位：像素
        realLength: 物体实际的物理尺寸，单位：mm
Output:         	realDistance: 物体实际到相机中心的水平距离，单位：mm
Others:      		只有当已知物体实际尺寸时可用
*************************************************/
float RMChallengeVision::imageToRealDistance( float imageLength, float imageDistance,
                                              float realLength )
{
    float realDistance = ( realLength / (imageLength+0.00001) ) * imageDistance;
    return realDistance / 1000;
}

/*************************************************
Function:  		imageToReadDistance
Description:	计算已知尺寸物体离相机的水平距离
Input:       		imageLength: 物体在图像中的长度，单位：像素
        realLength: 物体实际的物理尺寸，单位：mm
Output:         	realHeight: 物体到相机平面的实际距离，单位：mm
Others:      		只有当已知物体实际尺寸时可用
*************************************************/
float RMChallengeVision::imageToHeight( float imageLength, float realLength )
{
    static float f = 933.0; /*camera parameter*/
    float realHeight = ( realLength / (imageLength+0.00001) ) * f;
    return realHeight / 1000;
}

void RMChallengeVision::detectTriangle( Mat src, Mat color_region, int triangle[4] )
{
    vector< vector< Point > > contours;
    Mat temp;
    color_region.copyTo( temp );
    cv::findContours( temp, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );
    vector< vector< Point > > triangles;
    for ( int k = 0; k < 4; k++ )
        triangle[k] = 0;
    for ( int i = 0; i < contours.size(); i++ )
    {
        int area = contourArea( contours.at( i ) );
        if ( area < 80 )
            continue;
        vector< Point > approx;
        cv::approxPolyDP( Mat( contours[i] ), approx,
                          arcLength( Mat( contours[i] ), true ) * 0.045, true );
        if ( approx.size() != 3 )
            continue;
        if ( !isContourConvex( Mat( approx ) ) )
            continue;
        // fitting 45 45 90 degree of triangle
        float angle1 =
            angle( approx.at( 0 ), approx.at( 1 ), approx.at( 2 ) ) * 57.3;
        float angle2 =
            angle( approx.at( 2 ), approx.at( 0 ), approx.at( 1 ) ) * 57.3;
        float angle3 =
            angle( approx.at( 2 ), approx.at( 1 ), approx.at( 0 ) ) * 57.3;
        bool rightShape = false;
        float angleThreshold = 10.0;
        int verPointId;
        if ( fabs( angle1 - 45 ) < angleThreshold &&
             fabs( angle2 - 45 ) < angleThreshold &&
             fabs( angle3 - 90 ) < angleThreshold )
        {
            rightShape = true;
            verPointId = 0;
        }
        else if ( fabs( angle1 - 45 ) < angleThreshold &&
                  fabs( angle2 - 90 ) < angleThreshold &&
                  fabs( angle3 - 45 ) < angleThreshold )
        {
            rightShape = true;
            verPointId = 1;
        }
        else if ( fabs( angle1 - 90 ) < angleThreshold &&
                  fabs( angle2 - 45 ) < angleThreshold &&
                  fabs( angle3 - 45 ) < angleThreshold )
        {
            rightShape = true;
            verPointId = 2;
        }
        if ( !rightShape )
            continue;
        // judge the direction of the triangle
        Scalar dirColor;  // color indicating direction,red,yellow,blue,green
                          // for right up
                          // left down
        if ( approx.at( verPointId ).x > approx.at( ( verPointId + 1 ) % 3 ).x &&
             approx.at( verPointId ).x >
                 approx.at( ( verPointId + 2 ) % 3 ).x )  // left
        {
            triangle[2] = 1;
            dirColor = Scalar( 255, 0, 0 );
        }
        else if ( approx.at( verPointId ).x <
                      approx.at( ( verPointId + 1 ) % 3 ).x &&
                  approx.at( verPointId ).x <
                      approx.at( ( verPointId + 2 ) % 3 ).x )  // right
        {
            triangle[0] = 1;
            dirColor = Scalar( 0, 0, 255 );
        }
        else if ( approx.at( verPointId ).y <
                      approx.at( ( verPointId + 1 ) % 3 ).y &&
                  approx.at( verPointId ).y <
                      approx.at( ( verPointId + 2 ) % 3 ).y )  // DOWN
        {
            triangle[3] = 1;
            dirColor = Scalar( 0, 255, 0 );
        }
        else if ( approx.at( verPointId ).y >
                      approx.at( ( verPointId + 1 ) % 3 ).y &&
                  approx.at( verPointId ).y >
                      approx.at( ( verPointId + 2 ) % 3 ).y )  // UP
        {
            triangle[1] = 1;
            dirColor = Scalar( 0, 255, 255 );
        }

        // direction vector
        if ( m_visable )
        {
            Mat draw = src.clone();
            cv::drawContours( draw, contours, i, Scalar( 0, 0, 255 ), 2 );
            Point dirVec;
            Point end( approx.at( verPointId ).x, approx.at( verPointId ).y );
            Point start( ( approx.at( ( verPointId + 1 ) % 3 ).x +
                           approx.at( ( verPointId + 2 ) % 3 ).x ) /
                             2,
                         ( approx.at( ( verPointId + 1 ) % 3 ).y +
                           approx.at( ( verPointId + 2 ) % 3 ).y ) /
                             2 );
            dirVec.x = ( end.x - start.x );
            dirVec.y = -( end.y - start.y );
            cv::circle( draw, end, 3, dirColor, 4 );
            cv::circle( draw, start, 3, dirColor, 2 );
            cv::line( draw, end, start, dirColor, 2 );
            triangles.push_back( contours.at( i ) );
            cv::imshow( "triangle", draw );
        }
    }
}

void RMChallengeVision::detectPillarCircle( Mat src, Mat color_region,
                                            bool& circle_found,
                                            Point2f& circle_center, float& radius )
{
    circle_found = false;
    vector< vector< Point > > contours;
    Mat temp;
    color_region.copyTo( temp );
    cv::findContours( temp, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );
    // find circle in all contours
    vector< float > radiuses;
    Mat draw = src.clone();
    for ( int i = 0; i < contours.size(); i++ )
    {
        int area = contourArea( contours.at( i ) );
        if ( area > 900 )
        {
            Point2f center( 320, 240 );
            float radius = 0;
            cv::minEnclosingCircle( contours[i], center, radius );
            int area = contourArea( contours[i], false );
            float circleArea = 3.141592 * radius * radius;
            float r = area / circleArea;
            if ( r > 0.8 )
            {
            	//detect the color of out and in of the circle
            	//if the out is white, meaning origin is red
            	//and the in is black, meaning origin is other colors
            	//then throw the circle
				vector< Point > out_circle_pt;
				vector< Point > in_circle_pt;
				int out_circle_pt_cnt = 0;
				int in_circle_pt_cnt = 0;
				ellipse2Poly( center, Size( radius + 0, radius + 0),
							  0, 0, 360, 4, out_circle_pt);
				ellipse2Poly( center, Size( radius - 5, radius - 5),
							  0, 0, 360, 4, in_circle_pt);
				for ( int j = 0; j < (int)out_circle_pt.size(); j++)
				{
					int x = out_circle_pt.at( j ).x;
					int y = out_circle_pt.at( j ).y;
					if( *(color_region.ptr<uchar>( y ) + x ) == 255 )
						out_circle_pt_cnt++;
				}
				for ( int j = 0; j < (int)in_circle_pt.size(); j++)
				{
					int x = in_circle_pt.at( j ).x;
					int y = in_circle_pt.at( j ).y;
					uchar *data = color_region.ptr<uchar>( y ) + x;
					if( *data == 0 )
						in_circle_pt_cnt++;
				}
				if( out_circle_pt_cnt > 60 && in_circle_pt_cnt > 60)
				{
					cout << "throw a circle"<<endl;
					continue;
				}
				
                cv::drawContours( draw, contours, i, Scalar( 0, 255, 255 ), 2 );
                circle_center = center;
                circle_center.x = center.x - 320;
                circle_center.y = 480 - center.y;
                circle_found = true;
                radiuses.push_back( radius );
            }
        }
    }
    float max = 0;
    int maxId = -1;
    for ( int i = 0; i < radiuses.size(); i++ )
    {
        if ( radiuses.at( i ) > max )
        {
            max = radiuses.at( i );
            maxId = i;
        }
    }
    radius = max;
    // draw circle
    if ( m_visable )
    {
        Point pl( 0, 240 );
        Point pr( 640, 240 );
        Point pu( 320, 0 );
        Point pd( 320, 480 );
        Point cen( 320, 240 );
        Point ccen( 320, 240 );
        ccen = circle_center;
        line( draw, pl, pr, Scalar( 0, 255, 0 ), 1 );
        line( draw, pu, pd, Scalar( 0, 255, 0 ), 1 );
        if ( circle_center.x != 0 && circle_center.y != 0 )
            line( draw, ccen, cen, Scalar( 0, 0, 255 ), 2 );
        cv::imshow( "pillar circle", draw );
    }
}
/**detect all possible pillar in contest field*/
int RMChallengeVision::detectPillar( Mat src, PILLAR_RESULT& pillar_result )
{
    // first detect red pillar
    Mat red_region;
    extractColor( src, RMChallengeVision::RED, red_region );
    detectTriangle( src, red_region, pillar_result.triangle );
    int triangle_sum = pillar_result.triangle[0] + pillar_result.triangle[1] +
                       pillar_result.triangle[2] + pillar_result.triangle[3];
    if ( triangle_sum != 0 )
    {
        detectPillarCircle( src, red_region, pillar_result.circle_found,
                            pillar_result.circle_center, pillar_result.radius );
        ROS_INFO_STREAM( "red pillar" );
        return 1;
    }
    else  // no red triangle found ,detect blue region instead
    {
        Mat blue_region;
        extractColor( src, RMChallengeVision::BLUE, blue_region );
        detectTriangle( src, blue_region, pillar_result.triangle );
        triangle_sum = pillar_result.triangle[0] + pillar_result.triangle[1] +
                       pillar_result.triangle[2] + pillar_result.triangle[3];
        if ( triangle_sum != 0 )
        {
            detectPillarCircle( src, blue_region, pillar_result.circle_found,
                                pillar_result.circle_center, pillar_result.radius );
            ROS_INFO_STREAM( "blue pillar" );
            return 2;
        }
        else
        {
            // nothing in blue either, no pillar
            ROS_INFO_STREAM( "no pillar" );
            return 0;
        }
    }
}
/**p0 angle point,return the cosine of angle*/
float RMChallengeVision::angle( Point pt1, Point pt2, Point pt0 )
{
    float dx1 = pt1.x - pt0.x;
    float dy1 = pt1.y - pt0.y;
    float dx2 = pt2.x - pt0.x;
    float dy2 = pt2.y - pt0.y;
    float cosine =
        ( dx1 * dx2 + dy1 * dy2 ) /
        sqrt( ( dx1 * dx1 + dy1 * dy1 ) * ( dx2 * dx2 + dy2 * dy2 ) + 1e-10 );
    return acos( cosine );
}
/*********************************************************************
			函数名：getYellowRegion
			功能：输入图像src，返回图像dst，黄色区域值为200,
				其他区域根据满足条件多少，值分别为0, 50, 100, 150
				便于显示直观图像调试
				将最后一个参数if_debug赋1后，将通过颜色分别显示H
					SV三通道二值化结果
				对应关系：蓝色对于H通道，绿色对应S通道，红色对应V通道
***************************************************************/
void RMChallengeVision::getYellowRegion( Mat& src, Mat& dst, int LowH, int HighH,
                                         int sThreshold, int vThreshold )
{
    Mat hsv,  Line_sv, hsvColorRegion;
    vector< Mat > hsvSplit;
    GaussianBlur( src, src, Size( 5, 5), 0);
    cvtColor( src, hsv, CV_BGR2HSV_FULL );  //转换成HSV
    split( hsv, hsvSplit );  //分离出HSV通道，用于提取黄色区域
    //threshold( hsvSplit[0], Line_h1, LowH, 255,
    //           THRESH_BINARY );  //去除 H 小于 LowH 度的区域
    //threshold( hsvSplit[0], Line_h2, HighH, 255,
    //           THRESH_BINARY_INV );  //去除 H 大于 HighH 度的区域
	inRange( hsvSplit[0], LowH, HighH, hsvSplit[0]);
    threshold( hsvSplit[1], hsvSplit[1], sThreshold, 255,
               THRESH_BINARY );  //去除 S 小于 sThreshold 的区域
    threshold( hsvSplit[2], hsvSplit[2], vThreshold, 255,
               THRESH_BINARY );  //去除 V 小于 vThreshold 的区域
    //bitwise_and( Line_h1, Line_h2, Line_h );
    bitwise_and( hsvSplit[1], hsvSplit[2], Line_sv );
    bitwise_and( hsvSplit[0], Line_sv, hsvColorRegion );  //矩阵位与（255&255=255）
    // Line_h = Line_h1 + Line_h2;
    // dst = Line_h + hsvSplit[1] + hsvSplit[2];
    if ( m_visable )  //调试用
    {
        merge( hsvSplit, hsv );
        imshow( "getYellowRegion hsv", hsv );
        waitKey( 1 );
    }
	//use bgr to get yellow region
	vector< Mat > bgrSplit;
	Mat large, abs, region1, region2, bgrColorRegion;
	split( src, bgrSplit);
	compare( bgrSplit[1], bgrSplit[0], large, CMP_GT );
	absdiff( bgrSplit[1], bgrSplit[0], abs);
	threshold( abs, abs, 35, 255, THRESH_BINARY);
	bitwise_and( abs, large, region1);
	compare( bgrSplit[2], bgrSplit[0], large, CMP_GT );
	absdiff( bgrSplit[2], bgrSplit[0], abs);
	threshold( abs, abs, 20, 255, THRESH_BINARY);
	bitwise_and( abs, large, region2);
	/*absdiff( bgrSplit[2], bgrSplit[1], abs);
	threshold( abs, region3, 10, 255, THRESH_BINARY_INV);
	imshow( "region3", region3 );
	cv::waitKey( 1 );*/

	bitwise_and( region1, region2, bgrColorRegion);
	//bitwise_and( colorRegion, region3, colorRegion);
	if ( m_visable )
	{
		imshow( "region1 g-b", region1 );
		waitKey( 1 );
		imshow( "region2 r-b", region2 );
		waitKey( 1 );
	}

	bitwise_and(hsvColorRegion, bgrColorRegion, dst);
	//dst = bgrColorRegion;
	if( m_visable )
	{
		imshow("Yellow Region", dst);
	}
}
/**********************************************************
		函数名：detectLine														
		功能：对输入图像拟合直线											
			距离向量结果储存在函数引用参数distance_x,distance_y中			 
			线的单位方向向量储存在引用参数line_vector_x,line_vector_y中				 
			以竖直向上为x轴，水平向右为y轴								 
			if_debug表示是否调试，默认不调试							 
			调试时显示中间状况的窗口以及相关输出						 
			不调试时，仅仅计算距离向量和方向向量并储存					 
****************************************************************/
void RMChallengeVision::detectLine( Mat& src, float& distance_x, float& distance_y,
                                    float& line_vector_x, float& line_vector_y )
{
    Mat img, copy;
    vector< Mat > bgrSplit;
    vector< int > x, y;
    float picture_distance_x, picture_distance_y;  //图片参考系的距离向量
    uchar* data;                               //获取图像数据所用数组
    if ( m_visable )
        split( src, bgrSplit );  //分离出BGR通道，为最终显示结果做准备

    getYellowRegion( src, img, 30, 60, 110, 110 );  //获取黄色区域
    for ( int i = 0; i < src.rows; ++i )            //遍历每一行
    {
        data = img.ptr< uchar >( i );         //获取此行开头指针
        for ( int j = 0; j < src.cols; ++j )  //遍历此行每个元素
        {
            if ( *data == 255 )  //如果刚好满足之前4个条件（255 =255&255&255&255）
            {
                x.push_back( j );  //添加 x 坐标
                y.push_back( i );  //添加 y 坐标
            }
            ++data;  //指到下一个元素
        }
    }

    if ( x.size() > 100 )  //如果有数据
    {
        LeastSquare leastsq( x, y );  //拟合曲线
        leastsq.direction( src.cols / 2, src.rows / 2, picture_distance_x,
                           picture_distance_y );  //获取中心点到直线的向量,图像坐标
        distance_y = picture_distance_x;  //转换为无人机坐标
        distance_x = -picture_distance_y;
        line_vector_y = leastsq.tx;
        line_vector_x = -leastsq.ty;
        if ( m_visable )
        {
            leastsq.print();              //显示结果
            leastsq.draw( bgrSplit[1] );  //绘制图线
            line( bgrSplit[1], Point( src.cols / 2, src.rows / 2 ),
                  Point( src.cols / 2 + picture_distance_x,
                         src.rows / 2 + picture_distance_y ),
                  Scalar( 255 ) );    //以中心点为起点绘制该向量
            merge( bgrSplit, copy );  //加入copy
            imshow( "detectLine", copy );
        }
    }
    else 
	{
		distance_x = 0;
		distance_y = 0;
		line_vector_x = 0;
		line_vector_y = 0;
		
	}
}
/**************************************************************
		函数名：detectLineWithT
		功能： 判断是否有T型，若有，返回true
			若无，返回false，并计算出中心点到直线的距离向量
			距离向量结果储存在函数引用参数distance_x,distance_y中
			线的单位方向向量储存在引用参数line_vector_x,line_vector_y中
			以竖直向上为x轴，水平向右为y轴
			if_debug表示是否调试，默认不调试
			调试时显示中间状况的窗口以及相关输出
			不调试时，仅仅计算距离向量和方向向量并储存
*********************************************************/
bool RMChallengeVision::detectLineWithT( Mat& src, float& distance_x,
                                         float& distance_y, float& line_vector_x,
                                         float& line_vector_y )
{
    Mat img, T_img, copy;//img用于拟合，T_img用于判断
	vector< Mat > bgrSplit;
	vector< int > x, y;//x，y坐标储存vector
	float picture_distance_x, picture_distance_y;//图片参考系的距离向量
	int side = 71;//判断T型的核边长大小
	double val_max;//高斯滤波后的最大值
	Point p_max;//高斯滤波后最大值的位置
	uchar *data;//获取图像数据所用数组

	if( m_visable ) 
		split( src, bgrSplit);	//分离出BGR通道，为最终显示结果做准备
	getYellowRegion( src, img, 30, 60, 110, 110);//获取黄色区域
	for ( int i = 0; i < src.rows; ++i) //遍历每一行
	{
		data = img.ptr<uchar>(i); //获取此行开头指针
		for( int j = 0; j < src.cols; ++j) //遍历此行每个元素
		{
			if(*data == 255) //如果刚好满足之前4个条件（255 =255&255&255&255）
			{
				x.push_back( j );	//添加 x 坐标
				y.push_back( i ); //添加 y 坐标
			}
			++data; //指到下一个元素
		}
	}
	
	//cout <<"test"<<if_has_Tri(T_img,side,val_max,if_debug)<<endl;//另一种判断方式，测试功能
	//threshold(T_img, T_img, val_max/2, 255, THRESH_BINARY);//二值化，便于判断是否有三条边
	
	//if(if_Tri(T_img, p_max.x, p_max.y, side, if_debug))//判断最大点周围是否有三条边，若有，肯定为T型
	if( x.empty() > 100 ) //如果有数据
	{
		Mat element1 = getStructuringElement( MORPH_ELLIPSE,
											  Size(5,5));//设置腐蚀的核大小,5x5的椭圆，即圆
		Mat element2 = getStructuringElement( MORPH_ELLIPSE, 
											  Size(15,15));//设置膨胀的核大小
		erode( img, img, element1);//腐蚀，去除噪点
		dilate( img, img, element2);//膨胀，增加T型交叉点密度
		
		if(m_visable)
		{
			imshow( "T_img pre process", img);
			waitKey( 1 );
		}
		GaussianBlur( img, T_img, Size(side,side), 0);//高斯滤波，计算各点黄色密度
		minMaxLoc( T_img, NULL, &val_max, NULL, &p_max);//得到密度最大点位置和值
		if(hasTri( T_img,side / 2, val_max))
		{
			if( m_visable )
			{
				imshow( "T_img", T_img);
				waitKey( 1 );
			}

			if( m_visable )
			{
				circle( bgrSplit[1], Point( p_max.x, p_max.y),
					    side / 2, Scalar( 255 ) );
				merge( bgrSplit, copy);
				cout << "maxpoint:" << p_max.x << " " << p_max.y << endl;
				imshow( "max point in T_img", copy);
				waitKey( 1 );
			}
			return true;
		}
		else//不是T型，则计算距离向量
		{
			LeastSquare leastsq( x, y); //拟合曲线
			leastsq.direction( src.cols / 2, src.rows / 2, 
							  picture_distance_x, picture_distance_y);	//获取中心点到直线的向量,图像坐标
			distance_y = picture_distance_x;//转换为无人机坐标
			distance_x = -picture_distance_y;
			line_vector_y = leastsq.tx;
			line_vector_x = -leastsq.ty;
			if( m_visable )
			{
				leastsq.print(); //显示结果
				leastsq.draw( bgrSplit[ 1 ] );	//绘制图线	
				line( bgrSplit[1], Point(src.cols / 2, src.rows / 2), 
					  Point(src.cols/2+picture_distance_x, src.rows/2+picture_distance_y), 
					  Scalar( 255 ));	//以中心点为起点绘制该向量
				
				merge( bgrSplit, copy);		//加入copy
				imshow( "detectLineWithT", copy);
				waitKey( 1 );
			}
			return false;
		}
	}
	else 
	{
		distance_x = 0;
		distance_y = 0;
		line_vector_x = 0;
		line_vector_y = 0;
		return false;
	}
}

/**********************************************************
         函数名：getRectSide										     
		   功能：获取以点(x,y)为中心，2r为边长的矩形边缘点（顺时针方向） 
				结果储存在引用参数 side中								
				获取成功时返回true，超出边界时返回false					
				if_debug设为true时，会在原图上绘制值为120点，注意：会改变原图
*********************************************************/
bool RMChallengeVision::getRectSide( Mat& src, vector< uchar >& side, int x, int y,
                                     int r )
{
    if ( ( x >= r ) && ( ( x + r ) < src.cols ) && ( y >= r ) &&
         ( ( y + r ) < src.rows ) )  //是否超边界
    {
        uchar* data = src.ptr< uchar >( y - r );  //获取边缘点值
        for ( int k = x - r; k < x + r; ++k )
        {
            side.push_back( *( data + k ) );
            if ( m_visable )
                if ( *( data + k ) != 255 )
                    *( data + k ) = 120;
        }
        for ( int k = y - r + 1; k < y + r; ++k )
        {
            data = src.ptr< uchar >( k );
            side.push_back( *( data + x + r ) );
            if ( m_visable )
                if ( *( data + x + r ) != 255 )
                    *( data + x + r ) = 120;
        }
        data = src.ptr< uchar >( y + r );
        for ( int k = x + r - 1; k >= x - r; --k )
        {
            side.push_back( *( data + k ) );
            if ( m_visable )
                if ( *( data + k ) != 255 )
                    *( data + k ) = 120;
        }
        for ( int k = y + r - 1; k > y - r; --k )
        {
            data = src.ptr< uchar >( k );
            side.push_back( *( data + x - r ) );
            if ( m_visable )
                if ( *( data + x - r ) != 255 )
                    *( data + x - r ) = 120;
        }

        return true;
    }
    else
        return false;
}
/**************************************************************
		函数名：isTri														
		功能：判断点(x,y)周围r距离是否有三条边								
			有返回true，没有返回false										
			调试时输出边缘相对于左上角的路径长度							
		适用条件：边内部没有空隙											
**********************************************************/
bool RMChallengeVision::isTri( Mat& src, int x, int y, int r )
{
    Mat copy = src.clone();
    int plus_sum = 0, minus_sum = 0;
    vector< uchar > sides;
    if ( getRectSide( copy, sides, x, y, r ) )
    {
        for ( int k = 0; k < ( int )sides.size() - 1; k++ )
            if ( ( sides[k + 1] - sides[k] ) == 255 )
            {
                plus_sum++;
                if ( m_visable )
                    cout << "From:isTri side position:" << k << " +" << plus_sum
                         << endl;
            }
            else if ( ( sides[k + 1] - sides[k] ) == -255 )
            {
                minus_sum++;
                if ( m_visable )
                    cout << "From:isTri side position:" << k << " -" << minus_sum
                         << endl;
            }

        if ( plus_sum == 3 && minus_sum == 3 )
        {
            if ( m_visable )
                imshow( "isTri point", copy );
            waitKey( 1 );
            return true;
        }
        return false;
    }
    else
        return false;
}
/*******************************************************
		函数名：hasTri										 
		功能：判断图片中是否有点周围r距离有三条边				 
		要求：输入高斯滤波后的图和图中最大值					 
		原理：选取所有处于(0.968al_max,val_max]的点		 
			若其中有点周围有三条边，就判断有T型					 
*******************************************************/
bool RMChallengeVision::hasTri( Mat& src, int r, int val_max )
{
    vector< int > x, y;
    //int rand_index;
    Mat img_1, img_2;
    uchar *data;
    static int T_cnt = 0;
    //srand( val_max );
    threshold( src, img_2, val_max / 2, 255,
               THRESH_BINARY );  //获取二值图，便于使用if_Tri函数
    threshold( src, img_1, val_max * 0.98, 255, THRESH_BINARY );
    for ( int i = 0; i < src.rows; ++i) //遍历每一行
	{
		data = img_1.ptr<uchar>(i); //获取此行开头指针
		for( int j = 0; j < src.cols; ++j) //遍历此行每个元素
		{
			if(*data == 255) //如果刚好满足之前4个条件（255 =255&255&255&255）
			{
				x.push_back( j );	//添加 x 坐标
				y.push_back( i ); //添加 y 坐标
			}
			++data; //指到下一个元素
		}
	}
    for ( int i = 0; i < (int)x.size() ; i+=2 )
    {
        //rand_index = rand() % x.size();
        if ( isTri( img_2, x[ i ],
          	 y[ i ], r ) )
        	{
        		++T_cnt;
            	if( T_cnt >= 3)
            	return true;
            }
    }     
    {
    	T_cnt=0;
    	return false;
    }
}

