/*
 * detector.cpp
 *
 *  Created on: Mar 8, 2011
 *      Author: sturm
 */

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_datatypes.h>
#include <articulation_msgs/TrackMsg.h>

using namespace std;

const int MAX_PLANES = 10;

int PLANES = 1;
bool SHOW_IMAGES = true;

int RANSAC_ITERATIONS = 200;
double RANSAC_DISTANCE = 0.01;
int RANSAC_DOWNSAMPLE = 15;
int RANSAC_FITTING_ITERATIONS = 200;

double FILTER_Z = 2;

int COST_INPLANE = 0;
int COST_FREE = 255;
int COST_OCCLUDED = 20;
int COST_UNKNOWN = 20;

double PRIOR_WIDTH = 0.1;
double PRIOR_HEIGHT = 0.1;

double STEP_TRANS = 0.005;
double STEP_ROT = M_PI/180*0.5;
int STEP_FACTOR[] = {8,4,2,1};

double THRESHOLD_PRECISION = 0.7;
double THRESHOLD_RECALL = 0.7;

int CLIP_LEFT = 16;
int CLIP_RIGHT = 16;
int CLIP_TOP = 40;
int CLIP_BOTTOM = 16;

double MAX_DIFF_TRANS = 0.10;
double MAX_DIFF_ROT = M_PI/180*20;
double MAX_DIFF_SIZE = 0.05;

struct Observation {
	geometry_msgs::Transform transform;
	float w;
	float h;
	float precision;
	float recall;
};

struct Track {
	std::vector<Observation> track;
	bool modified;
};

sensor_msgs::CameraInfo::ConstPtr cameraInfo_;
cv_bridge::CvImagePtr cv_ptr;
cv::Mat debugImage_;
cv::Mat tmpImage_;
cv::Mat planeImage[MAX_PLANES];
int planeSize[MAX_PLANES];
cv::Vec3f planeCenter[MAX_PLANES];
cv::Vec3f planeNormal[MAX_PLANES];

ros::Publisher visualization_pub_;
ros::Publisher visualization_array_pub_;
ros::Publisher track_pub_;

std::vector<Track > tracks;

void CountConvexPoly( cv::Mat& img, const cv::Point* v, int npts,int &pixel_value,int &pixel_size,int &pixel_occ)
{
	pixel_value = 0;
	pixel_size = 0;
	pixel_occ = 0;
	int line_type=9;
	int shift=0;
	enum { XY_SHIFT = 16, XY_ONE = 1 << XY_SHIFT, DRAWING_STORAGE_BLOCK = (1<<12) - 256 };
    struct
    {
        int idx, di;
        int x, dx, ye;
    }
    edge[2];

    int delta = shift ? 1 << (shift - 1) : 0;
    int i, y, imin = 0, left = 0, right = 1, x1, x2;
    int edges = npts;
    int xmin, xmax, ymin, ymax;
    uchar* ptr = img.data;
    cv::Size size = img.size();
    cv::Point p0;
    int delta1, delta2;

    if( line_type < CV_AA )
        delta1 = delta2 = XY_ONE >> 1;
    else
        delta1 = XY_ONE - 1, delta2 = 0;

    p0 = v[npts - 1];
    p0.x <<= XY_SHIFT - shift;
    p0.y <<= XY_SHIFT - shift;

    assert( 0 <= shift && shift <= XY_SHIFT );
    xmin = xmax = v[0].x;
    ymin = ymax = v[0].y;

    for( i = 0; i < npts; i++ )
    {
        cv::Point p = v[i];
        if( p.y < ymin )
        {
            ymin = p.y;
            imin = i;
        }

        ymax = std::max( ymax, p.y );
        xmax = std::max( xmax, p.x );
        xmin = MIN( xmin, p.x );

        p.x <<= XY_SHIFT - shift;
        p.y <<= XY_SHIFT - shift;

		cv::Point pt0, pt1;
		pt0.x = p0.x >> XY_SHIFT;
		pt0.y = p0.y >> XY_SHIFT;
		pt1.x = p.x >> XY_SHIFT;
		pt1.y = p.y >> XY_SHIFT;
        p0 = p;
    }

    xmin = (xmin + delta) >> shift;
    xmax = (xmax + delta) >> shift;
    ymin = (ymin + delta) >> shift;
    ymax = (ymax + delta) >> shift;

    if( npts < 3 || xmax < 0 || ymax < 0 || xmin >= size.width || ymin >= size.height )
        return;

    ymax = MIN( ymax, size.height - 1 );
    edge[0].idx = edge[1].idx = imin;

    edge[0].ye = edge[1].ye = y = ymin;
    edge[0].di = 1;
    edge[1].di = npts - 1;

    ptr += img.step*y;

    do
    {
        if( line_type < CV_AA || y < ymax || y == ymin )
        {
            for( i = 0; i < 2; i++ )
            {
                if( y >= edge[i].ye )
                {
                    int idx = edge[i].idx, di = edge[i].di;
                    int xs = 0, xe, ye, ty = 0;

                    for(;;)
                    {
                        ty = (v[idx].y + delta) >> shift;
                        if( ty > y || edges == 0 )
                            break;
                        xs = v[idx].x;
                        idx += di;
                        idx -= ((idx < npts) - 1) & npts;   /* idx -= idx >= npts ? npts : 0 */
                        edges--;
                    }

                    ye = ty;
                    xs <<= XY_SHIFT - shift;
                    xe = v[idx].x << (XY_SHIFT - shift);

                    /* no more edges */
                    if( y >= ye )
                        return;

                    edge[i].ye = ye;
                    edge[i].dx = ((xe - xs)*2 + (ye - y)) / (2 * (ye - y));
                    edge[i].x = xs;
                    edge[i].idx = idx;
                }
            }
        }

        if( edge[left].x > edge[right].x )
        {
            left ^= 1;
            right ^= 1;
        }

        x1 = edge[left].x;
        x2 = edge[right].x;

        if( y >= 0 )
        {
            int xx1 = (x1 + delta1) >> XY_SHIFT;
            int xx2 = (x2 + delta2) >> XY_SHIFT;

            if( xx2 >= 0 && xx1 < size.width )
            {
                if( xx1 < 0 )
                    xx1 = 0;
                if( xx2 >= size.width )
                    xx2 = size.width - 1;
                for(int i=xx1;i<xx2;i++) {
//                	((cv::Vec3f*)ptr)[i] = cv::Vec3f(1,1,0);
                	pixel_size += 1;
                	pixel_value += ptr[i];
                	if(ptr[i]==0) pixel_occ += 1;
                }
//                ICV_HLINE( ptr, xx1, xx2, color, pix_size );
            }
        }

        x1 += edge[left].dx;
        x2 += edge[right].dx;

        edge[left].x = x1;
        edge[right].x = x2;
        ptr += img.step;
    }
    while( ++y <= ymax );
}

cv::Point samplePoint(int step) {
	cv::Point p(0, 0);
	int iter = 0;
	do {
		p = cv::Point(rand() % (tmpImage_.cols/step), rand() % (tmpImage_.rows/step)) * step;
	} while (!isfinite(tmpImage_.at<float> (p)) && iter++ < 10 );//&& (tmpImage_.at<float> (p)>FILTER_Z));
	return (p);
}

inline cv::Point3f to3D(cv::Point a) {
	cv::Point3f pt;
	float constant = 1.00 / cameraInfo_->K[0];
	pt.x = (a.x - (cv_ptr->image.cols >> 1)) * cv_ptr->image.at<float> (a)
			* constant;
	pt.y = (a.y - (cv_ptr->image.rows >> 1)) * cv_ptr->image.at<float> (a)
			* constant;

	pt.z = cv_ptr->image.at<float> (a);
	return pt;
}

inline cv::Point to2D(cv::Point3f pt) {
	cv::Point a;
	a.x = pt.x/ pt.z * cameraInfo_->K[0] +  (cv_ptr->image.cols >> 1);
	a.y = pt.y/ pt.z * cameraInfo_->K[0] +  (cv_ptr->image.rows >> 1);
	return a;
}

bool extractPlanes() {
	ros::Time begin = ros::Time::now();


	tmpImage_ = cv_ptr->image.clone();
	const float bad_point = std::numeric_limits<float>::quiet_NaN ();

	for (int plane=0;plane<PLANES;plane++) {
		int best_count = 0;
		cv::Point3f best_plane_normal(0,0,0);
		float best_plane_d = 0;
		for (int iter = 0; iter < RANSAC_ITERATIONS; iter++) {
			// sample 3 points
			cv::Point p1 = samplePoint(RANSAC_DOWNSAMPLE);
			cv::Point p2 = samplePoint(RANSAC_DOWNSAMPLE);
			cv::Point p3 = samplePoint(RANSAC_DOWNSAMPLE);

			cv::Point3f P1 = to3D(p1);
			cv::Point3f P2 = to3D(p2);
			cv::Point3f P3 = to3D(p3);
			// compute plane normal
			cv::Point3f plane_normal = (P2 - P1).cross(P3 - P1);
			plane_normal *= 1/sqrt((plane_normal.dot(plane_normal)));
			float plane_d = plane_normal.dot(P1);

			if(!isfinite(plane_d)) continue;
			int count = 0;

			for (int y = 0; y < tmpImage_.rows; y+=RANSAC_DOWNSAMPLE) {
				for (int x = 0; x < tmpImage_.cols; x+=RANSAC_DOWNSAMPLE) {
					if (isfinite(tmpImage_.at<float>(cv::Point(x,y))) &&
						(fabs(plane_normal.dot(to3D(cv::Point(x, y))) - plane_d) < RANSAC_DISTANCE)) {
						count++;
					}
				}
			}
			if(count > best_count) {
				best_count = count;
				best_plane_normal = plane_normal;
				best_plane_d = plane_d;
			}
		}
		if(!isfinite(best_plane_d)) return false;

		// recompute model coefficients
		cv::Mat points(tmpImage_.rows/RANSAC_DOWNSAMPLE * tmpImage_.cols/RANSAC_DOWNSAMPLE,3,CV_32FC1);
		cv::Vec3f center;
		int n=0;
		for (int y = 0; y < tmpImage_.rows; y+=RANSAC_DOWNSAMPLE) {
			for (int x = 0; x < tmpImage_.cols; x+=RANSAC_DOWNSAMPLE) {
				if (fabs(best_plane_normal.dot(to3D(cv::Point(x, y))) - best_plane_d)
						< RANSAC_DISTANCE) {
					tmpImage_.at<float>(cv::Point(x,y)) = bad_point;
					cv::Point3f P = to3D(cv::Point(x, y));
					points.at<float>(n,0) =P.x;
					points.at<float>(n,1) =P.y;
					points.at<float>(n,2) =P.z;
					center += cv::Vec3f(P);
					n++;
				}
			}
		}
		points = points.rowRange(0,n);
		center *= 1.0/n;
		planeCenter[plane] = center;
		for(int i=0;i<n;i++) {
			cv::Vec3f d = points.at<cv::Vec3f>(i,0) - center;
			points.at<float>(i,0) = d[0];
			points.at<float>(i,1) = d[1];
			points.at<float>(i,2) = d[2];
		}

		cv::SVD svd(points);
////		cv::Mat w(n,n,CV_32FC3),u(n,n,CV_32FC3),vt(n,n,CV_32FC3);
////		cv::Mat w,u,vt;
////		cv::SVD::compute(points,w,u,vt,CV_SVD_V_T);
//		cout << "w="<< svd.w.size().width<< " " <<svd.w.size().height<< endl;
//		cout << "vt="<< svd.vt.size().width<< " " <<svd.vt.size().height<< endl;
//		cout << "w="<<svd.w<<endl;
//		cout << "vt="<<svd.vt<<endl;
		best_plane_normal.x = svd.vt.at<float>(2,0);
		best_plane_normal.y = svd.vt.at<float>(2,1);
		best_plane_normal.z = svd.vt.at<float>(2,2);
		if(best_plane_normal.dot(cv::Point3f(0,0,1))<0)	// flip towards viewing direction
			best_plane_normal = -best_plane_normal;
		best_plane_d = best_plane_normal.dot( center );

//		cout <<"plane = "<<best_plane_normal<<" "<<best_plane_d<<endl;
//		best_plane_normal.x = 0.855839;
//		best_plane_normal.y = 0.163784;
//		best_plane_normal.z = 0.490627;
//		best_plane_d = 0.386299;

		planeNormal[plane] = best_plane_normal;

//		planeImage[plane] = cv::Mat(cv_ptr->image.size(),CV_8UC1,0);
		planeImage[plane] = cv::Mat::zeros(cv_ptr->image.size(),CV_8UC1);
		planeSize[plane]=0;
//		planeImage[plane].create(cv_ptr->image.size(),CV_8UC1);
		for (int y = 0; y < cv_ptr->image.rows; y++) {
			for (int x = 0; x < cv_ptr->image.cols; x++) {
				float dist = best_plane_normal.dot(to3D(cv::Point(x, y))) - best_plane_d;
				if (fabs(dist) < RANSAC_DISTANCE) {
					debugImage_.at<cv::Vec3f>(cv::Point(x,y))[plane%3] = 1.0 / (1+plane/3);
					planeSize[plane]++;
				}
//				debugImage_.at<cv::Vec3f>(cv::Point(x,y))[0] = 0;
//				debugImage_.at<cv::Vec3f>(cv::Point(x,y))[1] = 0;
//				debugImage_.at<cv::Vec3f>(cv::Point(x,y))[2] = 0;
				if(!isfinite(dist)) {
					// unknown
//					debugImage_.at<cv::Vec3f>(cv::Point(x,y))[2] = 0.5;
					planeImage[plane].at<char>(cv::Point(x,y)) = COST_UNKNOWN;
				} else
				if (dist > RANSAC_DISTANCE) {
					// free
//					debugImage_.at<cv::Vec3f>(cv::Point(x,y))[0] = 0.0;
					planeImage[plane].at<char>(cv::Point(x,y)) = COST_FREE;
				} else
				if (dist < -RANSAC_DISTANCE) {
					// occluded
//					debugImage_.at<cv::Vec3f>(cv::Point(x,y))[0] = 0.5;
					planeImage[plane].at<char>(cv::Point(x,y)) = COST_OCCLUDED;
				} else {
					// in plane
//					debugImage_.at<cv::Vec3f>(cv::Point(x,y))[0] = 1;
					planeImage[plane].at<char>(cv::Point(x,y)) = COST_INPLANE;
				}

			}
		}
		cout <<"inliers="<<best_count<<endl;
	}
//	cout <<"duration = "<< (ros::Time::now() - begin).toSec()<<endl;
	return true;
}

float evalPercept(Observation& percept,int plane,bool draw=false) {
	float score = 0;
	int pixel_value = 0;
	int pixel_occ = 0;
	int pixel_size = 0;


	tf::Transform bulletPose;
	tf::transformMsgToTF(percept.transform,bulletPose);
//	cv::Mat pose = cv::Mat::zeros(4,4,1,CV_32FC1);
//	for(int i=0;i<3;i++) {
//		for(int j=0;j<3;j++) {
//			pose.at<float>(i,j) = bulletPose.getBasis().getRow(i)[j];
//		}
//		pose.at<float>(i,3) = bulletPose.getOrigin()[i];
//	}
//	pose.at<float>()
	tf::Vector3 points3[4];
	points3[0] = bulletPose * tf::Vector3(0,0,0);
	points3[1] = bulletPose * tf::Vector3(percept.w,0,0);
	points3[2] = bulletPose * tf::Vector3(percept.w,percept.h,0);
	points3[3] = bulletPose * tf::Vector3(0,percept.h,0);

	// now project into image
	cv::Point points[4];
	for(int i=0;i<4;i++) {
		points[i] = to2D( cv::Point3f(points3[i][0],points3[i][1],points3[i][2]) );
		if(points[i].x < 0 || points[i].y < 0 || points[i].x >= planeImage[plane].cols || points[i].y >= planeImage[plane].rows) {
			percept.precision = 0;
			percept.recall = 0;
			return 0;
		}
	}

	if(draw) {
		for(int i=0;i<4;i++) {
			cv::line(debugImage_,points[i%4],points[(i+1)%4],cv::Scalar(1,1,1),3);
		}
		cv::circle(debugImage_,points[0],5,cv::Scalar(0,0,1));
	}
	CountConvexPoly(planeImage[plane],points,4,pixel_value,pixel_size,pixel_occ);

	score = 1.0 / pow(pixel_size,0.95) * (pixel_size*255 - pixel_value);
	percept.precision = (pixel_size - pixel_value/255) /(double) pixel_size;
	percept.recall= pixel_occ / (double)planeSize[plane];
	if(draw) {
		cout <<"  score = "<<score<<" pixel_size="<<pixel_size<<" pixel_value="<<(pixel_value/255)<<" planesize="<<planeSize[plane]
             <<" precision="<<percept.precision<<" recall="<<percept.recall<<endl;
	}
//	score = percept.precision * percept.recall;
	return score;
}

bool checkPercept(Observation& percept,int plane) {
	tf::Transform bulletPose;
	tf::transformMsgToTF(percept.transform,bulletPose);
	tf::Vector3 points3[4];
	points3[0] = bulletPose * tf::Vector3(0,0,0);
	points3[1] = bulletPose * tf::Vector3(percept.w,0,0);
	points3[2] = bulletPose * tf::Vector3(percept.w,percept.h,0);
	points3[3] = bulletPose * tf::Vector3(0,percept.h,0);

	// now project into image
	cv::Point points[4];
	for(int i=0;i<4;i++) {
//		if(points3[i][2]>FILTER_Z)
//			return false;
		points[i] = to2D( cv::Point3f(points3[i][0],points3[i][1],points3[i][2]) );
		if(points[i].x < CLIP_LEFT || points[i].y < CLIP_TOP || points[i].x >= planeImage[plane].cols-CLIP_RIGHT || points[i].y >= planeImage[plane].rows-CLIP_TOP) {
			return false;
		}
	}
	return true;
}

float optimizePercept(Observation& percept,int plane) {
	float best_score = evalPercept(percept,plane);
	float score;
	Observation best_percept = percept;
	Observation current_percept = percept;
	tf::Transform pose,poseInc;
	for(int iter=0;iter<RANSAC_FITTING_ITERATIONS;iter++) {
		bool change = false;
		Observation previous_best_percept = best_percept;
		for(int step_size=0;step_size<4;step_size++) {
			for(int step=0;step<10;step++) {
				current_percept = previous_best_percept;
				tf::transformMsgToTF(current_percept.transform,pose);
				float step_trans = STEP_TRANS*STEP_FACTOR[step_size];
				float step_rot = STEP_ROT*STEP_FACTOR[step_size];
				switch(step) {
				case 0: // shrink left
					pose = pose * tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(+step_trans,0,0));
					current_percept.w -= step_trans;
					break;
				case 1: // grow left
					pose = pose * tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(-step_trans,0,0));
					current_percept.w += step_trans;
					break;
				case 2: // shrink top
					pose = pose * tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,+step_trans,0));
					current_percept.h -= step_trans;
					break;
				case 3: // grow top
					pose = pose * tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,-step_trans,0));
					current_percept.h += step_trans;
					break;
				case 4: // shrink right
					current_percept.w -= step_trans;
					break;
				case 5: // grow right
					current_percept.w += step_trans;
					break;
				case 6: // shrink bottom
					current_percept.h -= step_trans;
					break;
				case 7: // grow bottom
					current_percept.h += step_trans;
					break;
				case 8:
					pose = pose * tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),+step_rot),tf::Vector3(0,0,0));
					break;
				case 9:
					pose = pose * tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),-step_rot),tf::Vector3(0,0,0));
					break;
				}
				tf::transformTFToMsg(pose,current_percept.transform);
				score = evalPercept(current_percept,plane);
				if(score>best_score) {
					best_score = score;
					best_percept = current_percept;
					change = true;
				}
			}
			if(change)
				break;
		}
//		cout <<" iter= "<<iter<<" score="<<score<<" precision="<<best_percept.precision<<" recall="<<best_percept.recall<<
//					" w="<<best_percept.w<<" h="<<best_percept.h<<endl;
		if(!change) break;
	}
	percept = best_percept;
	if(percept.precision>THRESHOLD_PRECISION && percept.recall>THRESHOLD_RECALL && checkPercept(best_percept,plane)) {
		score = evalPercept(best_percept,plane,true);
		cout << "plane="<<plane<<" precision="<<percept.precision<<" recall="<<percept.recall<<endl;
		return best_score;
	} else {
		return 0;
	}
}

bool sameObject(Observation& a,Observation& b) {
	if(fabs(a.w - b.w)> MAX_DIFF_SIZE) return false;
	if(fabs(a.h - b.h)> MAX_DIFF_SIZE) return false;
	tf::Transform pa,pb;
	tf::transformMsgToTF(a.transform,pa);
	tf::transformMsgToTF(b.transform,pb);
	tf::Transform diff = pa.inverseTimes(pb);
	if(diff.getOrigin().length() > MAX_DIFF_TRANS) return false;
	if(diff.getRotation().getAngle() > MAX_DIFF_ROT) return false;
	return true;
}

void addToTrack(Observation& percept) {
	bool found = false;
	for(size_t i=0;i<tracks.size();i++) {
		for(size_t j=0;j<tracks[i].track.size();j++) {
			if( sameObject( percept, tracks[i].track[j] ) ) {
				tracks[i].track.push_back(percept);
				tracks[i].modified = true;
				cout <<"add pose observation of object id="<<i<<" (total: "<<tracks[i].track.size()<<" observations)"<<endl;
				found = true;
				break;
			}
		}
		if(found)
			break;
	}
	if(!found) {
		cout <<"new object detected (id="<<tracks.size()<<" w="<<percept.w<<" h="<<percept.h<<endl;
		Track t;
		t.modified = true;
		t.track.push_back(percept);
		tracks.push_back(t);
	}
}

int openChannel(articulation_msgs::TrackMsg &track,std::string name,bool autocreate) {
	// find channel
	size_t i = 0;
	for(; i < track.channels.size(); i++) {
		if(track.channels[i].name == name)
			break;
	}
	// found?
	if( i == track.channels.size() ) {
		if(!autocreate) return -1;
		// create, if not found
		sensor_msgs::ChannelFloat32 ch;
		ch.name = name;
		track.channels.push_back(ch);
	}
	// ensure that channel has right number of elements
	track.channels[i].values.resize( track.pose.size() );
	// return channel number
	return i;
}

void sendTracks(std_msgs::Header header) {
	for(size_t i=0;i<tracks.size();i++) {
		if(!tracks[i].modified)
			continue;
		if(tracks[i].track.size()<5)
			continue;
		articulation_msgs::TrackMsg msg;
		msg.header = header;
		int ch_width = openChannel(msg,"width",true);
		int ch_height = openChannel(msg,"height",true);
		for(size_t j=0;j<tracks[i].track.size();j++) {
			geometry_msgs::Pose pose;
			pose.position.x = tracks[i].track[j].transform.translation.x;
			pose.position.y = tracks[i].track[j].transform.translation.y;
			pose.position.z = tracks[i].track[j].transform.translation.z;
			pose.orientation.x = tracks[i].track[j].transform.rotation.x;
			pose.orientation.y = tracks[i].track[j].transform.rotation.y;
			pose.orientation.z = tracks[i].track[j].transform.rotation.z;
			pose.orientation.w = tracks[i].track[j].transform.rotation.w;
			msg.pose.push_back(pose);
			msg.channels[ch_width].values.push_back(tracks[i].track[j].w);
			msg.channels[ch_height].values.push_back(tracks[i].track[j].h);
		}
		msg.id = i;
		track_pub_.publish(msg);
		tracks[i].modified=false;
	}
}

void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
	cameraInfo_ = msg;
	cout << "received info" << endl;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
	cout << "received image" << endl;
	ros::Time begin = ros::Time::now();
	if (!cameraInfo_)
		return;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//debugImage_ = cv_ptr->image * 0.2;
	cvtColor(cv_ptr->image * 0.2, debugImage_, CV_GRAY2BGR);
	//debugImage_.step.p[2]=4;

	//	for(int y=0;y<debugImage_.rows;y++) {
	//		for(int x=0;x<debugImage_.cols;x++) {
	//			debugImage_.at<float>(y,x,0) = 0;
	//		}
	//	}
	//	cv::circle(debugImage_,cv::Point(10,10),5,cv::Scalar(255),2);

	if(!extractPlanes()) {
		return;
	}

	for(int plane=0;plane<PLANES;plane++) {
		Observation percept;
		// initialize perception object
		tf::Vector3 center(planeCenter[plane][0],planeCenter[plane][1],planeCenter[plane][2]);
		tf::Vector3 z(planeNormal[plane][0],planeNormal[plane][1],planeNormal[plane][2]);
		tf::Vector3 x(1,0,0);
		x = x - x.dot(z)*z;
		tf::Vector3 y = z.cross(x);
		tf::Transform transform;
		transform.setOrigin(center);
		tf::Matrix3x3 basis;
		basis.setValue( x[0],y[0],z[0],x[1],y[1],z[1],x[2],y[2],z[2] );
		tf::Quaternion q;
		basis.getRotation(q);
		transform.setRotation(q);
		transform = transform * tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(-PRIOR_WIDTH/2,-PRIOR_HEIGHT/2,0));

		tf::transformTFToMsg(transform,percept.transform);
		percept.w = PRIOR_WIDTH;
		percept.h = PRIOR_HEIGHT;

		float score = optimizePercept(percept,plane);
		if(score>0) {
			addToTrack(percept);
		}
//	float score = evalPercept(percept,0);
//	cout << "score="<<score<<endl;
//	cout << "precision="<<percept.precision<<endl;
//	cout << "recall="<<percept.recall<<endl;
	}

	cout <<"duration= "<< (ros::Time::now() - begin).toSec()<<endl;

	if(SHOW_IMAGES) {
		cv::imshow("depth", debugImage_);
		cv::waitKey(3);
	}

	sendTracks(msg->header);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "detector");
	ros::NodeHandle nh("~");

	nh.getParam("planes",PLANES);
	nh.getParam("show_images",SHOW_IMAGES);


	nh.getParam("ransac_iterations",RANSAC_ITERATIONS);
	nh.getParam("ransac_distance",RANSAC_DISTANCE);
	nh.getParam("ransac_downsample",RANSAC_DOWNSAMPLE);
	nh.getParam("ransac_fitting_iterations",RANSAC_FITTING_ITERATIONS);
	nh.getParam("filter_z",FILTER_Z);

	nh.getParam("cost_inplane",COST_INPLANE);
	nh.getParam("cost_free",COST_FREE);
	nh.getParam("cost_occluded",COST_OCCLUDED);
	nh.getParam("cost_unknown",COST_UNKNOWN);

	nh.getParam("prior_width",PRIOR_WIDTH);
	nh.getParam("prior_height",PRIOR_HEIGHT);

	nh.getParam("step_trans",STEP_TRANS);
	nh.getParam("step_rot",STEP_ROT);

	nh.getParam("threshold_precision",THRESHOLD_PRECISION);
	nh.getParam("threshold_recall",THRESHOLD_RECALL);

	nh.getParam("clip_left",CLIP_LEFT);
	nh.getParam("clip_right",CLIP_RIGHT);
	nh.getParam("clip_top",CLIP_TOP);
	nh.getParam("clip_bottom",CLIP_BOTTOM);

	nh.getParam("MAX_DIFF_TRANS",MAX_DIFF_TRANS);
	nh.getParam("MAX_DIFF_ROT",MAX_DIFF_ROT);
	nh.getParam("MAX_DIFF_SIZE",MAX_DIFF_SIZE);

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("camera/depth/image", 1, imageCallback);
	ros::Subscriber sub2 = n.subscribe("camera/depth/camera_info", 1,
			infoCallback);
	track_pub_ = n.advertise<articulation_msgs::TrackMsg> ("track", 100);

	if(SHOW_IMAGES)
		cv::namedWindow("depth");

	ros::spin();
}
