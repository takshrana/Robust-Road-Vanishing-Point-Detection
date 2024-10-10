#include <opencv2/opencv.hpp>
#include <iostream>
#include <random>
#include <cmath>

using namespace cv;
using namespace std;


class KalmanFilterVP {
public:
    Point2f v_est;    // Estimated vanishing point
    Mat p_est;        // Covariance matrix
    float q;          // Process noise (variance of prediction)
    float r;          // Measurement noise (variance of observation)

    KalmanFilterVP(const Point2f& initial_vp, float process_noise = 0.1f, float measurement_noise = 10.0f) {
        // Initial state estimate is the initial vanishing point
        v_est = initial_vp;
        // Covariance matrix initialized to identity (uncertainty in x and y)
        p_est = Mat::eye(2, 2, CV_32F);
        q = process_noise;
        r = measurement_noise;
    }

    void reset(float process_noise, float measurement_noise){
        q = process_noise;
        r = measurement_noise;
    }

    // Update Kalman filter with the current observed vanishing point
    Point2f update(const Point2f& v_obs) {
        // Prediction step (assume a constant model)
        Mat F = Mat::eye(2, 2, CV_32F);  // Transition matrix (identity, since no velocity is used)
        Mat Q = q * Mat::eye(2, 2, CV_32F);  // Process noise covariance
        Mat R = r * Mat::eye(2, 2, CV_32F);  // Measurement noise covariance

        // Prediction of the next position
        Mat v_pred = Mat(v_est);  // Prediction based on the previous state
        Mat p_pred = F * p_est * F.t() + Q;  // Update covariance

        // Kalman gain calculation
        Mat H = Mat::eye(2, 2, CV_32F);  // Measurement matrix (identity)
        Mat S = H * p_pred * H.t() + R;  // Innovation covariance
        Mat K = p_pred * H.t() * S.inv();  // Kalman gain

        // Measurement residual (innovation)
        Mat v_obs_mat = Mat(v_obs);  // Current observed vanishing point
        Mat y = v_obs_mat - H * v_pred;

        // Update estimate
        Mat v_new = v_pred + K * y;
        p_est = (Mat::eye(2, 2, CV_32F) - K * H) * p_pred;

        // Store the updated estimate
        v_est = Point2f(v_new.at<float>(0), v_new.at<float>(1));
        return v_est;
    }
};

Point2f calculateAverage(const deque<Point2f>& points) {
    float sum_x = 0, sum_y = 0;
    for (const auto& point : points) {
        sum_x += point.x;
        sum_y += point.y;
    }
    return Point2f(sum_x / points.size(), sum_y / points.size());
}

// Function to compute the median of vanishing points in the buffer
Point2f calculateMedian(vector<Point2f>& points) {
    vector<float> x_vals, y_vals;
    for (const auto& point : points) {
        x_vals.push_back(point.x);
        y_vals.push_back(point.y);
    }

    // Sort the x and y values separately
    sort(x_vals.begin(), x_vals.end());
    sort(y_vals.begin(), y_vals.end());

    // Get the median for x and y
    float median_x = x_vals[x_vals.size() / 2];
    float median_y = y_vals[y_vals.size() / 2];

    return Point2f(median_x, median_y);
}

// Function to calculate Euclidean distance between two points
float calculateDistance(const Point2f& p1, const Point2f& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// Function to apply median filter with threshold-based filtering
Point2f medianFilterWithThreshold(Point2f current_vp, deque<Point2f>& vp_buffer, int buffer_size, float threshold_distance) {
    // If the buffer is not empty, calculate the average vanishing point
    if (!vp_buffer.empty()) {
        Point2f average_vp = calculateAverage(vp_buffer);

        // Calculate the distance from the current VP to the average VP
        float distance = calculateDistance(current_vp, average_vp);

        // Only add the current VP if it's within the threshold distance from the average VP
        if (distance > threshold_distance || current_vp.x<50 || current_vp.x>800 || current_vp.y<50 || current_vp.y> 430) {
            // Skip this vanishing point as it is considered an outlier
            vector<Point2f> vp_list(vp_buffer.begin(), vp_buffer.end());
            return calculateMedian(vp_list);
        }
    }

    // Add the new vanishing point to the buffer
    if(current_vp.x>50 && current_vp.x<800 && current_vp.y>=50 && current_vp.y<=430){
        vp_buffer.push_back(current_vp);


    // If the buffer size exceeds the limit, remove the oldest point
    if (vp_buffer.size() > buffer_size) {
        vp_buffer.pop_front();
    }

    // Calculate the median of the points in the buffer
    vector<Point2f> vp_list(vp_buffer.begin(), vp_buffer.end());
    return calculateMedian(vp_list);
    }
    else{
        return Point2f{854/2, 480/2};
    }
}


int distanceCalculate(int x1, int y1, int x2, int y2)
{
	int x = x1 - x2; //calculating number to square in next step
	int y = y1 - y2;
	int dist;

	dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
	dist = sqrt(dist);                  

	return dist;
}

Point2f find_intersection(const Point2f& p1, const Point2f& p2, const Point2f& p3, const Point2f& p4) {
    float A1 = p2.y - p1.y;
    float B1 = p1.x - p2.x;
    float C1 = A1 * p1.x + B1 * p1.y;
    
    float A2 = p4.y - p3.y;
    float B2 = p3.x - p4.x;
    float C2 = A2 * p3.x + B2 * p3.y;
    
    float det = A1 * B2 - A2 * B1;
    if (det == 0) {
        throw runtime_error("Lines do not intersect");
    }
    
    return Point2f(int((B2 * C1 - B1 * C2) / det), int((A1 * C2 - A2 * C1) / det));
}

float vector_norm(const Point2f& v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

Point2f point_diff(const pair<Point2f, Point2f> p) {
    return Point2f(p.second.x - p.first.x, p.second.y - p.first.y);
}

float angle_between_vectors(const pair<Point2f, Point2f>& v1, const pair<Point2f, Point2f>& v2) {
    Point2f diff_v1 = point_diff(v1);
    Point2f diff_v2 = point_diff(v2);
    
    float dot_product = (diff_v1.x * diff_v2.x) + (diff_v1.y * diff_v2.y);
    float cos_theta = dot_product / (vector_norm(diff_v1) * vector_norm(diff_v2));
    cos_theta = min(max(cos_theta, -1.0f), 1.0f);
    return acos(cos_theta) * 180.0f / CV_PI;
}

Point2f angle_ransac(const vector<pair<Point2f, Point2f>>& motion_vectors, int num_iterations = 45, float max_inlier_angle = 45.0f) {
    Point2f best_vp;
    float best_vp_score = -numeric_limits<float>::infinity();
    
    for (int i = 0; i < num_iterations; ++i) {
        if (motion_vectors.size() < 2) {
            return Point2f();
        }

        int idx1 = rand() % motion_vectors.size();
        int idx2 = rand() % motion_vectors.size();

        while(idx2 == idx1) {
            idx2 = rand() % motion_vectors.size();
        }

        Point2f vp;
        try {
            vp = find_intersection(motion_vectors[idx1].first, motion_vectors[idx1].second, 
                                   motion_vectors[idx2].first, motion_vectors[idx2].second);
        } catch (const runtime_error&) {
            continue;
        }

        float vp_score = 0.0f;
        for (const auto& v : motion_vectors) {
            Point2f base = v.second;
            pair<Point2f, Point2f> u = {vp, base};

            // cout<<u.first<<" "<<u.second<<endl;            
            float theta = angle_between_vectors(v, u);
            // cout<<theta<<endl;
            float score = (theta < max_inlier_angle) ? exp(-abs(theta)) : 0;
            vp_score += score;
        }

        if (vp_score > best_vp_score) {
            best_vp_score = vp_score;
            best_vp = vp;
        }
    }

    return best_vp;
}

vector<pair<Point2f, Point2f>> filter_motion_vectors(vector<pair<Point2f, Point2f>> motion_vectors){
    pair<int, int> center = {854/2, 480/2};
    vector<pair<Point2f, Point2f>> filtered_motion_vectors;
    for (auto vector: motion_vectors){
        int dist_old = distanceCalculate(vector.first.x, vector.first.y, center.first, center.second);
        int dist_new = distanceCalculate(vector.second.x, vector.second.y, center.first, center.second);
    
        double slope =  (vector.second.y - vector.first.y)/(vector.second.x - vector.first.x);
        double theta = abs(atan(slope) * 180/3.1415);

        if (dist_old < dist_new && (theta > 10 && theta < 170)){
            filtered_motion_vectors.push_back(vector);
        }

        
    }
    
    return filtered_motion_vectors;
}

int main() {
    string input_video_path = "/home/nayan/Downloads/Lane Dataset-20241007T100830Z-001/Lane Dataset/Jiqing Expressway Video/IMG_0249.MOV";
    VideoCapture cap(2);
    
    if (!cap.isOpened()) {
        cerr << "Error opening video file." << endl;
        return -1;
    }

    int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    Mat first_frame;
    
    cap.read(first_frame);
    resize(first_frame, first_frame, Size(854, 480));
    Mat gray_first;
    cvtColor(first_frame, gray_first, COLOR_BGR2GRAY);

    vector<Point2f> p0, curr_p0;
    goodFeaturesToTrack(gray_first, p0, 400, 0.005, 20);
    curr_p0 = p0;
    
    Mat gray_prev = gray_first.clone();
    Mat frame;
    int frame_count = 1;

    cout<<gray_first.type()<<" Row "<<gray_first.rows<<"Col : "  << gray_first.cols <<endl;
    cout<<first_frame.type()<<" Row "<<first_frame.rows<<"Col : "  << first_frame.cols <<endl;

    double start = cv::getTickCount();
    double elapsedTime=0;

    Point2f initial_vp(854 / 2, 480 / 2);

    // Kalman filter instance for vanishing point smoothing
    KalmanFilterVP kalman_filter(initial_vp); // Initial value
    bool initialized = false; 

    deque<Point2f> vp_buffer;
    int buffer_size = 30;
    float threshold_distance = 200.0f;

    Point2f previous_vanishing_point = initial_vp;  // Initialize with the center of the frame
    int unchanged_frames = 0;

    while (true) {
        double processTimeStart = cv::getTickCount();

        bool ret = cap.read(frame);
        if (!ret) break;

        resize(frame, frame, Size(854, 480));
        Mat gray_frame;
        cvtColor(frame, gray_frame, COLOR_BGR2GRAY);

        vector<Point2f> p1;
        vector<uchar> status;
        vector<float> err;

        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(gray_prev, gray_frame, curr_p0, p1, status, err, Size(21,21), 4, criteria);

                
        vector<pair<Point2f, Point2f>> motion_vectors;
        vector<Point2f> tracked_p0 ;
        vector<Point2f> tracked_curr_p0 ;
        vector<Point2f> tracked_p1 ;
        for (size_t i = 0; i < p1.size(); ++i) {
            if (status[i]) {
                // cout<<status[i]<<' ';
                // int distance = sqrt(pow(p1[i].x - curr_p0[i].x, 2) + pow(p1[i].y -  curr_p0[i].y, 2));
                int distance = distanceCalculate(curr_p0[i].x, curr_p0[i].y, p1[i].x, p1[i].y);
                if(distance > 2){
                    tracked_p0.push_back(p0[i]);
                    tracked_p1.push_back(p1[i]);
                    tracked_curr_p0.push_back(curr_p0[i]);
                    motion_vectors.push_back({p0[i], p1[i]});
                }
            }
        }
        p0 = tracked_p0;
        p1 = tracked_p1;
        curr_p0 = tracked_curr_p0;

        // Point2f vanishing_point = angle_ransac(motion_vectors);
        // if (vanishing_point != Point2f()) {
        //     circle(frame, vanishing_point, 7, Scalar(0, 0, 255), -1);
        // }
        // cout<<p1.size();
        
        bool detection = true;
        // cout << motion_vectors.size() << endl;
        
        if (motion_vectors.size() < 300){
            detection = false;
            vector<Point2f> new_p0;
            goodFeaturesToTrack(gray_frame, new_p0, 400, 0.005, 20);
            p0.insert(p0.end(), new_p0.begin(), new_p0.end());
            p1.insert(p1.end(), new_p0.begin(), new_p0.end());
        }

        gray_prev = gray_frame.clone();
        curr_p0 = p1;


        if (detection == false){
            cout<<"skipping detection \n";
            // continue;
        }
        else{


        // cout<<"Before Filtering "<<motion_vectors.size()<<endl;
        motion_vectors = filter_motion_vectors(motion_vectors);
        // cout<<"After Filtering "<<motion_vectors.size()<<endl;

        Point2f vanishing_point = angle_ransac(motion_vectors);

        // circle(frame, vanishing_point, 5, Scalar(255, 0, 0), -1);


        if (!initialized && frame_count > 20 ) {
            kalman_filter.reset(0.01F, 200.0F);
            cout<<"initialized"<<endl;
            initialized = true;
            threshold_distance = 100.0f;
            continue;
        }

        vanishing_point = medianFilterWithThreshold(vanishing_point, vp_buffer, buffer_size, threshold_distance);

        // cout<<previous_vanishing_point<<"- Prev "<<vanishing_point<<" - curr "<<endl;
        cout<<"Unchanged - "<<unchanged_frames<<" frame count "<<frame_count<<endl;
        if (initialized && calculateDistance(vanishing_point, previous_vanishing_point) < 1.0f) {
            unchanged_frames++;
        } else {
            unchanged_frames = 0;  // Reset counter if it changes
        }

        // If unchanged for 200 frames, reset the vanishing point
        if (unchanged_frames >= 10 && initialized) {
            // cout<<"reset"<<endl;
            vanishing_point = Point2f(854 / 2, 480 / 2);  // Reset to center
            unchanged_frames = 0;  // Reset the counter
            kalman_filter.reset(0.1F, 0.10F);
            vp_buffer.clear();
            initialized=false;
            threshold_distance=200.0f;
            frame_count = 0;
        }


        

        // Update previous vanishing point
        previous_vanishing_point = vanishing_point;

        vanishing_point = kalman_filter.update(vanishing_point);

        // for(auto point: motion_vectors){
        //     // circle(frame, point.second, 4,  Scalar(255, 255, 255), -1);
        //     line(frame, point.first,  point.second, Scalar(0, 255, 0), 2);
        // }

        line(frame, Point2d(0 , vanishing_point.y), Point2d(854, vanishing_point.y), Scalar(0, 255, 9), 2);
        circle(frame, vanishing_point, 5, Scalar(0, 0, 255), -1);
        // cout<<vanishing_point;

        double currentTime = (cv::getTickCount() - start) / cv::getTickFrequency();
        elapsedTime = currentTime;

        // Calculate the actual FPS during playback
        double actual_fps = frame_count / elapsedTime;
        // string fps = actual_fps - '0';
        putText(frame, to_string(int(actual_fps)), Point(10,30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2);
        
        frame_count++;
        if (initialized){

        if(vanishing_point.y < 170){
            // LOGD("Tilt Up c++");
           cout<<"Tilt Your Phone Up"<<endl;
            // tilt_front_back = 1;
        }
        else if(vanishing_point.y > 300){
            // LOGD("Tilt Down c++");
           cout<<"Tilt Your Phone Down"<<endl;
            // tilt_front_back = 2;
        }
//        else{
//            LOGD("Acive");
//            cout<<"Homography Active"<<endl;
//        }
        if(vanishing_point.x < 300){
           cout<<"Tilt Your Phone Right"<<endl;
            // tilt_right_left = 1;
        }
        else if(vanishing_point.x > 540){
           cout<<"Tilt Your Phone Left"<<endl;
            // tilt_right_left = 2;
        }
        }
        }
        
        line(frame,  Point2d(300, 170), Point2d(300, 300), Scalar(255,0 ,0), 2);
        line(frame,  Point2d(300, 170), Point2d(540, 170), Scalar(255,0 ,0), 2);
        line(frame,  Point2d(300, 300), Point2d(540, 300), Scalar(255,0 ,0), 2);
        line(frame,  Point2d(540, 170), Point2d(540, 300), Scalar(255,0 ,0), 2);
        imshow("Tracking", frame);


        if (waitKey(1) == 'q') break;

    }

    cap.release();
    // destroyAllWindows();
    return 0;
}
